//! First native Intel Wi-Fi attachment through the original PCI/DMA owner.
//! A PCIe function-level reset stops the endpoint before its original root
//! drains payload writes and binds a private domain. No firmware or rings are
//! published here. Radios without FLR are refused rather than assumed stopped.
use crate::{
    block::BlockError,
    bus::pci::{
        power::Timer,
        requester::preparation::{self, Attachment, PreparingRequester},
    },
};
use alloc::rc::Rc;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    Pci(BlockError),
    NoFunctionReset,
    Preparation(preparation::Error),
}

/// Original requester, BAR reservation and physical minimum-delay source.
pub struct Inputs<P> {
    pub requester: PreparingRequester,
    pub registers: P,
    pub minimum: Rc<dyn Timer>,
}

/// Both the original function and its new attachment must stay with the radio.
pub struct Bound<P> {
    pub function: crate::bus::pci::Function<crate::bus::pci::owned::OwnedConfig>,
    pub attachment: Attachment,
    pub registers: P,
    pub minimum: Rc<dyn Timer>,
}

/// Retain the complete original graph on an error, unwind or abandoned attempt.
struct Owner<T>(Option<T>);
impl<T> Drop for Owner<T> {
    fn drop(&mut self) {
        core::mem::forget(self.0.take());
    }
}

pub struct Failure<P> {
    pub error: Error,
    _owner: Owner<Inputs<P>>,
    _registers: Owner<(P, Rc<dyn Timer>)>,
    _preparation: Option<preparation::Failure>,
}

/// Stop this function, drain its old DMA root, then bind its private domain.
///
/// # Safety
/// `inputs` keeps the original physical Intel Wi-Fi function in D0 and its
/// exclusively reserved BAR. Prior firmware, interrupt/work, submissions and
/// competing configuration/reset owners are excluded; bypass/peer DMA is
/// independently excluded or drained. This is the first Stela attach, with no
/// Stela DMA allocations or routes. The function's advertised FLR resets all
/// its endpoint engines without resetting a sibling or changing its assignment.
pub unsafe fn reset_and_bind<P>(inputs: Inputs<P>) -> Result<Bound<P>, Failure<P>> {
    let mut owner = Owner(Some(inputs));
    let reset = (|| {
        let inputs = owner.0.as_mut().expect("original Intel requester");
        inputs.minimum.check().map_err(|e| Error::Pci(e.into()))?;
        let function = inputs.requester.function_mut();
        // Contain PCI delivery while FLR waits for inherited requests. PCI
        // containment alone is not an endpoint stop or payload-drain receipt.
        function.abandon().map_err(Error::Pci)?;
        if !function
            .function_level_reset(inputs.minimum.as_ref())
            .map_err(Error::Pci)?
        {
            return Err(Error::NoFunctionReset);
        }
        Ok(())
    })();
    if let Err(error) = reset {
        return Err(Failure {
            error,
            _owner: owner,
            _registers: Owner(None),
            _preparation: None,
        });
    }
    let Inputs {
        requester,
        registers,
        minimum,
    } = owner.0.take().expect("reset original Intel requester");
    // Keep BAR/timer custody even if root finalization unwinds.
    let mut kept = Owner(Some((registers, minimum)));
    // SAFETY: completed same-function FLR stopped every endpoint engine;
    // original submission/IRQ/work/bypass exclusions remain held. Finalization
    // drains the original root and checks PCI containment before attachment.
    let bound = unsafe { requester.acknowledge_quiescence() }.finalize();
    match bound {
        Ok(bound) => {
            let (registers, minimum) = kept.0.take().expect("original BAR and timer");
            Ok(Bound {
                function: bound.requester.into_function(),
                attachment: bound.attachment,
                registers,
                minimum,
            })
        }
        Err(failure) => {
            // The preparation failure already keeps its original requester/root.
            Err(Failure {
                error: Error::Preparation(failure.error),
                _owner: Owner(None),
                _registers: kept,
                _preparation: Some(failure),
            })
        }
    }
}

#[cfg(test)]
mod tests;
