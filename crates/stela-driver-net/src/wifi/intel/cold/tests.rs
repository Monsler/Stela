extern crate std;
use super::*;
use crate::bus::{
    dma::mapping::DmaError,
    pci::{
        Address, ConfigSpace, Function,
        owned::test_support::{fixture, fixture_pointer},
        power, regs,
        requester::{
            FunctionRole, RequesterBinding,
            preparation::{AttachmentBackend, Root},
        },
    },
};
use alloc::{boxed::Box, vec::Vec};
use core::{
    cell::{Cell, RefCell},
    ptr::NonNull,
};

const ADDRESS: Address = Address {
    segment: 0,
    bus: 4,
    device: 0,
    function: 0,
};
const DEVICE_CONTROL: u16 = 0x48;

#[derive(Default)]
struct State {
    events: RefCell<Vec<&'static str>>,
    released: Cell<usize>,
    missing_timer: Cell<bool>,
    fail_reset_wait: Cell<bool>,
    fail_drain: Cell<bool>,
    panic_drain: Cell<bool>,
}
struct Registers(Rc<State>);
impl Drop for Registers {
    fn drop(&mut self) {
        self.0.released.set(self.0.released.get() + 1);
    }
}
struct Minimum {
    state: Rc<State>,
    control: NonNull<u16>,
    command: NonNull<u16>,
}
// SAFETY: isolated synchronous PCI fixture; waiting models completed FLR.
unsafe impl Timer for Minimum {
    fn check(&self) -> Result<(), power::Error> {
        if self.state.missing_timer.get() {
            Err(power::Error::NotReady)
        } else {
            Ok(())
        }
    }
    fn wait_us(&self, microseconds: u64) -> Result<(), power::Error> {
        assert_eq!(microseconds, 100_000);
        // SAFETY: these are the same fixture's exclusively kept RAM registers.
        unsafe {
            assert_eq!(
                self.command.as_ptr().read_volatile() & regs::COMMAND_MASTER,
                0
            );
            assert_ne!(self.control.as_ptr().read_volatile() & 0x8000, 0);
        }
        self.state.events.borrow_mut().push("FLR completion wait");
        if self.state.fail_reset_wait.get() {
            return Err(power::Error::NotReady);
        }
        // SAFETY: model the self-clearing Initiate FLR bit in the owned fixture.
        unsafe {
            self.control.as_ptr().write_volatile(0);
        }
        Ok(())
    }
}
struct OldRoot(Rc<State>);
impl Drop for OldRoot {
    fn drop(&mut self) {
        self.0.released.set(self.0.released.get() + 1);
    }
}
// SAFETY: isolated idle fixture. This root deliberately refuses binding;
// shared preparation tests cover successful domain attachment and retirement.
unsafe impl Root for OldRoot {
    fn drain(&self) -> Result<(), DmaError> {
        self.0.events.borrow_mut().push("old root drain");
        assert!(!self.0.panic_drain.get(), "injected root unwind");
        if self.0.fail_drain.get() {
            Err(DmaError::InvalidationFailed)
        } else {
            Ok(())
        }
    }
    unsafe fn bind(
        &self,
        binding: RequesterBinding,
    ) -> Result<Box<dyn AttachmentBackend>, DmaError> {
        assert_eq!(binding.address(), ADDRESS);
        binding.check_live().unwrap();
        self.0.events.borrow_mut().push("new domain bind");
        Err(DmaError::MappingFailed)
    }
}

fn inputs(flr: bool) -> (Inputs<Registers>, Rc<State>) {
    let (mut config, _) = fixture(ADDRESS);
    config.write32(0, 0x2725_8086).unwrap();
    config.write32(8, 0x0280_001a).unwrap();
    config
        .write16(
            regs::COMMAND,
            regs::COMMAND_INTX_DISABLE | regs::COMMAND_MEMORY,
        )
        .unwrap();
    config.write16(regs::STATUS, regs::STATUS_CAP_LIST).unwrap();
    config.write8(0x34, 0x40).unwrap();
    config.write32(0x40, 0x0002_0010).unwrap();
    config.write32(0x44, if flr { 1 << 28 } else { 0 }).unwrap();
    config.write32(0x100, 0).unwrap();
    let state = Rc::new(State::default());
    let minimum = Rc::new(Minimum {
        state: state.clone(),
        control: fixture_pointer(&config, DEVICE_CONTROL).cast(),
        command: fixture_pointer(&config, regs::COMMAND).cast(),
    });
    let function = Function::new(ADDRESS, config).unwrap();
    // SAFETY: same exclusively held RAM fixture/root; no real DMA/IRQ/work.
    let requester = unsafe {
        PreparingRequester::new(
            function,
            FunctionRole::Physical,
            Box::new(OldRoot(state.clone())),
        )
    }
    .unwrap_or_else(|_| panic!("original fixture requester"));
    (
        Inputs {
            requester,
            registers: Registers(state.clone()),
            minimum,
        },
        state,
    )
}

#[test]
fn ax210_flr_completes_before_old_root_drain_and_new_domain_binding() {
    let (inputs, state) = inputs(true);
    // SAFETY: original standalone AX210-shaped synchronous fixture.
    let failure = unsafe { reset_and_bind(inputs) }.err().unwrap();
    assert_eq!(
        failure.error,
        Error::Preparation(preparation::Error::Dma(DmaError::MappingFailed))
    );
    assert_eq!(
        &*state.events.borrow(),
        &["FLR completion wait", "old root drain", "new domain bind"]
    );
    drop(failure);
    assert_eq!(
        state.released.get(),
        0,
        "failed bind keeps original root and BAR"
    );
}

#[test]
fn missing_flr_cannot_issue_a_stop_receipt_or_bind_a_domain() {
    let (inputs, state) = inputs(false);
    // SAFETY: isolated fixture with no advertised FLR.
    let failure = unsafe { reset_and_bind(inputs) }.err().unwrap();
    assert_eq!(failure.error, Error::NoFunctionReset);
    assert!(state.events.borrow().is_empty());
    drop(failure);
    assert_eq!(state.released.get(), 0);
}

#[test]
fn unavailable_minimum_refuses_before_any_pci_effect() {
    let (mut inputs, state) = inputs(true);
    inputs
        .requester
        .function_mut()
        .config()
        .write16(DEVICE_CONTROL, 0x40)
        .unwrap();
    let control =
        fixture_pointer(inputs.requester.function_mut().config(), DEVICE_CONTROL).cast::<u16>();
    state.missing_timer.set(true);
    // SAFETY: isolated original PCI/root fixture.
    let failure = unsafe { reset_and_bind(inputs) }.err().unwrap();
    assert_eq!(
        failure.error,
        Error::Pci(BlockError::PciPower(power::Error::NotReady))
    );
    // SAFETY: failure still keeps this exact model register allocation.
    assert_eq!(unsafe { control.as_ptr().read_volatile() }, 0x40);
    assert!(state.events.borrow().is_empty());
    drop(failure);
    assert_eq!(state.released.get(), 0);
}

#[test]
fn failed_reset_completion_does_not_drain_or_bind_and_keeps_owners() {
    let (inputs, state) = inputs(true);
    state.fail_reset_wait.set(true);
    // SAFETY: isolated original PCI/root fixture.
    let failure = unsafe { reset_and_bind(inputs) }.err().unwrap();
    assert_eq!(
        failure.error,
        Error::Pci(BlockError::PciPower(power::Error::NotReady))
    );
    assert_eq!(&*state.events.borrow(), &["FLR completion wait"]);
    drop(failure);
    assert_eq!(state.released.get(), 0);
}

#[test]
fn failed_old_root_drain_cannot_bind_and_keeps_the_reset_bar() {
    let (inputs, state) = inputs(true);
    state.fail_drain.set(true);
    // SAFETY: isolated original PCI/root fixture.
    let failure = unsafe { reset_and_bind(inputs) }.err().unwrap();
    assert_eq!(
        failure.error,
        Error::Preparation(preparation::Error::Dma(DmaError::InvalidationFailed))
    );
    assert_eq!(
        &*state.events.borrow(),
        &["FLR completion wait", "old root drain"]
    );
    drop(failure);
    assert_eq!(state.released.get(), 0);
}

#[test]
fn root_unwind_keeps_original_bar_timer_and_requester() {
    let (inputs, state) = inputs(true);
    state.panic_drain.set(true);
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        // SAFETY: isolated original PCI/root fixture.
        unsafe { reset_and_bind(inputs) }
    }));
    assert!(result.is_err());
    assert_eq!(
        &*state.events.borrow(),
        &["FLR completion wait", "old root drain"]
    );
    assert_eq!(state.released.get(), 0);
}
