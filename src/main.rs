#![no_std]
#![no_main]

use bsp::hal::Clock;
use rp_pico as bsp;

use bsp::entry;
use bsp::hal;
use bsp::hal::pac::interrupt;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};
use usb_device::{bus::UsbBusAllocator, device::UsbDevice};
use usbd_hid::descriptor::{MouseReport, SerializedDescriptor};
use usbd_hid::hid_class::HIDClass;

const VID: u16 = 0x16c0;
const PID: u16 = 0x27da;

static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

/// This function is called whenever the USB Hardware
/// generates an Interrupt Request
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}

fn push_mouse_movement(report: MouseReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|_| unsafe { USB_HID.as_mut().map(|hid| hid.push_input(&report)) })
        .unwrap()
}

#[entry]
fn main() -> ! {
    info!("Program start");

    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Configure USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        USB_BUS = Some(usb_bus);
    }

    // Grab reference to the USB Bus allocator
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Configure USB HID Class Device driver, providing mouse reports
    let usb_hid = HIDClass::new(bus_ref, MouseReport::desc(), 60);
    unsafe {
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(VID, PID))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake Company")
            .product("Twitchy Mouse")
            .serial_number("TEST")])
        .unwrap()
        .device_class(0)
        .build();
    unsafe {
        USB_DEVICE = Some(usb_dev);
        hal::pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    }

    let core = hal::pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    loop {
        delay.delay_ms(120000);

        let report_up = MouseReport {
            x: 0,
            y: 4,
            buttons: 0,
            wheel: 0,
            pan: 0,
        };
        push_mouse_movement(report_up).ok().unwrap_or(0);

        delay.delay_ms(100);

        let report_down = MouseReport {
            x: 0,
            y: -4,
            buttons: 0,
            wheel: 0,
            pan: 0,
        };
        push_mouse_movement(report_down).ok().unwrap_or(0);
    }
}

// End of file
