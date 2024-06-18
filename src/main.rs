#![no_std]
#![no_main]

mod st7789;

extern crate alloc;
slint::include_modules!();

use alloc::{boxed::Box, rc::Rc};
use core::cell::RefCell;
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
    peripherals::DMA1_CH3,
    peripherals::SPI3,
    spi::{self, Spi},
    time::mhz,
};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::Delay;
use embedded_alloc::Heap;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    pixelcolor::{raw::RawU16, Rgb565, RgbColor},
    prelude::*,
    primitives::Rectangle,
};
use panic_probe as _;
use slint::platform::software_renderer::MinimalSoftwareWindow;
use st7789::ST7789;
use static_cell::StaticCell;

const HEAP_SIZE: usize = 200 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

struct MyPlatform {
    window: Rc<MinimalSoftwareWindow>,
}

impl slint::platform::Platform for MyPlatform {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(embassy_time::Instant::now().as_millis())
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        defmt::info!("{}", defmt::Debug2Format(&arguments));
    }
}

static SPI_BUS: StaticCell<NoopMutex<RefCell<Spi<SPI3, DMA1_CH3, NoDma>>>> = StaticCell::new();
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize the heap allocator, peripheral devices and other things.
    let p = embassy_stm32::init(Default::default());

    // Initialize heap
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

    // Initialize a window (we'll need it later).
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );

    slint::platform::set_platform(Box::new(MyPlatform {
        window: window.clone(),
    }))
    .unwrap();

    const DISPLAY_WIDTH: usize = 320;
    let mut line_buffer = [slint::platform::software_renderer::Rgb565Pixel(0); DISPLAY_WIDTH];

    // Setup the UI.
    let _ui = AppWindow::new().unwrap();
    // // ... setup callback and properties on `ui` ...

    // // Make sure the window covers our entire screen.
    window.set_size(slint::PhysicalSize::new(320, 172));

    let mut spi_config = spi::Config::default();
    spi_config.frequency = mhz(64);
    let spi = spi::Spi::new_txonly(p.SPI3, p.PB3, p.PB5, p.DMA1_CH3, NoDma, spi_config);
    let spi_bus = SPI_BUS.init(NoopMutex::new(RefCell::new(spi)));
    let cs = Output::new(p.PB9, Level::High, Speed::VeryHigh);
    let spi_device = SpiDevice::new(spi_bus, cs);
    let dc = Output::new(p.PB7, Level::High, Speed::VeryHigh);
    let mut display = ST7789::<_, _, 320, 172, 0, 34>::new(spi_device, dc);

    display.init(&mut Delay);
    display.clear(Rgb565::BLACK).unwrap();

    loop {
        slint::platform::update_timers_and_animations();

        // Draw the scene if something needs to be drawn.
        window.draw_if_needed(|renderer| {
            // see next section about rendering.
            renderer.render_by_line(DisplayWrapper {
                display: &mut display,
                line_buffer: &mut line_buffer,
            });
        });

        if !window.has_active_animations() {
            if let Some(duration) = slint::platform::duration_until_next_timer_update() {
                embassy_time::Timer::after_millis(duration.as_millis() as u64).await;
                continue;
            }
        }
        embassy_time::Timer::after_millis(10).await;
    }
}

struct DisplayWrapper<'a, T> {
    display: &'a mut T,
    line_buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}
impl<T: DrawTarget<Color = embedded_graphics::pixelcolor::Rgb565>>
    slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, T>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        // Render into the line
        render_fn(&mut self.line_buffer[range.clone()]);

        // Send the line to the screen using DrawTarget::fill_contiguous
        self.display
            .fill_contiguous(
                &Rectangle::new(
                    Point::new(range.start as _, line as _),
                    Size::new(range.len() as _, 1),
                ),
                self.line_buffer[range.clone()]
                    .iter()
                    .map(|p| RawU16::new(p.0).into()),
            )
            .map_err(drop)
            .unwrap();
    }
}
