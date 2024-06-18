#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
mod st7789;

extern crate alloc;
slint::include_modules!();

use alloc::{boxed::Box, rc::Rc, vec::Vec};
use core::cell::RefCell;
use defmt::*;
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::{
    dma::NoDma,
    gpio::{Level, Output, Speed},
    peripherals::{DMA1_CH1, PC5, SPI1},
    spi::{self, Spi},
    time::{mhz, Hertz},
    Config,
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
use slint::{platform::software_renderer::MinimalSoftwareWindow, Model as _};
use st7789::ST7789;
use static_cell::StaticCell;

const HEAP_SIZE: usize = 110 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

struct MyPlatform<'a> {
    window: Rc<MinimalSoftwareWindow>,
    display: RefCell<
        ST7789<
            SpiDevice<
                'a,
                embassy_sync::blocking_mutex::raw::NoopRawMutex,
                Spi<'a, SPI1, DMA1_CH1, NoDma>,
                Output<'a, embassy_stm32::peripherals::PB1>,
            >,
            Output<'a, PC5>,
            320,
            172,
            0,
            34,
        >,
    >,
    line_buffer: RefCell<[slint::platform::software_renderer::Rgb565Pixel; 320]>,
}
impl<'a> slint::platform::Platform for MyPlatform<'a> {
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

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        loop {
            slint::platform::update_timers_and_animations();

            // Draw the scene if something needs to be drawn.
            self.window.draw_if_needed(|renderer| {
                // see next section about rendering.
                renderer.render_by_line(DisplayWrapper {
                    display: &mut *self.display.borrow_mut(),
                    line_buffer: &mut *self.line_buffer.borrow_mut(),
                });
            });

            if !self.window.has_active_animations() {
                if let Some(_duration) = slint::platform::duration_until_next_timer_update() {
                    // embassy_time::Timer::after_millis(duration.as_millis() as u64).await;
                    continue;
                }
            }
            // embassy_time::Timer::after_millis(10).await;
        }
    }
}
struct PrinterQueueData {
    data: Rc<slint::VecModel<PrinterQueueItem>>,
    print_progress_timer: slint::Timer,
}

impl PrinterQueueData {
    fn push_job(&self, title: slint::SharedString) {
        self.data.push(PrinterQueueItem {
            status: JobStatus::Waiting,
            progress: 0,
            title,
            owner: env!("CARGO_PKG_AUTHORS").into(),
            pages: 1,
            size: "100kB".into(),
            submission_date: "".into(),
        })
    }
}
static SPI_BUS: StaticCell<NoopMutex<RefCell<Spi<SPI1, DMA1_CH1, NoDma>>>> = StaticCell::new();
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Slint start");
    // Initialize the heap allocator, peripheral devices and other things.
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 168 / 2 = 168Mhz.
            divq: Some(PllQDiv::DIV7), // 8mhz / 4 * 168 / 7 = 48Mhz.
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
    }
    let p = embassy_stm32::init(config);

    // Initialize heap
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

    // Initialize a window (we'll need it later).
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );

    // Make sure the window covers our entire screen.
    window.set_size(slint::PhysicalSize::new(320, 172));

    info!("set window");
    const DISPLAY_WIDTH: usize = 320;
    let line_buffer: [slint::platform::software_renderer::Rgb565Pixel; 320] =
        [slint::platform::software_renderer::Rgb565Pixel(0); DISPLAY_WIDTH];

    let mut spi_config = spi::Config::default();
    spi_config.frequency = mhz(84);
    let spi = spi::Spi::new_txonly(p.SPI1, p.PA5, p.PA7, p.DMA1_CH1, NoDma, spi_config);
    let spi_bus = SPI_BUS.init(NoopMutex::new(RefCell::new(spi)));
    let mut blk = Output::new(p.PE8, Level::High, Speed::Low);
    blk.set_high();
    let mut res = Output::new(p.PE10, Level::High, Speed::Low);
    res.set_high();
    let cs = Output::new(p.PB1, Level::High, Speed::VeryHigh);
    let spi_device = SpiDevice::new(spi_bus, cs);
    let dc = Output::new(p.PC5, Level::High, Speed::VeryHigh);
    let mut display: ST7789<
        SpiDevice<
            embassy_sync::blocking_mutex::raw::NoopRawMutex,
            Spi<SPI1, DMA1_CH1, NoDma>,
            Output<embassy_stm32::peripherals::PB1>,
        >,
        Output<embassy_stm32::peripherals::PC5>,
        320,
        172,
        0,
        34,
    > = ST7789::<_, _, 320, 172, 0, 34>::new(spi_device, dc);

    display.init(&mut Delay);
    display.clear(Rgb565::BLACK).unwrap();

    info!("display clear");
    slint::platform::set_platform(Box::new(MyPlatform {
        window: window.clone(),
        display: RefCell::new(display),
        line_buffer: RefCell::new(line_buffer),
    }))
    .unwrap();

    info!("window set");
    // Setup the UI.
    // let _ui = AppWindow::new().unwrap();
    let main_window = MainWindow::new().unwrap();
    main_window.set_ink_levels(
        [
            InkLevel {
                color: slint::Color::from_rgb_u8(0, 255, 255),
                level: 0.40,
            },
            InkLevel {
                color: slint::Color::from_rgb_u8(255, 0, 255),
                level: 0.20,
            },
            InkLevel {
                color: slint::Color::from_rgb_u8(255, 255, 0),
                level: 0.50,
            },
            InkLevel {
                color: slint::Color::from_rgb_u8(0, 0, 0),
                level: 0.80,
            },
        ]
        .into(),
    );

    let default_queue: Vec<PrinterQueueItem> = main_window
        .global::<PrinterQueue>()
        .get_printer_queue()
        .iter()
        .collect();
    let printer_queue = Rc::new(PrinterQueueData {
        data: Rc::new(slint::VecModel::from(default_queue.clone())),
        print_progress_timer: Default::default(),
    });
    main_window
        .global::<PrinterQueue>()
        .set_printer_queue(printer_queue.data.clone().into());

    main_window.on_quit(move || {
        #[cfg(not(target_arch = "wasm32"))]
        slint::quit_event_loop().unwrap();
    });

    let printer_queue_copy = printer_queue.clone();
    main_window
        .global::<PrinterQueue>()
        .on_start_job(move |title| {
            printer_queue_copy.push_job(title);
        });

    let printer_queue_copy = printer_queue.clone();
    main_window
        .global::<PrinterQueue>()
        .on_cancel_job(move |idx| {
            printer_queue_copy.data.remove(idx as usize);
        });

    let printer_queue_weak = Rc::downgrade(&printer_queue);
    printer_queue.print_progress_timer.start(
        slint::TimerMode::Repeated,
        core::time::Duration::from_millis(1),
        move || {
            if let Some(printer_queue) = printer_queue_weak.upgrade() {
                if printer_queue.data.row_count() > 0 {
                    let mut top_item = printer_queue.data.row_data(0).unwrap();
                    top_item.progress += 1;
                    top_item.status = JobStatus::Printing;
                    if top_item.progress > 100 {
                        printer_queue.data.remove(0);
                        if printer_queue.data.row_count() == 0 {
                            return;
                        }
                        top_item = printer_queue.data.row_data(0).unwrap();
                    }
                    printer_queue.data.set_row_data(0, top_item);
                } else {
                    printer_queue.data.set_vec(default_queue.clone());
                }
            }
        },
    );

    info!("main window");
    main_window.run().unwrap();
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
