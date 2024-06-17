#![no_std]
#![no_main]

slint::include_modules!();

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_alloc::Heap;
use slint::platform::software_renderer::MinimalSoftwareWindow;
use embassy_stm32 as _;
extern crate alloc;
use alloc::{boxed::Box, rc::Rc};
use defmt_rtt as _;
use panic_probe as _;

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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize the heap allocator, peripheral devices and other things.
    let p = embassy_stm32::init(Default::default());
    
    // Initialize heap
    unsafe { ALLOCATOR.init(core::ptr::addr_of_mut!(HEAP) as usize, HEAP_SIZE) }

    // Initialize a window (we'll need it later).
    let window = MinimalSoftwareWindow::new(Default::default());
    slint::platform::set_platform(Box::new(MyPlatform {
        window: window.clone(),
    }))
    .unwrap();

    // Setup the UI.
    let ui = AppWindow::new().unwrap();
    // // ... setup callback and properties on `ui` ...

    // // Make sure the window covers our entire screen.
    window.set_size(slint::PhysicalSize::new(320, 240));
    // let mut buffer_provider = DrawBuffer {
    //     display,
    //     buffer: &mut [Rgb565Pixel(0); 240],
    // };
    // ... start event loop (see later) ...
    loop {
        slint::platform::update_timers_and_animations();
        // window.draw_if_needed(|renderer| {
        //     renderer.render_by_line(&mut buffer_provider);
        // });

        if !window.has_active_animations() {
            if let Some(duration) = slint::platform::duration_until_next_timer_update() {
                Timer::after(Duration::from_millis(duration.as_millis() as u64)).await;
                continue;
            }
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}
