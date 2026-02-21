#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::I2c;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    // generator version: 1.1.0

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let i2c = I2c::new(peripherals.I2C0, Default::default())
        .unwrap()
        .with_scl(peripherals.GPIO0)
        .with_sda(peripherals.GPIO1)
        .into_async();
    let mut mpu = esp_mpu6050::Mpu6050::new(i2c, 0x68);
    mpu.init().await;
    // Values got from calibration.rs
    mpu.set_offsets(878, -136, 285, -185, 190, -108);

    const DEG_2_RAD: f32 = 3.14159 / 180.0;
    const RAD_2_DEG: f32 = 180.0 / 3.14159;
    let mut angle = 0.0;
    loop {
        Timer::after_millis(100).await;
        let meas = mpu.get_measurements().await;
        let gyro_angle = angle + (meas.gyro_x * DEG_2_RAD) * 0.1;
        let accel_angle = libm::atan2f(meas.accel_y, meas.accel_z);
        const ALPHA: f32 = 0.02;
        angle = ALPHA * gyro_angle + (1.0 - ALPHA) * accel_angle;
        info!("{}", angle * RAD_2_DEG);
        info!("");
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
}
