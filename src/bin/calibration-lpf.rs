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

    info!("Start calibrating...");

    let raw_meas = mpu.get_raw_measurements().await;
    let mut offsets = (
        raw_meas.accel_x as f32,
        raw_meas.accel_y as f32,
        raw_meas.accel_z as f32 - 16384.0,
        raw_meas.gyro_x as f32,
        raw_meas.gyro_y as f32,
        raw_meas.gyro_z as f32,
    );

    for _ in 0..1000 {
        let raw_meas = mpu.get_raw_measurements().await;
        low_pass_filter(&mut offsets.0, raw_meas.accel_x as f32);
        low_pass_filter(&mut offsets.1, raw_meas.accel_y as f32);
        low_pass_filter(&mut offsets.2, raw_meas.accel_z as f32 - 16384.0);
        low_pass_filter(&mut offsets.3, raw_meas.gyro_x as f32);
        low_pass_filter(&mut offsets.4, raw_meas.gyro_y as f32);
        low_pass_filter(&mut offsets.5, raw_meas.gyro_z as f32);
        Timer::after_millis(10).await;
    }

    info!("Finished calibrating!");
    info!("Calibrated values: {:?}", offsets);
    mpu.set_offsets(
        offsets.0 as i16,
        offsets.1 as i16,
        offsets.2 as i16,
        offsets.3 as i16,
        offsets.4 as i16,
        offsets.5 as i16,
    );

    loop {
        Timer::after_secs(1).await;
        let meas = mpu.get_measurements().await;
        info!("Accel X: {}", meas.accel_x);
        info!("Accel Y: {}", meas.accel_y);
        info!("Accel Z: {}", meas.accel_z);
        info!("Gyro X: {}", meas.gyro_x);
        info!("Gyro Y: {}", meas.gyro_y);
        info!("Gyro Z: {}", meas.gyro_z);
        info!("");
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
}

fn low_pass_filter(current: &mut f32, meas: f32) {
    const ALPHA: f32 = 0.01;
    *current = (1.0 - ALPHA) * (*current) + ALPHA * meas;
}
