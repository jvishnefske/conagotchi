#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // TODO: Initialize your HAL and configure the LED GPIO pin here.
    info!("conagotchi started!");

    loop {
        // TODO: Toggle LED pin
        info!("blink!");
        Timer::after_millis(500).await;
    }
}
