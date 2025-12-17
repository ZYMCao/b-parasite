#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    
    // LED on P0.28 (active high)
    let mut led = Output::new(p.P0_28, Level::Low, OutputDrive::Standard);
    
    // Button on P0.30 (active low with pull-up)
    let mut button = Input::new(p.P0_30, Pull::Up);
    
    // Flash LED twice (400ms period as in C code)
    for _ in 0..2 {
        led.toggle();
        Timer::after_millis(200).await;
        led.toggle();
        Timer::after_millis(200).await;
    }
    
    // Read initial button state
    let initial_state = button.is_high(); // Button is active low, so high = inactive
    defmt::info!("Initial button state: {}", if initial_state { "inactive" } else { "active" });
    
    // TODO: Initialize sensors (ADC, I2C) to put them in low power mode
    // For now, just log that we would initialize sensors
    defmt::info!("Sensor initialization placeholder");
    
    // Flash LED twice again (as in C code after sensor init)
    for _ in 0..2 {
        led.toggle();
        Timer::after_millis(200).await;
        led.toggle();
        Timer::after_millis(200).await;
    }
    
    defmt::info!("Main loop.");
    
    // Button handling with debouncing
    let mut debounce_timer = Timer::after(Duration::from_millis(10));
    let mut last_state = button.is_high();
    
    loop {
        // Wait for either button change or debounce timer
        match select(button.wait_for_any_edge(), &mut debounce_timer).await {
            Either::First(_) => {
                // Button edge detected
                let current_state = button.is_high();
                
                if current_state != last_state {
                    last_state = current_state;
                    let is_active = !current_state; // Invert since active low
                    
                    if is_active {
                        defmt::info!("Button pressed (debounced)");
                        led.set_high();
                    } else {
                        defmt::info!("Button released (debounced)");
                        led.set_low();
                    }
                    
                    // Reset debounce timer
                    debounce_timer = Timer::after(Duration::from_millis(10));
                }
            }
            Either::Second(_) => {
                // Debounce timer expired, just reset it
                debounce_timer = Timer::after(Duration::from_millis(10));
            }
        }
    }
}
