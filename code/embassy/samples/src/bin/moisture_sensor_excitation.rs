#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::pwm::{
    Config, Prescaler, SequenceConfig, SequencePwm, SingleSequenceMode, SingleSequencer,
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    info!("=== Soil Moisture PWM Test ===");
    info!("Replicating nrf-connect PWM functionality for oscilloscope testing");
    info!("Target signal: 500kHz square wave with 50% duty cycle on P0.05");

    // =========================================================================
    // PWM Configuration Matching nrf-connect
    // =========================================================================
    // nrf-connect device tree configuration:
    //   soil_pwm: soil_pwm {
    //     compatible = "pwm-fixed";
    //     pwms = <&pwm0 0 PWM_USEC(2) PWM_POLARITY_NORMAL>;
    //     pulse = <PWM_USEC(1)>;
    //   };
    //
    // This means:
    // - Period: 2 microseconds (PWM_USEC(2)) = 500kHz
    // - Pulse width: 1 microsecond (PWM_USEC(1)) = 50% duty cycle
    // - Pin: P0.05 (configured in pinctrl)
    // - Polarity: Normal (not inverted)
    //
    // Embassy-nrf PWM frequency formula:
    //   frequency = 16,000,000 Hz / (prescaler * max_duty)
    // Where 16,000,000 Hz is the PWM clock frequency
    //
    // To get 500kHz: 16,000,000 / (prescaler * max_duty) = 500,000
    // So: prescaler * max_duty = 32
    //
    // Using prescaler = Div1 and max_duty = 32 gives us:
    //   frequency = 16,000,000 / (1 * 32) = 500,000 Hz ✓
    // =========================================================================

    let mut config = Config::default();
    config.prescaler = Prescaler::Div1; // Prescaler value 1
    config.max_duty = 32; // Results in 500kHz frequency

    info!("PWM Configuration:");
    info!("  Prescaler: Div1 (value = 1)");
    info!("  Max duty: {}", config.max_duty);
    info!(
        "  Calculated frequency: {} Hz ({} kHz)",
        16_000_000 / (1 * config.max_duty as u32),
        16_000_000 / (1 * config.max_duty as u32) / 1000
    );

    // Calculate signal parameters
    let compare_value = config.max_duty / 2; // 16 for 50% duty cycle
    // PWM period (PWM cycle time) = 1 / PWM frequency = 1 / (clock frequency / (prescaler * max duty)) = (prescaler * max duty) / clock frequency = 32 / 16mHZ = 2/1000000 s = 2µs
    // PW = PWM period * compare value / max_duty = 2µs * 50% = 1µs

    // Initialize LED on P0.28 (same as blinky example)
    let mut led = Output::new(p.P0_28, Level::Low, OutputDrive::Standard);
    info!("LED initialized on P0.28 - will blink with each PWM pulse");

    // Create PWM on P0.05 (single channel)
    // Note: P0.05 is the soil moisture PWM pin in B-Parasite hardware
    // Using SequencePwm as shown in the existing pwm.rs example
    let mut pwm = unwrap!(SequencePwm::new_1ch(p.PWM0, p.P0_05, config));

    // Create a simple sequence with our desired duty cycle
    // Sequence with a single value that gives us 50% duty cycle
    let pwm_sequence: [u16; 1] = [compare_value];
    let mut seq_config = SequenceConfig::default();
    // We want the sequence to refresh at the PWM frequency
    seq_config.refresh = 0; // No additional delays between sequence steps


    let mut cycle_count = 0;

    loop {
        cycle_count += 1;

        // Start PWM with our sequence (50% duty cycle)
        info!("Cycle {}: PWM ON - Soil circuit active", cycle_count);
        
        // Blink LED at the start of PWM pulse
        led.set_high();
        Timer::after_millis(10).await; // Short blink duration
        led.set_low();
        
        {
            let sequencer = SingleSequencer::new(&mut pwm, &pwm_sequence, seq_config.clone());

            // Start the sequencer in infinite mode (or times mode)
            match sequencer.start(SingleSequenceMode::Infinite) {
                Ok(()) => {
                    // PWM is now running with 50% duty cycle
                    // Wait 30ms (as in nrf-connect's prst_adc_soil_read)
                    Timer::after_millis(30).await;

                    // In actual nrf-connect code, ADC reading happens here
                    // info!("  [ADC reading would occur here in full implementation]");

                    // Stop PWM by stopping the sequencer
                    // Note: We need to handle this differently since Infinite mode doesn't stop
                    // For now, we'll create a new sequencer with duty cycle 0
                }
                Err(e) => {
                    error!("Failed to start PWM sequencer: {:?}", e);
                    Timer::after_millis(1000).await;
                    continue;
                }
            }
        } // sequencer dropped here

        // To stop PWM, we create a sequence with duty cycle 0
        info!("  PWM OFF - Power saving mode");
        let zero_sequence: [u16; 1] = [0];
        let zero_sequencer = SingleSequencer::new(&mut pwm, &zero_sequence, seq_config.clone());

        match zero_sequencer.start(SingleSequenceMode::Infinite) {
            Ok(()) => {
                // Wait 970ms before next reading
                Timer::after_millis(970).await;
            }
            Err(e) => {
                error!("Failed to start zero-duty PWM: {:?}", e);
                Timer::after_millis(970).await;
            }
        }
    }
}

// Additional notes for the user:
// 1. This code replicates the PWM functionality from nrf-connect's soil moisture sensing
// 2. The PWM generates a 500kHz square wave with 50% duty cycle on P0.05
// 3. This matches the nrf-connect configuration:
//    - Device tree: pwms = <&pwm0 0 PWM_USEC(2) PWM_POLARITY_NORMAL>;
//    - pulse = <PWM_USEC(1)>;
// 4. The ON/OFF pattern (30ms ON, 970ms OFF) simulates the soil reading cycle
// 5. Use an oscilloscope on P0.05 to verify:
//    - Frequency: 500kHz (period: 2µs)
//    - Duty cycle: 50% (pulse width: 1µs)
//    - Timing: 30ms bursts every 1 second
