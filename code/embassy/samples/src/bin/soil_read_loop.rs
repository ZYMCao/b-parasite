#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::pwm::{
    Config, Prescaler, SequenceConfig, SequencePwm, SingleSequenceMode, SingleSequencer,
};
use embassy_nrf::saadc::{ChannelConfig, Config as SaadcConfig, Saadc, VddInput};
use embassy_nrf::{bind_interrupts, saadc};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    // =========================================================================
    // GPIO Configuration - Fast Discharge Circuit
    // =========================================================================
    // From nrf-connect: fast_disch on P1.10 (GPIO_ACTIVE_HIGH)
    // Enable fast discharge circuit continuously (as in original C code)
    let fast_disch = Output::new(p.P1_10, Level::High, OutputDrive::Standard);

    // Initialize LED on P0.28 for status indication
    let mut led = Output::new(p.P0_28, Level::Low, OutputDrive::Standard);

    // =========================================================================
    // PWM Configuration - Soil Moisture Excitation
    // =========================================================================
    // nrf-connect configuration:
    //   pwms = <&pwm0 0 PWM_USEC(2) PWM_POLARITY_NORMAL>;
    //   pulse = <PWM_USEC(1)>;
    // This means: 500kHz frequency, 50% duty cycle on P0.05
    let mut pwm_config = Config::default();
    pwm_config.prescaler = Prescaler::Div1; // Prescaler value 1
    pwm_config.max_duty = 32; // Results in 500kHz frequency (16MHz / 32)

    let compare_value = pwm_config.max_duty / 2; // 16 for 50% duty cycle

    // Create PWM on P0.05 (soil moisture excitation pin)
    let mut pwm = unwrap!(SequencePwm::new_1ch(p.PWM0, p.P0_05, pwm_config));

    // PWM sequences
    let pwm_sequence: [u16; 1] = [compare_value]; // 50% duty cycle
    let zero_sequence: [u16; 1] = [0]; // 0% duty cycle (PWM off)
    let mut seq_config = SequenceConfig::default();
    seq_config.refresh = 0; // No additional delays

    // =========================================================================
    // SAADC Configuration - Soil Moisture and Battery Voltage
    // =========================================================================
    // From nrf-connect device tree:
    // Soil: channel@0, AIN1 (P0.03), gain 1/6, reference VDD/4
    // Battery: channel@2, VDD internal, gain 1/6, reference internal

    let mut saadc_config = SaadcConfig::default();
    saadc_config.resolution = saadc::Resolution::_10BIT; // code/nrf-connect/prstlib/boards/bparasite/bparasite_nrf52840.dts has specified zephyr,resolution = <10>

    // Soil moisture channel: P0.03 (AIN1)
    // From C version: zephyr,gain = "ADC_GAIN_1_6", zephyr,reference = "ADC_REF_VDD_1_4"
    let mut soil_channel_config = ChannelConfig::single_ended(p.P0_03);
    soil_channel_config.reference = saadc::Reference::VDD1_4;
    soil_channel_config.gain = saadc::Gain::GAIN1_6;

    // Battery voltage channel: VDD internal
    // From C version: zephyr,gain = "ADC_GAIN_1_6", zephyr,reference = "ADC_REF_INTERNAL"
    let mut battery_channel_config = ChannelConfig::single_ended(VddInput);
    battery_channel_config.gain = saadc::Gain::GAIN1_6;

    let mut saadc = Saadc::new(
        p.SAADC,
        Irqs,
        saadc_config,
        [soil_channel_config, battery_channel_config],
    );

    // Calibrate SAADC
    saadc.calibrate().await;
    info!("input_voltage;soil_adc_output");

    loop {
        // =========================================================================
        // PWM ON Phase - Excite soil moisture sensor
        // =========================================================================
        // Blink LED to indicate PWM active
        led.set_high();

        // Start PWM with 50% duty cycle
        {
            let sequencer = SingleSequencer::new(&mut pwm, &pwm_sequence, seq_config.clone());
            match sequencer.start(SingleSequenceMode::Infinite) {
                Ok(()) => {
                    // PWM is now running, wait for stabilization (match C version timing)
                    Timer::after_millis(500).await;

                    // =========================================================================
                    // ADC Reading Phase
                    // =========================================================================
                    // Read both channels
                    let mut adc_buf = [0; 2]; // [soil, battery]
                    saadc.sample(&mut adc_buf).await;

                    let soil_raw = adc_buf[0];
                    let battery_raw = adc_buf[1];

                    // Convert battery ADC to voltage
                    // From nrf-connect: adc_raw_to_millivolts_dt with gain 1/6, internal reference
                    // For nRF52840, internal reference is 0.6V
                    // With gain 1/6: voltage_mv = (adc_value * 0.6V * 6) / (2^resolution - 1)
                    // For 10-bit resolution: voltage_mv = (adc_value * 3600) / 1023
                    let battery_voltage_mv = (battery_raw as i32 * 3600) / 1023;
                    let battery_voltage_v = battery_voltage_mv as f32 / 1000.0;

                    // Log data in format matching original C code
                    // C version logs: battery_voltage(V);soil_adc_raw (not converted)
                    info!("{=f32};{=i16}", battery_voltage_v, soil_raw);

                    // Continue PWM for total of 30ms
                    Timer::after_millis(20).await;
                }
                Err(e) => {
                    error!("Failed to start PWM sequencer: {:?}", e);
                    Timer::after_millis(30).await;
                }
            }
        }

        led.set_low();

        // =========================================================================
        // PWM OFF Phase - Power saving
        // =========================================================================
        // Stop PWM by setting duty cycle to 0
        {
            let zero_sequencer = SingleSequencer::new(&mut pwm, &zero_sequence, seq_config.clone());
            match zero_sequencer.start(SingleSequenceMode::Infinite) {
                Ok(()) => {
                    // Wait 970ms before next reading (total 1 second cycle)
                    Timer::after_millis(970).await;
                }
                Err(e) => {
                    error!("Failed to stop PWM: {:?}", e);
                    Timer::after_millis(970).await;
                }
            }
        }
    }
}
