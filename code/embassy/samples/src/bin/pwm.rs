#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::pwm::{Config, Error, Prescaler, SequenceConfig, SequencePwm, SingleSequenceMode, SingleSequencer};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let seq_words: [u16; 19] = [2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2];

    // PWM frequency = clock frequency / (prescaler * max duty)
    // PWM resolution = 1 / max duty
    // Duty cycle (占空比) = 比较值 / max duty
    let mut config = Config::default();
    config.prescaler = Prescaler::Div128;
    config.max_duty = 1024;
    // embassy_nrf::pwm::PWM_CLK_HZ: u32 = 16_000_000;
    // PWM period (PWM cycle time) is 128 / 16mhz = 128 / (16,000,000 / s) = 128s / 16,000,000 = 128ms / 16,000 = 128,000µs / 16,000 = 128µs / 16 = 8µs
    // Refresh period (sequence step duration) = PWM period * max duty = 1024 * 8 = 8192 µs
    // We want each brightness level to last for roughly half a second (524,288µs), so we need 524,288µs / 8192 µs = 64 periods
    let mut seq_config = SequenceConfig::default();
    seq_config.refresh = 63;

    let mut pwm = unwrap!(SequencePwm::new_1ch(p.PWM0, p.P0_28, config));
    info!("Starting PWM breathing effect on P0.28");

    let sequencer = SingleSequencer::new(&mut pwm, &seq_words, seq_config.clone());
    // unwrap!(sequencer.start(SingleSequenceMode::Infinite)); // ToDo: figure out why SingleSequenceMode::Infinite does not work
    loop {
        match sequencer.start(SingleSequenceMode::Times(1)) {
            Ok(()) => {
                Timer::after_millis(10500).await;
            }
            Err(e) => {
                // 处理启动失败的错误
                error!("Failed to start PWM sequencer: {:?}", e);

                // 根据错误类型采取不同的恢复策略
                match e {
                    Error::SequenceTooLong => {
                        error!("Sequence too long error");
                    }
                    Error::SequenceTimesAtLeastOne => {
                        error!("Sequence times must be at least one");
                    }
                    Error::BufferNotInRAM => {
                        error!("Buffer not in RAM");
                    }
                    _ => {}
                }

                Timer::after_millis(1000).await;
            }
        }
    }
}
