#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::mode::Async;
use embassy_nrf::peripherals::RNG;
use embassy_nrf::pwm::{
    Config, Prescaler, SequenceConfig, SequencePwm, SingleSequenceMode, SingleSequencer,
};
use embassy_nrf::saadc::{ChannelConfig, Config as SaadcConfig, Saadc, VddInput};
use embassy_nrf::{bind_interrupts, rng, saadc};
use embassy_time::{Duration, Timer};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use static_cell::StaticCell;
use trouble_host::prelude::*;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    RADIO => mpsl::HighPrioInterruptHandler;
    TIMER0 => mpsl::HighPrioInterruptHandler;
    RTC0 => mpsl::HighPrioInterruptHandler;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

fn build_sdc<'d, const N: usize>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<Async>,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?.support_adv()?.build(p, rng, mpsl, mem)
}

#[derive(Clone, Copy, Format)]
struct SensorData {
    battery_voltage_mv: u16,
    battery_percentage: u8,
    soil_adc_raw: u16,
    soil_percentage: u8,
    temperature_centicelsius: i16,
    humidity_percentage: u8,
    illuminance_lux: u32,
    packet_counter: u8,
}

impl SensorData {
    fn new(battery_voltage_mv: u16, soil_adc_raw: u16, packet_counter: u8) -> Self {
        // Calculate battery percentage (mock calculation - adjust based on your battery characteristics)
        let battery_percentage = if battery_voltage_mv >= 3000 {
            100
        } else if battery_voltage_mv >= 2700 {
            ((battery_voltage_mv - 2700) * 100) / 300
        } else {
            0
        };

        // Mock soil percentage (since we only have raw ADC)
        let soil_percentage = if soil_adc_raw >= 500 {
            100
        } else if soil_adc_raw >= 100 {
            ((soil_adc_raw - 100) * 100) / 400
        } else {
            0
        };

        // Mock temperature (25.0°C = 2500 centicelsius)
        let temperature_centicelsius = 2500;

        // Mock humidity (50%)
        let humidity_percentage = 50;

        // Mock illuminance (1000 lux)
        let illuminance_lux = 1000;

        Self {
            battery_voltage_mv,
            battery_percentage: battery_percentage as u8,
            soil_adc_raw,
            soil_percentage: soil_percentage as u8,
            temperature_centicelsius,
            humidity_percentage,
            illuminance_lux,
            packet_counter,
        }
    }

    fn to_ble_payload(&self) -> [u8; 17] {
        // BTHome v2 format - matching C implementation exactly (without packet counter)
        // https://bthome.io/format/
        let mut payload = [0u8; 17];

        // BTHome header: version 2, no encryption
        payload[0] = 0x40; // BTHome v2, no encryption

        // Battery percentage (Object ID: 0x01, uint8, factor 1, unit %)
        payload[1] = 0x01;
        payload[2] = self.battery_percentage;

        // Temperature (Object ID: 0x02, int16, factor 0.01, unit °C)
        payload[3] = 0x02;
        payload[4] = (self.temperature_centicelsius & 0xFF) as u8;
        payload[5] = ((self.temperature_centicelsius >> 8) & 0xFF) as u8;

        // Illuminance (Object ID: 0x05, uint24, factor 0.01, unit lux)
        payload[6] = 0x05;
        payload[7] = (self.illuminance_lux & 0xFF) as u8;
        payload[8] = ((self.illuminance_lux >> 8) & 0xFF) as u8;
        payload[9] = ((self.illuminance_lux >> 16) & 0xFF) as u8;

        // Battery voltage (Object ID: 0x0C, uint16, factor 0.001, unit V)
        payload[10] = 0x0C;
        payload[11] = (self.battery_voltage_mv & 0xFF) as u8;
        payload[12] = ((self.battery_voltage_mv >> 8) & 0xFF) as u8;

        // Humidity (Object ID: 0x2E, uint8, factor 1, unit %)
        payload[13] = 0x2E;
        payload[14] = self.humidity_percentage;

        // Soil moisture (Object ID: 0x2F, uint8, factor 1, unit %)
        payload[15] = 0x2F;
        payload[16] = self.soil_percentage;

        payload
    }
}

async fn read_sensors(
    pwm: &mut SequencePwm<'_>,
    saadc: &mut Saadc<'_, 2>,
    led: &mut Output<'_>,
    packet_counter: u8,
) -> Result<SensorData, &'static str> {
    // PWM sequences
    let compare_value = 16; // 50% duty cycle (32/2)
    let pwm_sequence: [u16; 1] = [compare_value];
    let zero_sequence: [u16; 1] = [0];
    let mut seq_config = SequenceConfig::default();
    seq_config.refresh = 0;

    // Turn on LED to indicate sensor reading
    led.set_high();

    // Start PWM with 50% duty cycle
    {
        let sequencer = SingleSequencer::new(pwm, &pwm_sequence, seq_config.clone());
        sequencer
            .start(SingleSequenceMode::Infinite)
            .map_err(|_| "Failed to start PWM")?;

        Timer::after_millis(30).await;

        // Read both channels
        let mut adc_buf = [0; 2]; // [soil, battery]
        saadc.sample(&mut adc_buf).await;

        let soil_raw = adc_buf[0];
        let battery_raw = adc_buf[1];

        // Convert battery ADC to voltage
        // For nRF52840, internal reference is 0.6V
        // With gain 1/6: voltage_mv = (adc_value * 0.6V * 6) / (2^resolution - 1)
        // For 10-bit resolution: voltage_mv = (adc_value * 3600) / 1023
        let battery_voltage_mv = (battery_raw as u32 * 3600) / 1023;

        // Explicitly drop the first sequencer before creating the second one
        drop(sequencer);

        // Stop PWM by setting duty cycle to 0
        let zero_sequencer = SingleSequencer::new(pwm, &zero_sequence, seq_config.clone());
        zero_sequencer
            .start(SingleSequenceMode::Infinite)
            .map_err(|_| "Failed to stop PWM")?;

        led.set_low();

        Ok(SensorData::new(battery_voltage_mv as u16, soil_raw as u16, packet_counter))
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    // Initialize MPSL (Multiprotocol Service Layer)
    let mpsl_p = mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg)));
    spawner.must_spawn(mpsl_task(&*mpsl));

    // Initialize SDC (SoftDevice Controller)
    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let mut rng = rng::Rng::new(p.RNG, Irqs);
    let mut sdc_mem = sdc::Mem::<1120>::new();
    let sdc = unwrap!(build_sdc(sdc_p, &mut rng, &mpsl, &mut sdc_mem));

    // Initialize BLE Host
    // let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    let address: Address = Address::random([0xff, 0x00, 0x00, 0x00, 0x00, 0x01]);
    let addr_bytes = address.to_bytes();
    info!("Our address = {=[u8]:x}", addr_bytes);

    let mut resources: HostResources<DefaultPacketPool, 0, 0> = HostResources::new();
    let stack = trouble_host::new(sdc, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    // Initialize GPIO and peripherals
    let _fast_disch = Output::new(p.P1_10, Level::High, OutputDrive::Standard);
    let mut led = Output::new(p.P0_28, Level::Low, OutputDrive::Standard);

    // Initialize PWM for soil moisture excitation
    let mut pwm_config = Config::default();
    pwm_config.prescaler = Prescaler::Div1;
    pwm_config.max_duty = 32;
    let mut pwm = unwrap!(SequencePwm::new_1ch(p.PWM0, p.P0_05, pwm_config));

    // Initialize SAADC for sensor readings
    let mut saadc_config = SaadcConfig::default();
    saadc_config.resolution = saadc::Resolution::_10BIT;

    let mut soil_channel_config = ChannelConfig::single_ended(p.P0_03);
    soil_channel_config.reference = saadc::Reference::VDD1_4;
    soil_channel_config.gain = saadc::Gain::GAIN1_6;

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

    info!("B-Parasite BLE advertising started");

    // Main loop - similar to original C code
    let _ = join(runner.run(), async {
        let mut packet_counter: u8 = 0;
        loop {
            // Read sensors
            let sensor_data = match read_sensors(&mut pwm, &mut saadc, &mut led, packet_counter).await {
                Ok(data) => {
                    info!(
                        "Battery: {}mV, Soil ADC: {}",
                        data.battery_voltage_mv, data.soil_adc_raw
                    );
                    data
                }
                Err(e) => {
                    error!("Failed to read sensors: {}", e);
                    Timer::after_millis(5000).await;
                    continue;
                }
            };

            // Prepare BLE advertisement data
            let payload = sensor_data.to_ble_payload();
            info!("Broadcast payload: {=[u8]:x}", payload);

            // Create advertisement data
            let mut adv_data = [0; 31];
            let len = AdStructure::encode_slice(
                &[
                    AdStructure::CompleteLocalName(b"prst"),
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceData16 { uuid: [0xD2, 0xFC], data: &payload }, // BTHome service UUID (little endian)
                ],
                &mut adv_data[..],
            )
            .unwrap();

            // Start advertising
            let mut params = AdvertisementParameters::default();
            params.interval_min = Duration::from_millis(100);
            params.interval_max = Duration::from_millis(200);

            match peripheral
                .advertise(
                    &params,
                    Advertisement::NonconnectableScannableUndirected {
                        adv_data: &adv_data[..len],
                        scan_data: &[],
                    },
                )
                .await
            {
                Ok(_adv) => {
                    // info!("Started BLE advertising");

                    // Advertise for configured duration
                    Timer::after_millis(3000).await; // 3 seconds advertising

                    // Stop advertising (by dropping _adv)
                    // info!("Stopped BLE advertising");
                }
                Err(_e) => {
                    error!("Failed to start advertising");
                }
            }

            // Increment packet counter for next cycle
            packet_counter = packet_counter.wrapping_add(1);

            // Sleep before next cycle
            Timer::after_millis(7000).await; // Total 10 second cycle (3s adv + 7s sleep)
        }
    })
    .await;
}
