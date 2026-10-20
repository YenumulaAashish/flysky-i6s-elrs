//! On-chip Flash storage for non-volatile configuration and 20-model memory system.
//!
//! Log-structured append-only storage (`sequential-storage`):
//! - Pages 60..63: 0x0801_E000 .. 0x0802_0000 (8,192 bytes across 4 pages)
//!
//! Total capacity: 8,192 bytes.
//! RadioConfig: 148 bytes.
//! ModelConfig: 348 bytes. Active model cached in SRAM; 20 models stored on-demand.

use embedded_storage_async::nor_flash::NorFlash;
#[cfg(not(test))]
use stm32f0xx_hal::pac;

pub const FLASH_MAGIC: u32 = 0x4653_4B59; // "FSKY"
pub const CONFIG_VERSION: u32 = 5;
pub const NUM_MODELS: usize = 20;

pub const FLASH_STORAGE_ADDR: usize = 0x0801_E000;
pub const FLASH_STORAGE_END: usize = 0x0802_0000;

pub const KEY_RADIO: u8 = 0;
pub const KEY_MODEL_BASE: u8 = 1;

const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

/// Calibration parameters for a single analog channel (8 bytes).
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ChannelCalib {
    pub min: u16,
    pub center: u16,
    pub max: u16,
    pub _pad: u16,
}

impl ChannelCalib {
    pub const fn new(min: u16, center: u16, max: u16) -> Self {
        Self {
            min,
            center,
            max,
            _pad: 0,
        }
    }
}

/// Compact calibration parameters for an auxiliary potentiometer/slider (6 bytes).
/// Supports both standard pots (with calculated midpoint) and detented pots (with fixed detent center).
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PotCalib {
    pub min: u16,
    pub center: u16,
    pub max: u16,
}

impl PotCalib {
    pub const fn new(min: u16, center: u16, max: u16) -> Self {
        Self { min, center, max }
    }
}

/// Universal input mode for auxiliary analog channels (SA..SD, VRA..VRB, VRC..VRF).
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum AdcInputMode {
    #[default]
    Default = 0,
    Off = 1,
    TwoPos = 2,
    ThreePos = 3,
    SixPos = 4,
    Pot = 5,
    PotDetent = 6,
    InstantTrim = 7,
}

impl AdcInputMode {
    pub fn from_u8(val: u8) -> Self {
        match val {
            1 => Self::Off,
            2 => Self::TwoPos,
            3 => Self::ThreePos,
            4 => Self::SixPos,
            5 => Self::Pot,
            6 => Self::PotDetent,
            7 => Self::InstantTrim,
            _ => Self::Default,
        }
    }

    /// Check if this mode represents any analog potentiometer (continuous or center-detent).
    pub fn is_pot(&self) -> bool {
        matches!(self, Self::Pot | Self::PotDetent)
    }

    /// Check if this mode utilizes a physical center detent.
    pub fn has_detent(&self) -> bool {
        matches!(self, Self::PotDetent)
    }

    /// Check if this input is disabled / turned off.
    pub fn is_off(&self) -> bool {
        matches!(self, Self::Off)
    }

    /// Resolve effective mode for a given channel index (0..3: SA..SD, 4..5: VRA..VRB, 6..10: VRC..VRG)
    pub fn resolve(channel_idx: usize, mode_val: u8) -> Self {
        let mode = Self::from_u8(mode_val);
        if mode != Self::Default {
            return mode;
        }
        // Default hardware mapping:
        match channel_idx {
            0 => Self::TwoPos,               // SA (2-pos stock)
            1 => crate::hw::DEFAULT_SB_MODE, // SB (2-pos stock on i6X, 3-pos stock on i6S)
            2 => Self::ThreePos,             // SC (3-pos stock)
            3 => Self::TwoPos,               // SD (2-pos stock)
            _ => Self::Pot,                  // VRA..VRG (Pots stock)
        }
    }
}

/// Persistent system/radio-level configuration (140 bytes, 4-byte aligned for STM32F0 Flash programming).
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct RadioConfig {
    pub magic: u32,                // 0..4 ("FSKY")
    pub version: u32,              // 4..8 (5)
    pub active_model: u8,          // 8 (0..19, active model index)
    pub throttle_trim: u8,         // 9 (0: Off/Lock, 1: Idle T-Trim, 2: Linear)
    pub audio_enabled: u8,         // 10 (0: Muted, 1: Enabled)
    pub backlight_timeout: u8,     // 11 (0: Always On, 1: 15s, 2: 30s, 3: 60s)
    pub backlight_brightness: u8,  // 12 (1..10)
    pub vbat_warn_deci: u8,        // 13 (40..50 = 4.0V..5.0V, default 44 = 4.4V)
    pub lcd_contrast: u8,          // 14 (15..55, default 37 / 0x25)
    pub usb_mode: u8,              // 15 (0: Off, 1: Joystick, 2: Serial, 3: Composite)
    pub sticks: [ChannelCalib; 4], // 16..48 (32 bytes: Roll, Pitch, Throttle, Yaw)
    pub aux_pots: [PotCalib; 11],  // 48..114 (66 bytes: SA..SD, VRA..VRB, VRC..VRG)
    pub ext_module_pwr: u8,        // 114 (0: Active HIGH / N-type, 1: Active LOW / P-type)
    pub tone_style: u8,            // 115 (0: Simple / Standard, 1: Rich / Melodic)
    pub servo_rate_hz: u16,        // 116..118 (50..400 Hz, default 50 Hz for analog servo safety)
    pub rx_out_mode: u8,           // 118 (0: PWM, 1: PPM, default 0)
    pub rx_serial_proto: u8,       // 119 (0: i-BUS, 1: S.BUS, default 0)
    pub ext_switches: u8,          // 120 (0: Disabled, 1: Enabled / PC12+PC15)
    pub ext_adc: u8,               // 121 (0: Disabled, 1: Enabled / Header P7 AD11-AD15 or PC1-PC5)
    pub adc_modes: [u8; 11],       // 122..133: Input modes for the 11 auxiliary analog channels
    pub stick_mode: u8,            // 133 (0: Mode 1, 1: Mode 2, 2: Mode 3, 3: Mode 4)
    pub telem_slots: [u8; 4],      // 134..138: Configurable telemetry dashboard sensor slots (1..4)
    pub trainer_config: u8,        // 138: Bit 0: PPM Out on PF10, Bit 1: PPM In on PF9
    pub rear_left_func: u8,        // 139: 0: RC Switch (SWE), 1: Timer Reset, 2: Trims Modifier
    pub rear_right_func: u8,       // 140: 0: RC Switch (SWF), 1: Instant Trim, 2: Trims Modifier
    pub touch_disabled: u8,        // 141: 0: Touch Enabled, 1: Touch Disabled
    pub j4_sg_sh_en: u8,           // 142: 0: SWD Debug, 1: SG/SH Buttons on PA13/PA14
    pub switch_beep: u8,           // 143: 0: Muted, 1: Beep on switch toggle
    pub _pad_align: [u8; 4],       // 144..148: Align to 148 bytes (4-byte word boundary)
}

impl RadioConfig {
    pub const fn default_factory() -> Self {
        Self {
            magic: FLASH_MAGIC,
            version: CONFIG_VERSION,
            active_model: 0,
            throttle_trim: 0,
            audio_enabled: 1,
            backlight_timeout: 0,
            backlight_brightness: 10,
            vbat_warn_deci: 44,
            lcd_contrast: 37,
            usb_mode: 0,
            sticks: [
                ChannelCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // Roll (Horizontal)
                ChannelCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // Pitch (Vertical)
                ChannelCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // Throttle (Vertical)
                ChannelCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // Yaw (Horizontal)
            ],
            aux_pots: [
                PotCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // SA
                PotCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // SB
                PotCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // SC
                PotCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // SD
                PotCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // VRA
                PotCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // VRB
                PotCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // VRC
                PotCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // VRD
                PotCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // VRE
                PotCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // VRF
                PotCalib::new(
                    crate::adc::ADC_MIN,
                    crate::adc::ADC_CENTER,
                    crate::adc::ADC_MAX,
                ), // VRG
            ],
            ext_module_pwr: 0,
            tone_style: 1,
            servo_rate_hz: 50,
            rx_out_mode: 0,
            rx_serial_proto: 0,
            ext_switches: 0,
            ext_adc: 0,
            adc_modes: [0; 11], // Default hardware modes
            stick_mode: 1,      // Mode 2 default
            telem_slots: [0, 1, 2, 3], // Default: RxBat, RSSI/LQ, TxBat, Current/Link
            trainer_config: 0,
            rear_left_func: 0,
            rear_right_func: 0,
            touch_disabled: 0,
            j4_sg_sh_en: 0,
            switch_beep: 1,
            _pad_align: [0; 4],
        }
    }

    /// Check if PPM Output on PF10 is enabled.
    #[inline(always)]
    pub fn is_ppm_out_enabled(&self) -> bool {
        (self.trainer_config & 0x01) != 0
    }

    /// Check if PPM Input on PF9 is enabled.
    #[inline(always)]
    pub fn is_ppm_in_enabled(&self) -> bool {
        (self.trainer_config & 0x02) != 0
    }

    /// Set PPM Output on PF10 enabled state.
    #[inline(always)]
    pub fn set_ppm_out_enabled(&mut self, enabled: bool) {
        if enabled {
            self.trainer_config |= 0x01;
        } else {
            self.trainer_config &= !0x01;
        }
    }

    /// Set PPM Input on PF9 enabled state.
    #[inline(always)]
    pub fn set_ppm_in_enabled(&mut self, enabled: bool) {
        if enabled {
            self.trainer_config |= 0x02;
        } else {
            self.trainer_config &= !0x02;
        }
    }

    /// Get raw configured ADC mode index (0..=6) for auxiliary input `ch` (`0..10`).
    #[inline(always)]
    pub const fn get_adc_mode_val(&self, ch: usize) -> u8 {
        if ch < 11 {
            self.adc_modes[ch]
        } else {
            0
        }
    }

    /// Set raw configured ADC mode index (0..=7) for auxiliary input `ch` (`0..10`).
    #[inline(always)]
    pub fn set_adc_mode_val(&mut self, ch: usize, val: u8) {
        if ch < 11 {
            self.adc_modes[ch] = val.min(7);
        }
    }

    /// Get the effective AdcInputMode for an auxiliary channel (0..10: SA..SD, VRA..VRB, VRC..VRG).
    pub fn get_adc_mode(&self, ch: usize) -> AdcInputMode {
        if ch < 11 {
            AdcInputMode::resolve(ch, self.adc_modes[ch])
        } else {
            AdcInputMode::Default
        }
    }

    /// Get the configured raw AdcInputMode (including Default).
    pub fn get_raw_adc_mode(&self, ch: usize) -> AdcInputMode {
        if ch < 11 {
            AdcInputMode::from_u8(self.adc_modes[ch])
        } else {
            AdcInputMode::Default
        }
    }
}

pub const NUM_MIX_LINES: usize = 32;

/// A single freeform mix rule in the matrix mixer (8 bytes).
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MixLine {
    pub target_ch: u8, // 0: Disabled, 1..18: Target Channel CH1..CH18
    pub source: u8, // 0: None, 1: Roll, 2: Pitch, 3: Thr, 4: Yaw, 5: VRA, 6: VRB, 7: SA, 8: SB, 9: SC, 10: SD, 11: MAX, 12..29: CH1..CH18, 30: ThrUnipolar, 31..40: SE..VRF
    pub weight_src: u8, // 0: Fixed Weight, 1..18: Modulate with CH1..CH18 (Bipolar), 19..36: Modulate (Unipolar 0..100%)
    pub weight: i8, // -100% .. +100%
    pub offset: i8, // -100% .. +100%
    pub switch: u8, // 0: Always On, 1..22: SA_UP..SWJ_DN
    pub mode: u8,   // 0: Add (+), 1: Multiply (*), 2: Replace (:=)
    pub _reserved: u8, // 1-byte padding for 8-byte alignment
}

impl MixLine {
    pub const fn disabled() -> Self {
        Self {
            target_ch: 0,
            source: 0,
            weight_src: 0,
            weight: 100,
            offset: 0,
            switch: 0,
            mode: 0,
            _reserved: 0,
        }
    }
}

/// Complete profile for an individual aircraft/model (exactly 348 bytes).
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ModelConfig {
    pub name: [u8; 10],         // 0..10: 10-char ASCII name
    pub model_type: u8,         // 10: 0: Airplane, 1: Glider, 2: Heli, 3: Quad, 4: General
    pub _pad0: u8,              // 11: align rx_id to 4 bytes
    pub rx_id: u32,             // 12..16: Model Match bound receiver ID
    pub trims: [i8; 4],         // 16..20: Roll, Pitch, Throttle, Yaw (-25..+25)
    pub channel_reverse: u32,   // 20..24: Bitmask for CH1..CH18 inversion
    pub dr_switch: u8,          // 24: 0: None, 1: SA, 2: SB, 3: SC, 4: SD, 5: SE, 6: SF
    pub thr_curve_pts: u8,      // 25: 5, 7, or 9 points mode
    pub thr_curve_smooth: u8,   // 26: 0: Linear, 1: Spline / Smooth
    pub thr_curve: [u8; 17],    // 27..44: Points buffer (0..100%, reserved up to 17)
    pub _pad_crv: u8,           // 44..45
    pub dr_high: [u8; 3],       // 45..48: High rate (AIL, ELE, RUD: 50..100%)
    pub dr_low: [u8; 3],        // 48..51: Low rate (AIL, ELE, RUD: 30..100%)
    pub expo_high: [i8; 3],     // 51..54: High expo (-100..+100%)
    pub expo_low: [i8; 3],      // 54..57: Low expo (-100..+100%)
    pub _pad1: u8,              // 57..58: align timer_secs to 2-byte boundary
    pub timer_secs: u16,        // 58..60: Countdown timer in seconds (e.g. 300 = 5 min)
    pub timer_source: u8, // 60: 0: Off, 1: THs (Thr>5%), 2: THt (Thr Latched), 3: Always On, 4..17: SA^..SFv
    pub rf_power: u8, // 61: AFHDS 2A RF power (0: 100mW, 1: 10mW, 2: 1mW, 3: 100uW)
    pub failsafe_thr: u16, // 62..64: Failsafe throttle pulse in µs (e.g. 1000)
    pub aux_channels: [u8; 14], // 64..78: Source for CH5..CH18
    pub wing_tail_mix: u8, // 78: 0: Normal, 1: Elevon/Delta, 2: V-Tail, 3: Flaperon
    pub template_diff: i8, // 79: Differential / mix ratio (-100..+100)
    pub mixes: [MixLine; NUM_MIX_LINES], // 80..336: 32 freeform mix rules (32 * 8 = 256 bytes)
    pub failsafe_mode: u8, // 336: 0: Hold last, 1: Custom pulses
    pub failsafe_timeout: u8, // 337: 10..50 (1.0s..5.0s)
    pub rf_protocol: u8,  // 338: 0: AFHDS 2A, 1: CRSF / ELRS
    pub crsf_baud: u8,    // 339: 0: 115.2k, 1: 416.6k, 2: 420k, 3: 921.6k, 4: 1.875M
    pub arm_switch: u8, // 340: 0: None, 1..14: SA^..SFv
    pub rx_out_mode: u8, // 341: 0: PWM, 1: PPM (default 0)
    pub servo_rate_hz: u16, // 342..344: 50..400 Hz (default 50 Hz)
    pub rx_serial_proto: u8, // 344: 0: i-BUS, 1: S.BUS (default 0)
    pub crsf_half_duplex: u8, // 345: 0: Full-Duplex (2-Wire), 1: Half-Duplex (1-Wire)
    pub _pad_end: [u8; 2], // 346..348: Align struct to 4-byte word boundary
}

impl ModelConfig {
    pub const fn default_for_index(idx: usize) -> Self {
        let num = (idx + 1) as u8;
        let digit1 = b'0' + (num / 10);
        let digit2 = b'0' + (num % 10);
        Self {
            name: [
                b'M', b'O', b'D', b'E', b'L', b' ', digit1, digit2, b' ', b' ',
            ],
            model_type: 0,
            _pad0: 0,
            rx_id: 0,
            trims: [0, 0, 0, 0],
            channel_reverse: 0,
            dr_switch: 0,
            thr_curve_pts: 5,
            thr_curve_smooth: 0,
            thr_curve: [0, 25, 50, 75, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            _pad_crv: 0,
            dr_high: [100, 100, 100],
            dr_low: [70, 70, 70],
            expo_high: [0, 0, 0],
            expo_low: [0, 0, 0],
            _pad1: 0,
            timer_secs: 300,
            timer_source: 1,
            rf_power: 0,
            failsafe_thr: 1000,
            aux_channels: [7, 8, 5, 6, 9, 10, 0, 0, 0, 0, 0, 0, 0, 0],
            wing_tail_mix: 0,
            template_diff: 0,
            mixes: [MixLine::disabled(); NUM_MIX_LINES],
            failsafe_mode: 0,
            failsafe_timeout: 20,
            rf_protocol: 0,
            crsf_baud: 2,
            arm_switch: 0,
            servo_rate_hz: 50,
            rx_out_mode: 0,
            rx_serial_proto: 0,
            crsf_half_duplex: 0,
            _pad_end: [0; 2],
        }
    }

    #[inline(always)]
    pub fn model_type(&self) -> crate::safety::ModelType {
        crate::safety::ModelType::from_u8(self.model_type)
    }

    #[inline(always)]
    pub fn set_model_type(&mut self, mtype: crate::safety::ModelType) {
        self.model_type = mtype.to_u8();
    }
}

/// Compact storage layout containing radio settings and active model (488 bytes).
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct RadioStorage {
    pub radio: RadioConfig,
    pub active_model: ModelConfig,
}

impl RadioStorage {
    pub const fn empty() -> Self {
        Self {
            radio: RadioConfig::default_factory(),
            active_model: ModelConfig::default_for_index(0),
        }
    }

    #[allow(dead_code)]
    pub const fn default_factory() -> Self {
        Self::empty()
    }

    pub fn init_default(&mut self) {
        self.radio = RadioConfig::default_factory();
        self.active_model = ModelConfig::default_for_index(0);
    }

    #[inline(always)]
    pub fn active_model(&self) -> &ModelConfig {
        &self.active_model
    }

    #[inline(always)]
    pub fn active_model_mut(&mut self) -> &mut ModelConfig {
        &mut self.active_model
    }

    /// Check if a mix source ID (0..40, matching `SOURCE_NAMES`) is available on this radio.
    /// Filters out unconfigured switches, potentiometers, and unmapped channels.
    pub fn is_source_available(&self, src: u8) -> bool {
        match src {
            0..=4 => true, // None, Roll, Pitch, Thr, Yaw are always available
            5 => self.radio.get_adc_mode(4).is_pot(), // VRA (ADC ch 4)
            6 => self.radio.get_adc_mode(5).is_pot(), // VRB (ADC ch 5)
            7 => matches!(self.radio.get_adc_mode(0), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SA
            8 => matches!(self.radio.get_adc_mode(1), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SB
            9 => matches!(self.radio.get_adc_mode(2), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SC
            10 => matches!(self.radio.get_adc_mode(3), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SD
            11 => true, // MAX constant is always available
            12..=15 => true, // CH1..CH4 primary flight channels are always active
            16..=29 => {
                // CH5..CH18: available if assigned to an active source or targeted by a mix line
                let ch_idx = (src - 12) as usize; // 4..17
                let aux_idx = ch_idx - 4; // 0..13
                if self.active_model.wing_tail_mix == 3 && ch_idx == 5 {
                    return true; // Flaperon drives CH6
                }
                if self.active_model.aux_channels[aux_idx] != 0 {
                    return true;
                }
                self.active_model.mixes.iter().any(|m| m.target_ch == (ch_idx + 1) as u8)
            }
            30 => true, // Thr+ is always available
            31 => self.radio.ext_switches != 0, // SE
            32 => self.radio.ext_switches != 0, // SF
            33 => self.radio.j4_sg_sh_en != 0,   // SG
            34 => self.radio.j4_sg_sh_en != 0,   // SH
            35 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(6), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWI (ADC ch 6 / PC1)
            36 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(7), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWJ (ADC ch 7 / PC2)
            37 => self.radio.ext_adc != 0 && self.radio.get_adc_mode(6).is_pot(), // VRC (ADC ch 6 / PC1)
            38 => self.radio.ext_adc != 0 && self.radio.get_adc_mode(7).is_pot(), // VRD (ADC ch 7 / PC2)
            39 => self.radio.ext_adc != 0 && self.radio.get_adc_mode(8).is_pot(), // VRE (ADC ch 8 / PC3)
            40 => self.radio.ext_adc != 0 && self.radio.get_adc_mode(9).is_pot(), // VRF (ADC ch 9 / PC4)
            41 => self.radio.ext_adc != 0 && self.radio.get_adc_mode(10).is_pot(), // VRG (ADC ch 10 / PC5)
            42 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(8), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWK (ADC ch 8 / PC3)
            43 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(9), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWL (ADC ch 9 / PC4)
            44 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(10), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWM (ADC ch 10 / PC5)
            _ => false,
        }
    }

    /// Check if a switch condition index (0..28, matching `SWITCH_COND_NAMES`) is valid given hardware config.
    pub fn is_switch_cond_available(&self, cond: u8) -> bool {
        match cond {
            0 => true, // Always ON
            1 | 2 => matches!(self.radio.get_adc_mode(0), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SA
            3 | 5 => matches!(self.radio.get_adc_mode(1), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SB Up, Down
            4 => self.radio.get_adc_mode(1) == AdcInputMode::ThreePos, // SB Mid
            6 | 8 => matches!(self.radio.get_adc_mode(2), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SC Up, Down
            7 => self.radio.get_adc_mode(2) == AdcInputMode::ThreePos, // SC Mid
            9 | 10 => matches!(self.radio.get_adc_mode(3), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SD
            11 | 12 => self.radio.ext_switches != 0, // SE
            13 | 14 => self.radio.ext_switches != 0, // SF
            15 | 16 => self.radio.j4_sg_sh_en != 0, // SG
            17 | 18 => self.radio.j4_sg_sh_en != 0, // SH
            19 | 20 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(6), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWI
            21 | 22 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(7), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWJ
            23 | 24 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(8), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWK
            25 | 26 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(9), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWL
            27 | 28 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(10), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWM
            _ => false,
        }
    }

    /// Check if a channel index (1..=18) is configured / active in the active model.
    pub fn is_channel_configured(&self, ch: u8) -> bool {
        if ch == 0 || ch > 18 {
            return false;
        }
        if ch <= 4 {
            return true; // CH1..CH4 primary controls
        }
        let ch_idx = (ch - 1) as usize; // 4..17
        let aux_idx = ch_idx - 4; // 0..13
        if self.active_model.wing_tail_mix == 3 && ch == 6 {
            return true; // Flaperon right aileron
        }
        if self.active_model.aux_channels[aux_idx] != 0 {
            return true;
        }
        self.active_model.mixes.iter().any(|m| m.target_ch == ch)
    }

    /// Check if a weight modulation source index (0..=36, matching `WEIGHT_SRC_NAMES`) is available.
    pub fn is_weight_src_available(&self, ws: u8) -> bool {
        match ws {
            0 => true, // Fixed weight
            1..=18 => self.is_channel_configured(ws), // Bipolar CH1..CH18
            19..=36 => self.is_channel_configured(ws - 18), // Unipolar CH1+..CH18+
            _ => false,
        }
    }

    /// Find the next available mix source index, wrapping around.
    pub fn next_available_source(&self, cur: u8, max_len: usize) -> u8 {
        let max = max_len as u8;
        let mut candidate = (cur + 1) % max;
        while candidate != cur {
            if self.is_source_available(candidate) {
                return candidate;
            }
            candidate = (candidate + 1) % max;
        }
        cur
    }

    /// Find the previous available mix source index, wrapping around.
    pub fn prev_available_source(&self, cur: u8, max_len: usize) -> u8 {
        let max = max_len as u8;
        let mut candidate = if cur == 0 { max - 1 } else { cur - 1 };
        while candidate != cur {
            if self.is_source_available(candidate) {
                return candidate;
            }
            candidate = if candidate == 0 { max - 1 } else { candidate - 1 };
        }
        cur
    }

    /// Find the next available weight modulation source index, wrapping around.
    pub fn next_available_weight_src(&self, cur: u8, max_len: usize) -> u8 {
        let max = max_len as u8;
        let mut candidate = (cur + 1) % max;
        while candidate != cur {
            if self.is_weight_src_available(candidate) {
                return candidate;
            }
            candidate = (candidate + 1) % max;
        }
        cur
    }

    /// Find the previous available weight modulation source index, wrapping around.
    pub fn prev_available_weight_src(&self, cur: u8, max_len: usize) -> u8 {
        let max = max_len as u8;
        let mut candidate = if cur == 0 { max - 1 } else { cur - 1 };
        while candidate != cur {
            if self.is_weight_src_available(candidate) {
                return candidate;
            }
            candidate = if candidate == 0 { max - 1 } else { candidate - 1 };
        }
        cur
    }

    /// Find the next available switch condition index, wrapping around.
    pub fn next_available_switch_cond(&self, cur: u8, max_len: usize) -> u8 {
        let max = max_len as u8;
        let mut candidate = (cur + 1) % max;
        while candidate != cur {
            if self.is_switch_cond_available(candidate) {
                return candidate;
            }
            candidate = (candidate + 1) % max;
        }
        cur
    }

    /// Find the previous available switch condition index, wrapping around.
    pub fn prev_available_switch_cond(&self, cur: u8, max_len: usize) -> u8 {
        let max = max_len as u8;
        let mut candidate = if cur == 0 { max - 1 } else { cur - 1 };
        while candidate != cur {
            if self.is_switch_cond_available(candidate) {
                return candidate;
            }
            candidate = if candidate == 0 { max - 1 } else { candidate - 1 };
        }
        cur
    }

    /// Check if a physical switch index (0..6: None, SA..SF) is available for Dual Rate selection.
    pub fn is_dr_switch_available(&self, sw: u8) -> bool {
        match sw {
            0 => true, // None
            1 => matches!(self.radio.get_adc_mode(0), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SA
            2 => matches!(self.radio.get_adc_mode(1), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SB
            3 => matches!(self.radio.get_adc_mode(2), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SC
            4 => matches!(self.radio.get_adc_mode(3), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SD
            5 => self.radio.ext_switches != 0, // SE
            6 => self.radio.ext_switches != 0, // SF
            _ => false,
        }
    }

    /// Find next available Dual Rate switch (0..6).
    pub fn next_available_dr_switch(&self, cur: u8) -> u8 {
        let mut candidate = (cur + 1) % 7;
        while candidate != cur {
            if self.is_dr_switch_available(candidate) {
                return candidate;
            }
            candidate = (candidate + 1) % 7;
        }
        cur
    }

    /// Find previous available Dual Rate switch (0..6).
    pub fn prev_available_dr_switch(&self, cur: u8) -> u8 {
        let mut candidate = if cur == 0 { 6 } else { cur - 1 };
        while candidate != cur {
            if self.is_dr_switch_available(candidate) {
                return candidate;
            }
            candidate = if candidate == 0 { 6 } else { candidate - 1 };
        }
        cur
    }

    /// Check if an AUX channel input source (from `crate::hw::AUX_SOURCES`) is hardware-available.
    pub fn is_aux_source_available(&self, src: u8) -> bool {
        match src {
            0..=4 => true, // None, Roll, Pitch, Thr, Yaw
            5 => self.radio.get_adc_mode(4).is_pot(), // VRA
            6 => self.radio.get_adc_mode(5).is_pot(), // VRB
            7 => matches!(self.radio.get_adc_mode(0), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SA
            8 => matches!(self.radio.get_adc_mode(1), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SB
            9 => matches!(self.radio.get_adc_mode(2), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SC
            10 => matches!(self.radio.get_adc_mode(3), AdcInputMode::TwoPos | AdcInputMode::ThreePos | AdcInputMode::InstantTrim), // SD
            31 => self.radio.ext_switches != 0, // SE
            32 => self.radio.ext_switches != 0, // SF
            33 => self.radio.j4_sg_sh_en != 0,   // SG
            34 => self.radio.j4_sg_sh_en != 0,   // SH
            35 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(6), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWI
            36 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(7), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWJ
            37 => self.radio.ext_adc != 0 && self.radio.get_adc_mode(6).is_pot(), // VRC
            38 => self.radio.ext_adc != 0 && self.radio.get_adc_mode(7).is_pot(), // VRD
            39 => self.radio.ext_adc != 0 && self.radio.get_adc_mode(8).is_pot(), // VRE
            40 => self.radio.ext_adc != 0 && self.radio.get_adc_mode(9).is_pot(), // VRF
            41 => self.radio.ext_adc != 0 && self.radio.get_adc_mode(10).is_pot(), // VRG
            42 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(8), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWK
            43 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(9), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWL
            44 => self.radio.ext_adc != 0 && matches!(self.radio.get_adc_mode(10), AdcInputMode::TwoPos | AdcInputMode::ThreePos), // SWM
            _ => false,
        }
    }

    /// Cycle to next available AUX source in a given slice of source IDs.
    pub fn next_available_aux_source(&self, cur: u8, sources: &[u8]) -> u8 {
        let cur_pos = sources.iter().position(|&s| s == cur).unwrap_or(0);
        let len = sources.len();
        let mut idx = (cur_pos + 1) % len;
        while idx != cur_pos {
            let cand = sources[idx];
            if self.is_aux_source_available(cand) {
                return cand;
            }
            idx = (idx + 1) % len;
        }
        cur
    }

    /// Cycle to previous available AUX source in a given slice of source IDs.
    pub fn prev_available_aux_source(&self, cur: u8, sources: &[u8]) -> u8 {
        let cur_pos = sources.iter().position(|&s| s == cur).unwrap_or(0);
        let len = sources.len();
        let mut idx = if cur_pos == 0 { len - 1 } else { cur_pos - 1 };
        while idx != cur_pos {
            let cand = sources[idx];
            if self.is_aux_source_available(cand) {
                return cand;
            }
            idx = if idx == 0 { len - 1 } else { idx - 1 };
        }
        cur
    }

    /// Check if an Arm Switch condition (0..=14: 0: None, 1..=14: SA^..SFv) is available.
    pub fn is_arm_switch_available(&self, sw: u8) -> bool {
        match sw {
            0 => true, // None
            1..=14 => self.is_switch_cond_available(sw),
            _ => false,
        }
    }

    /// Cycle to next available Arm Switch (0..=14).
    pub fn next_available_arm_switch(&self, cur: u8) -> u8 {
        let mut candidate = (cur + 1) % 15;
        while candidate != cur {
            if self.is_arm_switch_available(candidate) {
                return candidate;
            }
            candidate = (candidate + 1) % 15;
        }
        cur
    }

    /// Cycle to previous available Arm Switch (0..=14).
    pub fn prev_available_arm_switch(&self, cur: u8) -> u8 {
        let mut candidate = if cur == 0 { 14 } else { cur - 1 };
        while candidate != cur {
            if self.is_arm_switch_available(candidate) {
                return candidate;
            }
            candidate = if candidate == 0 { 14 } else { candidate - 1 };
        }
        cur
    }

    /// Check if a Timer Source (0..=17: 0: Off, 1: THs, 2: THt, 3: Always On, 4..=17: SA^..SFv) is available.
    pub fn is_timer_source_available(&self, ts: u8) -> bool {
        match ts {
            0..=3 => true, // Off, THs, THt, Always On
            4..=17 => self.is_switch_cond_available(ts - 3),
            _ => false,
        }
    }

    /// Cycle to next available Timer Source (0..=17).
    pub fn next_available_timer_source(&self, cur: u8) -> u8 {
        let mut candidate = (cur + 1) % 18;
        while candidate != cur {
            if self.is_timer_source_available(candidate) {
                return candidate;
            }
            candidate = (candidate + 1) % 18;
        }
        cur
    }

    /// Cycle to previous available Timer Source (0..=17).
    pub fn prev_available_timer_source(&self, cur: u8) -> u8 {
        let mut candidate = if cur == 0 { 17 } else { cur - 1 };
        while candidate != cur {
            if self.is_timer_source_available(candidate) {
                return candidate;
            }
            candidate = if candidate == 0 { 17 } else { candidate - 1 };
        }
        cur
    }

    /// Sanitize all radio and model parameters to valid operating bounds.
    pub fn sanitize(&mut self) {
        self.radio.magic = FLASH_MAGIC;
        self.radio.version = CONFIG_VERSION;
        if self.radio.vbat_warn_deci < 35 || self.radio.vbat_warn_deci > 60 {
            self.radio.vbat_warn_deci = 44;
        }
        if self.radio.active_model >= NUM_MODELS as u8 {
            self.radio.active_model = 0;
        }
        if self.radio.backlight_brightness == 0 || self.radio.backlight_brightness > 10 {
            self.radio.backlight_brightness = 10;
        }
        if self.radio.lcd_contrast < 15 || self.radio.lcd_contrast > 55 {
            self.radio.lcd_contrast = 37;
        }
        if self.radio.usb_mode > 3 {
            self.radio.usb_mode = 0;
        }
        if self.radio.ext_module_pwr > 1 {
            self.radio.ext_module_pwr = 0;
        }
        if self.radio.throttle_trim > 2 {
            self.radio.throttle_trim = 0;
        }
        if self.radio.tone_style > 1 {
            self.radio.tone_style = 1;
        }
        if self.radio.servo_rate_hz < 50 || self.radio.servo_rate_hz > 400 {
            self.radio.servo_rate_hz = 50;
        }
        if self.radio.rx_out_mode > 1 {
            self.radio.rx_out_mode = 0;
        }
        if self.radio.rx_serial_proto > 1 {
            self.radio.rx_serial_proto = 0;
        }
        if self.radio.ext_switches > 1 {
            self.radio.ext_switches = 0;
        }
        if self.radio.rear_left_func > 2 {
            self.radio.rear_left_func = 0;
        }
        if self.radio.rear_right_func > 2 {
            self.radio.rear_right_func = 0;
        }
        if self.radio.touch_disabled > 1 {
            self.radio.touch_disabled = 0;
        }
        if self.radio.j4_sg_sh_en > 1 {
            self.radio.j4_sg_sh_en = 0;
        }
        if self.radio.switch_beep > 1 {
            self.radio.switch_beep = 1;
        }

        for stick in self.radio.sticks.iter_mut() {
            if stick.min >= stick.center
                || stick.center >= stick.max
                || stick.max > crate::adc::ADC_MAX
            {
                stick.min = crate::adc::ADC_MIN;
                stick.center = crate::adc::ADC_CENTER;
                stick.max = crate::adc::ADC_MAX;
            }
        }
        if self.radio.ext_adc > 1 {
            self.radio.ext_adc = 0;
        }
        for pot in self.radio.aux_pots.iter_mut() {
            if pot.min >= pot.center || pot.center >= pot.max || pot.max > crate::adc::ADC_MAX {
                pot.min = crate::adc::ADC_MIN;
                pot.center = crate::adc::ADC_CENTER;
                pot.max = crate::adc::ADC_MAX;
            }
        }
        for mode in self.radio.adc_modes.iter_mut() {
            if *mode > 6 {
                *mode = 0; // Default
            }
        }
        for slot in self.radio.telem_slots.iter_mut() {
            if *slot > 12 {
                *slot = 12; // OFF
            }
        }
        if self.radio.trainer_config > 3 {
            self.radio.trainer_config = 0;
        }

        let m = &mut self.active_model;
        if m.model_type > 4 {
            m.model_type = 0;
        }
        if m.rf_protocol > 1 {
            m.rf_protocol = 0;
        }
        if m.crsf_baud > 4 {
            m.crsf_baud = 2;
        }
        if m.crsf_half_duplex > 1 {
            m.crsf_half_duplex = 0;
        }
        for axis in 0..3 {
            if m.dr_high[axis] < 30 || m.dr_high[axis] > 100 {
                m.dr_high[axis] = 100;
            }
            if m.dr_low[axis] < 30 || m.dr_low[axis] > 100 {
                m.dr_low[axis] = 70;
            }
            if m.expo_high[axis] < -100 || m.expo_high[axis] > 100 {
                m.expo_high[axis] = 0;
            }
            if m.expo_low[axis] < -100 || m.expo_low[axis] > 100 {
                m.expo_low[axis] = 0;
            }
        }
        if m.dr_switch > 6 {
            m.dr_switch = 0;
        }
        if m.wing_tail_mix > 3 {
            m.wing_tail_mix = 0;
        }
        if m.template_diff < -100 || m.template_diff > 100 {
            m.template_diff = 0;
        }
        if m.failsafe_thr < 900 || m.failsafe_thr > 2100 {
            m.failsafe_thr = 1000;
        }
        if m.aux_channels[0..6] == [0; 6] {
            m.aux_channels = [7, 8, 5, 6, 9, 10, 0, 0, 0, 0, 0, 0, 0, 0];
        }
        for src in m.aux_channels.iter_mut() {
            if *src > 40 {
                *src = 0;
            }
        }
        for mix in m.mixes.iter_mut() {
            if mix.target_ch > 18 || mix.source > 40 || mix.weight_src > 36 || mix.mode > 2 || mix.switch > 22 {
                *mix = MixLine::disabled();
            }
        }
        if m.thr_curve_pts != 5 && m.thr_curve_pts != 7 && m.thr_curve_pts != 9 {
            m.thr_curve_pts = 5;
            m.thr_curve = [0, 25, 50, 75, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        }
        if m.arm_switch > 14 {
            m.arm_switch = 0;
        }
        if m.timer_source > 17 {
            m.timer_source = 0;
        }
        if m.timer_secs > 3600 {
            m.timer_secs = 0;
        }
        if m.rf_power > 3 {
            m.rf_power = 0;
        }
        if m.servo_rate_hz < 50 || m.servo_rate_hz > 400 {
            m.servo_rate_hz = 50;
        }
        if m.rx_out_mode > 1 {
            m.rx_out_mode = 0;
        }
        if m.rx_serial_proto > 1 {
            m.rx_serial_proto = 0;
        }
        if m.name[0] == 0 || m.name[0] == 0xFF {
            let num = self.radio.active_model + 1;
            let digit1 = b'0' + (num / 10);
            let digit2 = b'0' + (num % 10);
            m.name = [
                b'M', b'O', b'D', b'E', b'L', b' ', digit1, digit2, b' ', b' ',
            ];
        }
    }
}

const _: () = assert!(core::mem::size_of::<RadioConfig>() == 148);
const _: () = assert!(core::mem::size_of::<MixLine>() == 8);
const _: () = assert!(core::mem::size_of::<ModelConfig>() == 348);
const _: () = assert!(core::mem::size_of::<RadioStorage>() == 496);

static mut GLOBAL_STORAGE: RadioStorage = RadioStorage::empty();

/// Get global mutable reference to persistent RadioStorage.
#[allow(static_mut_refs)]
pub fn get_storage() -> &'static mut RadioStorage {
    unsafe { &mut GLOBAL_STORAGE }
}

#[allow(dead_code)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum FlashError {
    AddressMisaligned,
    LengthMisaligned,
    OutOfBounds,
    ProgrammingError,
    WriteProtectionError,
    Timeout,
    UnlockFailed,
}

impl core::fmt::Display for FlashError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl embedded_storage_async::nor_flash::NorFlashError for FlashError {
    fn kind(&self) -> embedded_storage_async::nor_flash::NorFlashErrorKind {
        match self {
            FlashError::AddressMisaligned | FlashError::LengthMisaligned => {
                embedded_storage_async::nor_flash::NorFlashErrorKind::NotAligned
            }
            FlashError::OutOfBounds => {
                embedded_storage_async::nor_flash::NorFlashErrorKind::OutOfBounds
            }
            _ => embedded_storage_async::nor_flash::NorFlashErrorKind::Other,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Stm32Flash;

impl embedded_storage_async::nor_flash::ErrorType for Stm32Flash {
    type Error = FlashError;
}

impl embedded_storage_async::nor_flash::ReadNorFlash for Stm32Flash {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let capacity = self.capacity() as u32;
        if offset as usize + bytes.len() > capacity as usize {
            return Err(FlashError::OutOfBounds);
        }
        let src = offset as *const u8;
        unsafe {
            for (i, b) in bytes.iter_mut().enumerate() {
                *b = core::ptr::read_volatile(src.add(i));
            }
        }
        Ok(())
    }

    fn capacity(&self) -> usize {
        0x0802_0000
    }
}

impl embedded_storage_async::nor_flash::NorFlash for Stm32Flash {
    const WRITE_SIZE: usize = 2; // 16-bit halfword programming on STM32F0
    const ERASE_SIZE: usize = 2048; // 2 KB page erase

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if !from.is_multiple_of(2048) || !to.is_multiple_of(2048) || from >= to {
            return Err(FlashError::AddressMisaligned);
        }

        #[cfg(test)]
        {
            let _ = (from, to);
            return Ok(());
        }

        #[cfg(not(test))]
        cortex_m::interrupt::free(|_| unsafe {
            let flash = &*pac::FLASH::ptr();

            if flash.cr.read().lock().bit_is_set() {
                flash.keyr.write(|w| w.bits(FLASH_KEY1));
                flash.keyr.write(|w| w.bits(FLASH_KEY2));
            }
            if flash.cr.read().lock().bit_is_set() {
                return Err(FlashError::UnlockFailed);
            }

            let mut page_addr = from;
            while page_addr < to {
                crate::watchdog::feed();

                let mut timeout = 1_000_000u32;
                while flash.sr.read().bsy().bit_is_set() && timeout > 0 {
                    timeout -= 1;
                }
                if timeout == 0 {
                    return Err(FlashError::Timeout);
                }

                flash
                    .sr
                    .write(|w| w.eop().event().wrprt().error().pgerr().error());

                flash.cr.write_with_zero(|w| w.per().page_erase());
                flash.ar.write_with_zero(|w| w.far().bits(page_addr));
                flash
                    .cr
                    .write_with_zero(|w| w.per().page_erase().strt().start());

                timeout = 1_000_000;
                while flash.sr.read().bsy().bit_is_set() && timeout > 0 {
                    timeout -= 1;
                }
                if timeout == 0 {
                    return Err(FlashError::Timeout);
                }

                if flash.sr.read().wrprt().is_error() {
                    flash.sr.write(|w| w.wrprt().error());
                    return Err(FlashError::WriteProtectionError);
                }
                flash.sr.write(|w| w.eop().event());

                page_addr += 2048;
            }

            flash.cr.write_with_zero(|w| w.bits(0));
            flash.cr.write(|w| w.lock().locked());
            crate::watchdog::feed();

            Ok(())
        })
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if !offset.is_multiple_of(2) {
            return Err(FlashError::AddressMisaligned);
        }
        if bytes.is_empty() {
            return Ok(());
        }

        #[cfg(test)]
        {
            let _ = (offset, bytes);
            return Ok(());
        }

        #[cfg(not(test))]
        cortex_m::interrupt::free(|_| unsafe {
            let flash = &*pac::FLASH::ptr();

            if flash.cr.read().lock().bit_is_set() {
                flash.keyr.write(|w| w.bits(FLASH_KEY1));
                flash.keyr.write(|w| w.bits(FLASH_KEY2));
            }
            if flash.cr.read().lock().bit_is_set() {
                return Err(FlashError::UnlockFailed);
            }

            let mut timeout = 1_000_000u32;
            while flash.sr.read().bsy().bit_is_set() && timeout > 0 {
                timeout -= 1;
            }
            if timeout == 0 {
                return Err(FlashError::Timeout);
            }

            flash
                .sr
                .write(|w| w.eop().event().wrprt().error().pgerr().error());
            flash.cr.write_with_zero(|w| w.pg().program());

            let halfwords = bytes.len() / 2;
            let src = bytes.as_ptr() as *const u16;
            let dst = offset as *mut u16;

            for i in 0..halfwords {
                let hw = core::ptr::read_unaligned(src.add(i));
                core::ptr::write_volatile(dst.add(i), hw);

                timeout = 100_000;
                while flash.sr.read().bsy().bit_is_set() && timeout > 0 {
                    timeout -= 1;
                }
                if timeout == 0 {
                    flash.cr.write_with_zero(|w| w.bits(0));
                    flash.cr.write(|w| w.lock().locked());
                    return Err(FlashError::Timeout);
                }

                let sr = flash.sr.read();
                if sr.pgerr().is_error() {
                    flash.sr.write(|w| w.pgerr().error());
                    flash.cr.write_with_zero(|w| w.bits(0));
                    flash.cr.write(|w| w.lock().locked());
                    return Err(FlashError::ProgrammingError);
                }
                if sr.wrprt().is_error() {
                    flash.sr.write(|w| w.wrprt().error());
                    flash.cr.write_with_zero(|w| w.bits(0));
                    flash.cr.write(|w| w.lock().locked());
                    return Err(FlashError::WriteProtectionError);
                }
                flash.sr.write(|w| w.eop().event());

                if (i & 0x7F) == 0 {
                    crate::watchdog::feed();
                }
            }

            if bytes.len() % 2 != 0 {
                let last_byte = bytes[bytes.len() - 1];
                let hw = (last_byte as u16) | 0xFF00;
                core::ptr::write_volatile(dst.add(halfwords), hw);

                timeout = 100_000;
                while flash.sr.read().bsy().bit_is_set() && timeout > 0 {
                    timeout -= 1;
                }
                if timeout == 0 {
                    flash.cr.write_with_zero(|w| w.bits(0));
                    flash.cr.write(|w| w.lock().locked());
                    return Err(FlashError::Timeout);
                }

                let sr = flash.sr.read();
                if sr.pgerr().is_error() {
                    flash.sr.write(|w| w.pgerr().error());
                    flash.cr.write_with_zero(|w| w.bits(0));
                    flash.cr.write(|w| w.lock().locked());
                    return Err(FlashError::ProgrammingError);
                }
                if sr.wrprt().is_error() {
                    flash.sr.write(|w| w.wrprt().error());
                    flash.cr.write_with_zero(|w| w.bits(0));
                    flash.cr.write(|w| w.lock().locked());
                    return Err(FlashError::WriteProtectionError);
                }
                flash.sr.write(|w| w.eop().event());
            }

            flash.cr.write_with_zero(|w| w.bits(0));
            flash.cr.write(|w| w.lock().locked());
            crate::watchdog::feed();

            Ok(())
        })
    }
}

impl embedded_storage_async::nor_flash::MultiwriteNorFlash for Stm32Flash {}

fn block_on<F: core::future::Future>(mut future: F) -> F::Output {
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    fn noop_clone(_: *const ()) -> RawWaker {
        RawWaker::new(core::ptr::null(), &NOOP_VTABLE)
    }
    fn noop(_: *const ()) {}

    static NOOP_VTABLE: RawWakerVTable = RawWakerVTable::new(noop_clone, noop, noop, noop);

    let raw_waker = RawWaker::new(core::ptr::null(), &NOOP_VTABLE);
    let waker = unsafe { Waker::from_raw(raw_waker) };
    let mut cx = Context::from_waker(&waker);

    let mut pinned = unsafe { core::pin::Pin::new_unchecked(&mut future) };
    loop {
        match pinned.as_mut().poll(&mut cx) {
            Poll::Ready(output) => return output,
            Poll::Pending => {
                #[cfg(not(test))]
                crate::watchdog::feed();
            }
        }
    }
}

#[inline(always)]
fn storage_config() -> sequential_storage::map::MapConfig<Stm32Flash> {
    sequential_storage::map::MapConfig::new(FLASH_STORAGE_ADDR as u32..FLASH_STORAGE_END as u32)
}
/// Legacy ModelConfig v1 (128 bytes) from prior releases.
/// Used for transparent, lossless migration to v2 (348 bytes).
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct LegacyModelConfigV1 {
    pub name: [u8; 10],
    pub model_type: u8,
    pub _pad0: u8,
    pub rx_id: u32,
    pub trims: [i8; 4],
    pub channel_reverse: u32,
    pub dr_switch: u8,
    pub thr_curve_pts: u8,
    pub thr_curve_smooth: u8,
    pub thr_curve: [u8; 9],
    pub dr_high: [u8; 3],
    pub dr_low: [u8; 3],
    pub expo_high: [i8; 3],
    pub expo_low: [i8; 3],
    pub timer_secs: u16,
    pub timer_source: u8,
    pub rf_power: u8,
    pub failsafe_thr: u16,
    pub aux_channels: [u8; 14],
    pub wing_tail_mix: u8,
    pub template_diff: i8,
    pub mixes: [LegacyMixLineV1; 8],
    pub failsafe_mode: u8,
    pub failsafe_timeout: u8,
    pub rf_protocol: u8,
    pub crsf_baud: u8,
    pub arm_switch: u8,
    pub rx_out_mode: u8,
    pub servo_rate_hz: u16,
    pub rx_serial_proto: u8,
    pub crsf_half_duplex: u8,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct LegacyMixLineV1 {
    pub target_ch: u8,
    pub source: u8,
    pub weight: i8,
    pub offset: i8,
    pub switch: u8,
    pub mode: u8,
}

const _: () = assert!(core::mem::size_of::<LegacyModelConfigV1>() == 128);
const _: () = assert!(core::mem::size_of::<LegacyMixLineV1>() == 6);

pub fn unpack_legacy_v1_model(v1: &LegacyModelConfigV1) -> ModelConfig {
    let mut v2 = ModelConfig::default_for_index(0);
    v2.name = v1.name;
    v2.model_type = v1.model_type;
    v2.rx_id = v1.rx_id;
    v2.trims = v1.trims;
    v2.channel_reverse = v1.channel_reverse;
    v2.dr_switch = v1.dr_switch;
    v2.thr_curve_pts = v1.thr_curve_pts;
    v2.thr_curve_smooth = v1.thr_curve_smooth;
    // Copy the first 9 curve points, rest are 0
    v2.thr_curve[0..9].copy_from_slice(&v1.thr_curve);
    v2.dr_high = v1.dr_high;
    v2.dr_low = v1.dr_low;
    v2.expo_high = v1.expo_high;
    v2.expo_low = v1.expo_low;
    v2.timer_secs = v1.timer_secs;
    v2.timer_source = v1.timer_source;
    v2.rf_power = v1.rf_power;
    v2.failsafe_thr = v1.failsafe_thr;
    v2.aux_channels = v1.aux_channels;
    v2.wing_tail_mix = v1.wing_tail_mix;
    v2.template_diff = v1.template_diff;

    // Migrate first 8 mixer lines
    for i in 0..8 {
        v2.mixes[i] = MixLine {
            target_ch: v1.mixes[i].target_ch,
            source: v1.mixes[i].source,
            weight_src: 0, // default fixed weight
            weight: v1.mixes[i].weight,
            offset: v1.mixes[i].offset,
            switch: v1.mixes[i].switch,
            mode: v1.mixes[i].mode,
            _reserved: 0,
        };
    }
    // Remaining lines 8..32 remain disabled

    v2.failsafe_mode = v1.failsafe_mode;
    v2.failsafe_timeout = v1.failsafe_timeout;
    v2.rf_protocol = v1.rf_protocol;
    v2.crsf_baud = v1.crsf_baud;
    v2.arm_switch = v1.arm_switch;
    v2.rx_out_mode = v1.rx_out_mode;
    v2.servo_rate_hz = v1.servo_rate_hz;
    v2.rx_serial_proto = v1.rx_serial_proto;
    v2.crsf_half_duplex = v1.crsf_half_duplex;

    v2
}

impl<'a> sequential_storage::map::Value<'a> for RadioConfig {
    fn serialize_into(
        &self,
        buffer: &mut [u8],
    ) -> Result<usize, sequential_storage::map::SerializationError> {
        let size = core::mem::size_of::<Self>();
        if buffer.len() < size {
            return Err(sequential_storage::map::SerializationError::BufferTooSmall);
        }
        let src = self as *const Self as *const u8;
        unsafe {
            core::ptr::copy_nonoverlapping(src, buffer.as_mut_ptr(), size);
        }
        Ok(size)
    }

    fn deserialize_from(
        buffer: &'a [u8],
    ) -> Result<(Self, usize), sequential_storage::map::SerializationError> {
        let size = core::mem::size_of::<Self>();
        if buffer.len() >= size {
            let mut val = Self::default_factory();
            let dst = &mut val as *mut Self as *mut u8;
            unsafe {
                core::ptr::copy_nonoverlapping(buffer.as_ptr(), dst, size);
            }
            Ok((val, size))
        } else if buffer.len() >= 140 {
            // Backward-compatible loading from v0.2.0 (140 bytes)
            let mut val = Self::default_factory();
            let dst = &mut val as *mut Self as *mut u8;
            unsafe {
                core::ptr::copy_nonoverlapping(buffer.as_ptr(), dst, 140);
            }
            Ok((val, 140))
        } else {
            Err(sequential_storage::map::SerializationError::BufferTooSmall)
        }
    }
}

impl<'a> sequential_storage::map::Value<'a> for ModelConfig {
    fn serialize_into(
        &self,
        buffer: &mut [u8],
    ) -> Result<usize, sequential_storage::map::SerializationError> {
        let size = core::mem::size_of::<Self>();
        if buffer.len() < size {
            return Err(sequential_storage::map::SerializationError::BufferTooSmall);
        }
        let src = self as *const Self as *const u8;
        unsafe {
            core::ptr::copy_nonoverlapping(src, buffer.as_mut_ptr(), size);
        }
        Ok(size)
    }

    fn deserialize_from(
        buffer: &'a [u8],
    ) -> Result<(Self, usize), sequential_storage::map::SerializationError> {
        let size = core::mem::size_of::<Self>();
        if buffer.len() >= size {
            let mut val = Self::default_for_index(0);
            let dst = &mut val as *mut Self as *mut u8;
            unsafe {
                core::ptr::copy_nonoverlapping(buffer.as_ptr(), dst, size);
            }
            Ok((val, size))
        } else if buffer.len() >= 128 {
            // Backward-compatible loading from v0.2.0 (128 bytes)
            let mut v1 = core::mem::MaybeUninit::<LegacyModelConfigV1>::uninit();
            unsafe {
                core::ptr::copy_nonoverlapping(buffer.as_ptr(), v1.as_mut_ptr() as *mut u8, 128);
                let legacy = v1.assume_init();
                let v2 = unpack_legacy_v1_model(&legacy);
                Ok((v2, 128))
            }
        } else {
            Err(sequential_storage::map::SerializationError::BufferTooSmall)
        }
    }
}

fn format_and_save_all(storage: &RadioStorage) {
    let mut flash = Stm32Flash;
    let _ = block_on(flash.erase(FLASH_STORAGE_ADDR as u32, FLASH_STORAGE_END as u32));

    let mut map = sequential_storage::map::MapStorage::new(
        flash,
        storage_config(),
        sequential_storage::cache::Cache::new_uncached(),
    );
    let mut buf = [0u8; 512];
    let _ = block_on(map.store_item(&mut buf, &KEY_RADIO, &storage.radio));
    let active_idx = (storage.radio.active_model as usize).min(NUM_MODELS - 1);
    let key = KEY_MODEL_BASE + active_idx as u8;
    let _ = block_on(map.store_item(&mut buf, &key, &storage.active_model));
}

/// Initialize Flash storage and load radio configuration and active model into memory.
#[inline(never)]
pub fn load_storage_into(storage: &mut RadioStorage) {
    #[cfg(test)]
    {
        storage.init_default();
        return;
    }

    #[cfg(not(test))]
    {
        let mut map = sequential_storage::map::MapStorage::new(
            Stm32Flash,
            storage_config(),
            sequential_storage::cache::Cache::new_uncached(),
        );
        let mut buf = [0u8; 512];

        let radio_res: Result<Option<RadioConfig>, _> =
            block_on(map.fetch_item(&mut buf, &KEY_RADIO));
        if let Ok(Some(cfg)) = radio_res {
            if cfg.magic == FLASH_MAGIC && cfg.version == CONFIG_VERSION {
                storage.radio = cfg;
                let active_idx = (storage.radio.active_model as usize).min(NUM_MODELS - 1);
                let key = KEY_MODEL_BASE + active_idx as u8;
                if let Ok(Some(m)) = block_on(map.fetch_item(&mut buf, &key)) {
                    storage.active_model = m;
                } else {
                    storage.active_model = ModelConfig::default_for_index(active_idx);
                }
                storage.sanitize();
                return;
            }
        }

        // Clean factory default initialization
        storage.init_default();
        format_and_save_all(storage);
    }
}

/// Load a specific model slot into .
pub fn load_active_model(storage: &mut RadioStorage) -> bool {
    let active_idx = (storage.radio.active_model as usize).min(NUM_MODELS - 1);
    #[cfg(test)]
    {
        storage.active_model = ModelConfig::default_for_index(active_idx);
        storage.sanitize();
        return true;
    }
    #[cfg(not(test))]
    {
        let mut map = sequential_storage::map::MapStorage::new(
            Stm32Flash,
            storage_config(),
            sequential_storage::cache::Cache::new_uncached(),
        );
        let mut buf = [0u8; 512];
        let key = KEY_MODEL_BASE + active_idx as u8;
        if let Ok(Some(model)) = block_on(map.fetch_item::<ModelConfig>(&mut buf, &key)) {
            storage.active_model = model;
            storage.sanitize();
            true
        } else {
            storage.active_model = ModelConfig::default_for_index(active_idx);
            storage.sanitize();
            false
        }
    }
}

/// Save arbitrary ModelConfig to an arbitrary model slot index.
pub fn save_model_to_slot(slot_idx: usize, model: &ModelConfig) -> bool {
    #[cfg(test)]
    {
        let _ = slot_idx;
        let _ = model;
        return true;
    }
    #[cfg(not(test))]
    {
        if slot_idx >= NUM_MODELS {
            return false;
        }
        let mut map = sequential_storage::map::MapStorage::new(
            Stm32Flash,
            storage_config(),
            sequential_storage::cache::Cache::new_uncached(),
        );
        let mut buf = [0u8; 512];
        let key = KEY_MODEL_BASE + slot_idx as u8;
        block_on(map.store_item(&mut buf, &key, model)).is_ok()
    }
}

#[allow(dead_code)]
pub fn load_storage() -> RadioStorage {
    let mut storage = RadioStorage::empty();
    load_storage_into(&mut storage);
    storage
}

/// Save the currently active model to its dedicated slot in Flash.
pub fn save_active_model(storage: &RadioStorage) -> bool {
    #[cfg(test)]
    {
        let _ = storage;
        return true;
    }
    #[cfg(not(test))]
    {
        let idx = (storage.radio.active_model as usize).min(NUM_MODELS - 1);
        save_model_to_slot(idx, &storage.active_model)
    }
}

/// Save RadioConfig to Flash storage.
pub fn save_radio_config(storage: &RadioStorage) -> bool {
    #[cfg(test)]
    {
        let _ = storage;
        return true;
    }
    #[cfg(not(test))]
    {
        let mut map = sequential_storage::map::MapStorage::new(
            Stm32Flash,
            storage_config(),
            sequential_storage::cache::Cache::new_uncached(),
        );
        let mut buf = [0u8; 512];
        block_on(map.store_item(&mut buf, &KEY_RADIO, &storage.radio)).is_ok()
    }
}

pub fn save_storage(storage: &RadioStorage) -> bool {
    let mut ok = save_radio_config(storage);
    ok = ok && save_active_model(storage);
    ok
}

pub fn load_config() -> RadioConfig {
    let storage = get_storage();
    load_storage_into(storage);
    storage.radio
}

/// Read only model name without transferring or deserializing unneeded fields.
pub fn peek_model_name(slot_idx: usize) -> Option<[u8; 10]> {
    #[cfg(test)]
    {
        let _ = slot_idx;
        None
    }
    #[cfg(not(test))]
    {
        if slot_idx >= NUM_MODELS {
            return None;
        }
        let mut map = sequential_storage::map::MapStorage::new(
            Stm32Flash,
            storage_config(),
            sequential_storage::cache::Cache::new_uncached(),
        );
        let mut buf = [0u8; 512];
        let key = KEY_MODEL_BASE + slot_idx as u8;
        if let Ok(Some(model)) = block_on(map.fetch_item::<ModelConfig>(&mut buf, &key)) {
            Some(model.name)
        } else {
            None
        }
    }
}

/// Delete a model slot in Flash by removing the key from the sequential-storage map.
pub fn delete_model_slot(slot_idx: usize) -> bool {
    #[cfg(test)]
    {
        let _ = slot_idx;
        true
    }
    #[cfg(not(test))]
    {
        if slot_idx >= NUM_MODELS {
            return false;
        }
        let mut map = sequential_storage::map::MapStorage::new(
            Stm32Flash,
            storage_config(),
            sequential_storage::cache::Cache::new_uncached(),
        );
        let mut buf = [0u8; 512];
        let key = KEY_MODEL_BASE + slot_idx as u8;
        block_on(map.remove_item(&mut buf, &key)).is_ok()
    }
}

/// Read persisted receiver ID from active model if previously bound.
pub fn load_saved_rx_id() -> Option<u32> {
    #[cfg(test)]
    {
        None
    }
    #[cfg(not(test))]
    {
        let storage = get_storage();
        let active = storage.active_model();
        if active.rx_id != 0 && active.rx_id != 0xFFFF_FFFF {
            Some(active.rx_id)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adc_input_mode_detent() {
        assert_eq!(AdcInputMode::from_u8(0), AdcInputMode::Default);
        assert_eq!(AdcInputMode::from_u8(1), AdcInputMode::Off);
        assert_eq!(AdcInputMode::from_u8(2), AdcInputMode::TwoPos);
        assert_eq!(AdcInputMode::from_u8(3), AdcInputMode::ThreePos);
        assert_eq!(AdcInputMode::from_u8(4), AdcInputMode::SixPos);
        assert_eq!(AdcInputMode::from_u8(5), AdcInputMode::Pot);
        assert_eq!(AdcInputMode::from_u8(6), AdcInputMode::PotDetent);
        assert_eq!(AdcInputMode::from_u8(7), AdcInputMode::InstantTrim);
        assert!(AdcInputMode::Pot.is_pot());
        assert!(AdcInputMode::PotDetent.is_pot());
        assert!(!AdcInputMode::InstantTrim.is_pot());
        assert!(!AdcInputMode::Off.is_pot());
        assert!(AdcInputMode::Off.is_off());
        assert!(!AdcInputMode::Pot.has_detent());
        assert!(AdcInputMode::PotDetent.has_detent());
    }

    #[test]
    fn test_pot_calib_and_radio_config_defaults() {
        let radio = RadioConfig::default_factory();
        assert_eq!(radio.aux_pots.len(), 11);
        assert_eq!(radio.adc_modes.len(), 11);
        for i in 0..11 {
            assert_eq!(radio.aux_pots[i].min, crate::adc::ADC_MIN);
            assert_eq!(radio.aux_pots[i].center, crate::adc::ADC_CENTER);
            assert_eq!(radio.aux_pots[i].max, crate::adc::ADC_MAX);
            assert_eq!(radio.adc_modes[i], 0);
        }

        let pot = PotCalib::new(100, 2000, 3900);
        assert_eq!(pot.min, 100);
        assert_eq!(pot.center, 2000);
        assert_eq!(pot.max, 3900);
    }

    #[test]
    fn test_model_config_size_and_rf_power() {
        assert_eq!(core::mem::size_of::<ModelConfig>(), 348);
        assert_eq!(core::mem::size_of::<RadioConfig>(), 148);
        assert_eq!(core::mem::size_of::<RadioStorage>(), 496);

        let default_m = ModelConfig::default_for_index(0);
        assert_eq!(default_m.rf_power, 0);

        let mut storage = RadioStorage::default_factory();
        storage.active_model.rf_power = 99;
        storage.sanitize();
        assert_eq!(storage.active_model.rf_power, 0);
    }

    #[test]
    fn test_legacy_v1_migration() {
        let v1 = LegacyModelConfigV1 {
            name: *b"PLANE 01  ",
            model_type: 1, // Glider
            _pad0: 0,
            rx_id: 0x1234_5678,
            trims: [5, -3, 2, 0],
            channel_reverse: 0x05,
            dr_switch: 2,
            thr_curve_pts: 9,
            thr_curve_smooth: 1,
            thr_curve: [0, 10, 25, 40, 55, 70, 80, 90, 100],
            dr_high: [90, 85, 80],
            dr_low: [60, 55, 50],
            expo_high: [20, -10, 0],
            expo_low: [10, -5, 0],
            timer_secs: 420,
            timer_source: 2,
            rf_power: 1,
            failsafe_thr: 980,
            aux_channels: [7, 8, 5, 6, 9, 10, 11, 12, 0, 0, 0, 0, 0, 0],
            wing_tail_mix: 2,
            template_diff: 15,
            mixes: [LegacyMixLineV1 {
                target_ch: 1,
                source: 2,
                weight: 50,
                offset: -10,
                switch: 3,
                mode: 1,
            }; 8],
            failsafe_mode: 1,
            failsafe_timeout: 30,
            rf_protocol: 1,
            crsf_baud: 3,
            arm_switch: 4,
            rx_out_mode: 1,
            servo_rate_hz: 200,
            rx_serial_proto: 1,
            crsf_half_duplex: 1,
        };
        let v2 = unpack_legacy_v1_model(&v1);
        assert_eq!(&v2.name, b"PLANE 01  ");
        assert_eq!(v2.model_type, 1);
        assert_eq!(v2.rx_id, 0x1234_5678);
        assert_eq!(v2.trims, [5, -3, 2, 0]);
        assert_eq!(v2.channel_reverse, 0x05);
        assert_eq!(v2.dr_switch, 2);
        assert_eq!(v2.thr_curve_pts, 9);
        assert_eq!(v2.thr_curve_smooth, 1);
        assert_eq!(&v2.thr_curve[0..9], &[0, 10, 25, 40, 55, 70, 80, 90, 100]);
        assert_eq!(&v2.thr_curve[9..17], &[0; 8]);
        assert_eq!(v2.timer_secs, 420);
        assert_eq!(v2.template_diff, 15);
        assert_eq!(v2.mixes.len(), 32);
        for i in 0..8 {
            assert_eq!(v2.mixes[i].target_ch, 1);
            assert_eq!(v2.mixes[i].source, 2);
            assert_eq!(v2.mixes[i].weight_src, 0);
            assert_eq!(v2.mixes[i].weight, 50);
            assert_eq!(v2.mixes[i].offset, -10);
            assert_eq!(v2.mixes[i].switch, 3);
            assert_eq!(v2.mixes[i].mode, 1);
        }
        for i in 8..32 {
            assert_eq!(v2.mixes[i], MixLine::disabled());
        }
    }

    #[test]
    fn test_telem_slots_sanitize() {
        let mut storage = RadioStorage::default_factory();
        storage.radio.telem_slots = [0, 11, 12, 13];
        storage.sanitize();
        assert_eq!(storage.radio.telem_slots, [0, 11, 12, 12]);
    }

    #[test]
    fn test_trainer_config_helpers_and_sanitize() {
        let mut radio = RadioConfig::default_factory();
        assert_eq!(radio.trainer_config, 0);
        assert!(!radio.is_ppm_out_enabled());
        assert!(!radio.is_ppm_in_enabled());

        radio.set_ppm_out_enabled(true);
        assert!(radio.is_ppm_out_enabled());
        assert!(!radio.is_ppm_in_enabled());
        assert_eq!(radio.trainer_config, 1);

        radio.set_ppm_in_enabled(true);
        assert!(radio.is_ppm_out_enabled());
        assert!(radio.is_ppm_in_enabled());
        assert_eq!(radio.trainer_config, 3);

        radio.set_ppm_out_enabled(false);
        assert!(!radio.is_ppm_out_enabled());
        assert!(radio.is_ppm_in_enabled());
        assert_eq!(radio.trainer_config, 2);

        let mut storage = RadioStorage::default_factory();
        storage.radio.trainer_config = 99;
        storage.sanitize();
        assert_eq!(storage.radio.trainer_config, 0);
    }

    #[test]
    fn test_value_serialization_roundtrip_radio_config() {
        use sequential_storage::map::Value;
        let mut radio = RadioConfig::default_factory();
        radio.vbat_warn_deci = 48;
        radio.backlight_brightness = 7;
        radio.adc_modes[10] = 2; // PC5 TwoPos

        let mut buf = [0u8; 512];
        let len = radio.serialize_into(&mut buf).expect("Serialize success");
        assert_eq!(len, core::mem::size_of::<RadioConfig>());

        let (deserialized, dlen): (RadioConfig, usize) =
            RadioConfig::deserialize_from(&buf[..len]).expect("Deserialize success");
        assert_eq!(dlen, len);
        assert_eq!(deserialized, radio);
    }

    #[test]
    fn test_value_serialization_roundtrip_model_config() {
        use sequential_storage::map::Value;
        let mut model = ModelConfig::default_for_index(3);
        model.name = *b"TEST 04   ";
        model.failsafe_thr = 1150;
        model.mixes[0].target_ch = 5;
        model.mixes[0].source = 2;
        model.mixes[0].weight = 75;

        let mut buf = [0u8; 512];
        let len = model.serialize_into(&mut buf).expect("Serialize success");
        assert_eq!(len, core::mem::size_of::<ModelConfig>());

        let (deserialized, dlen): (ModelConfig, usize) =
            ModelConfig::deserialize_from(&buf[..len]).expect("Deserialize success");
        assert_eq!(dlen, len);
        assert_eq!(deserialized, model);
    }

    #[test]
    fn test_value_serialization_legacy_migration() {
        use sequential_storage::map::Value;
        let legacy = LegacyModelConfigV1 {
            name: *b"LEGACY 01 ",
            model_type: 2,
            _pad0: 0,
            rx_id: 0xDEAD_BEEF,
            trims: [1, 2, 3, 4],
            channel_reverse: 0x01,
            dr_switch: 1,
            thr_curve_pts: 5,
            thr_curve_smooth: 0,
            thr_curve: [0, 25, 50, 75, 100, 0, 0, 0, 0],
            dr_high: [100, 100, 100],
            dr_low: [70, 70, 70],
            expo_high: [0, 0, 0],
            expo_low: [0, 0, 0],
            timer_secs: 300,
            timer_source: 1,
            rf_power: 0,
            failsafe_thr: 1000,
            aux_channels: [7, 8, 5, 6, 9, 10, 0, 0, 0, 0, 0, 0, 0, 0],
            wing_tail_mix: 0,
            template_diff: 0,
            mixes: [LegacyMixLineV1 {
                target_ch: 1,
                source: 2,
                weight: 50,
                offset: 0,
                switch: 0,
                mode: 0,
            }; 8],
            failsafe_mode: 0,
            failsafe_timeout: 20,
            rf_protocol: 0,
            crsf_baud: 2,
            arm_switch: 0,
            rx_out_mode: 0,
            servo_rate_hz: 50,
            rx_serial_proto: 0,
            crsf_half_duplex: 0,
        };

        let raw_legacy = unsafe {
            core::slice::from_raw_parts(&legacy as *const LegacyModelConfigV1 as *const u8, 128)
        };

        let (migrated, len): (ModelConfig, usize) =
            ModelConfig::deserialize_from(raw_legacy).expect("Migration success");
        assert_eq!(len, 128);
        assert_eq!(&migrated.name, b"LEGACY 01 ");
        assert_eq!(migrated.rx_id, 0xDEAD_BEEF);
        assert_eq!(migrated.mixes[0].target_ch, 1);
        assert_eq!(migrated.mixes[0].weight, 50);
        assert_eq!(migrated.mixes[8], MixLine::disabled());
    }
    #[test]
    fn test_dynamic_mixer_source_and_switch_availability() {
        let mut storage = RadioStorage::default_factory();

        // Stock config: ext_switches=0, ext_adc=0, j4_sg_sh_en=0
        // Sticks (1..4), MAX (11), CH1..CH4 (12..15), Thr+ (30) always available
        for s in 0..=4 {
            assert!(storage.is_source_available(s));
        }
        assert!(storage.is_source_available(11));
        assert!(storage.is_source_available(30));

        // Pots VRA & VRB are available by default
        assert!(storage.is_source_available(5));
        assert!(storage.is_source_available(6));

        // Switches SA..SD are available by default
        assert!(storage.is_source_available(7)); // SA
        assert!(storage.is_source_available(8)); // SB
        assert!(storage.is_source_available(9)); // SC
        assert!(storage.is_source_available(10)); // SD

        // Extended switches SE..SH and SWI..SWM / VRC..VRG unavailable by default
        assert!(!storage.is_source_available(31)); // SE
        assert!(!storage.is_source_available(32)); // SF
        assert!(!storage.is_source_available(33)); // SG
        assert!(!storage.is_source_available(34)); // SH
        assert!(!storage.is_source_available(35)); // SWI
        assert!(!storage.is_source_available(36)); // SWJ
        assert!(!storage.is_source_available(37)); // VRC
        assert!(!storage.is_source_available(38)); // VRD
        assert!(!storage.is_source_available(39)); // VRE
        assert!(!storage.is_source_available(40)); // VRF
        assert!(!storage.is_source_available(41)); // VRG
        assert!(!storage.is_source_available(42)); // SWK
        assert!(!storage.is_source_available(43)); // SWL
        assert!(!storage.is_source_available(44)); // SWM

        // Enable ext_adc
        storage.radio.ext_adc = 1;
        assert!(storage.is_source_available(37)); // VRC
        assert!(storage.is_source_available(38)); // VRD
        assert!(storage.is_source_available(39)); // VRE
        assert!(storage.is_source_available(40)); // VRF
        assert!(storage.is_source_available(41)); // VRG
        storage.radio.set_adc_mode_val(10, 2); // 2-pos switch on PC5 (2 = TwoPos)
        assert!(!storage.is_source_available(41)); // VRG is now switch, not pot
        assert!(storage.is_source_available(44));  // SWM
        assert!(storage.is_switch_cond_available(27)); // SWM^
        assert!(storage.is_switch_cond_available(28)); // SWMv

        // Turn PC5 to OFF (1 = Off)
        storage.radio.set_adc_mode_val(10, 1);
        assert!(!storage.is_source_available(41)); // VRG not available when Off
        assert!(!storage.is_source_available(44)); // SWM not available when Off
        assert!(!storage.is_switch_cond_available(27)); // SWM^ not available when Off
        assert!(!storage.is_switch_cond_available(28)); // SWMv not available when Off
        assert!(!storage.is_aux_source_available(41));  // VRG not available in AUX CHANNELS
        assert!(!storage.is_aux_source_available(44));  // SWM not available in AUX CHANNELS

        // Enable ext_switches
        storage.radio.ext_switches = 1;
        assert!(storage.is_source_available(31)); // SE
        assert!(storage.is_source_available(32)); // SF
        assert!(storage.is_switch_cond_available(11)); // SE^
        assert!(storage.is_switch_cond_available(14)); // SFv

        // Enable SG/SH
        storage.radio.j4_sg_sh_en = 1;
        assert!(storage.is_source_available(33)); // SG
        assert!(storage.is_source_available(34)); // SH
        assert!(storage.is_switch_cond_available(15)); // SG^
        assert!(storage.is_switch_cond_available(18)); // SHv

        // Channels: default model has aux_channels [7, 8, 5, 6, 9, 10, 0, 0, 0, 0, 0, 0, 0, 0]
        // CH5..CH10 are configured (sources 16..21)
        for ch in 1..=10 {
            assert!(storage.is_channel_configured(ch));
            assert!(storage.is_source_available(ch + 11));
            assert!(storage.is_weight_src_available(ch)); // Bipolar
            assert!(storage.is_weight_src_available(ch + 18)); // Unipolar
        }
        // CH11..CH18 are unmapped (source=0) and no mix line targets them
        for ch in 11..=18 {
            assert!(!storage.is_channel_configured(ch));
            assert!(!storage.is_source_available(ch + 11));
            assert!(!storage.is_weight_src_available(ch));
            assert!(!storage.is_weight_src_available(ch + 18));
        }

        // Target CH11 with a mix line -> CH11 becomes configured
        storage.active_model.mixes[0].target_ch = 11;
        assert!(storage.is_channel_configured(11));
        assert!(storage.is_source_available(22)); // CH11 source
        assert!(storage.is_weight_src_available(11));
    }
}
