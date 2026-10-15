//! Input normalization, stick axis processing, switch decoding, and battery calculation.

use crate::adc;

/// 3-position switch states.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum SwitchPos {
    #[default]
    Up,
    Mid,
    Down,
}

impl SwitchPos {
    pub fn as_char(&self) -> char {
        match self {
            SwitchPos::Up => 'U',
            SwitchPos::Mid => 'M',
            SwitchPos::Down => 'D',
        }
    }
}

/// Normalized physical gimbal axes (-1000 .. +1000).
/// Model-type and stick-mode agnostic hardware representation:
/// - `rh`: Right Horizontal (PA0, index 0)
/// - `rv`: Right Vertical   (PA1, index 1)
/// - `lv`: Left Vertical    (PA2, index 2)
/// - `lh`: Left Horizontal  (PA3, index 3)
#[derive(Copy, Clone, Debug, Default)]
pub struct Gimbals {
    pub rh: i16,
    pub rv: i16,
    pub lv: i16,
    pub lh: i16,
}

impl Gimbals {
    /// Map physical gimbals to logical flight controls according to radio stick mode.
    #[inline(always)]
    pub fn to_flight_controls(&self, mode: crate::safety::StickMode) -> FlightControls {
        use crate::safety::StickMode;
        match mode {
            StickMode::Mode1 => FlightControls {
                aileron: self.rh,
                elevator: self.lv,
                throttle: self.rv,
                rudder: self.lh,
            },
            StickMode::Mode2 => FlightControls {
                aileron: self.rh,
                elevator: self.rv,
                throttle: self.lv,
                rudder: self.lh,
            },
            StickMode::Mode3 => FlightControls {
                aileron: self.lh,
                elevator: self.lv,
                throttle: self.rv,
                rudder: self.rh,
            },
            StickMode::Mode4 => FlightControls {
                aileron: self.lh,
                elevator: self.rv,
                throttle: self.lv,
                rudder: self.rh,
            },
        }
    }
}

/// Logical primary flight control axes (-1000 .. +1000).
#[derive(Copy, Clone, Debug, Default)]
pub struct FlightControls {
    pub aileron: i16,  // Roll (AIL)
    pub elevator: i16, // Pitch (ELE)
    pub throttle: i16, // Throttle (THR)
    pub rudder: i16,   // Yaw (RUD)
}

/// Backwards compatibility alias while migrating.
pub type Sticks = Gimbals;

/// Normalized potentiometer rotary dials (-1000 .. +1000).
#[derive(Copy, Clone, Debug)]
pub struct Pots {
    pub vr1: i16, // VRA
    pub vr2: i16, // VRB
}

/// Physical switch states on the radio.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct Switches {
    pub sa: SwitchPos,  // 2-pos
    pub sb: SwitchPos,  // 3-pos
    pub sc: SwitchPos,  // 3-pos
    pub sd: SwitchPos,  // 2-pos
    pub swe: SwitchPos, // 2-pos (Rear Left PA9)
    pub swf: SwitchPos, // 2-pos (Rear Right PA10)
    pub sg: SwitchPos,  // 2-pos (J4 PA13 SWDIO when enabled)
    pub sh: SwitchPos,  // 2-pos (J4 PA14 SWCLK when enabled)
    pub swi: SwitchPos, // 2-pos or 3-pos (PC1 / ADC11 when enabled)
    pub swj: SwitchPos, // 2-pos or 3-pos (PC2 / ADC12 when enabled)
    pub swk: SwitchPos, // 2-pos or 3-pos (PC3 / ADC13 when enabled)
    pub swl: SwitchPos, // 2-pos or 3-pos (PC4 / ADC14 when enabled)
}

impl Switches {
    pub const fn new() -> Self {
        Self {
            sa: SwitchPos::Up,
            sb: SwitchPos::Up,
            sc: SwitchPos::Up,
            sd: SwitchPos::Up,
            swe: SwitchPos::Up,
            swf: SwitchPos::Up,
            sg: SwitchPos::Up,
            sh: SwitchPos::Up,
            swi: SwitchPos::Up,
            swj: SwitchPos::Up,
            swk: SwitchPos::Up,
            swl: SwitchPos::Up,
        }
    }

    /// Detect if a switch moved between prev and self, returning the matching condition index 1..26.
    pub fn detect_condition_change(&self, prev: &Switches) -> Option<u8> {
        if self.sa != prev.sa {
            return Some(if self.sa == SwitchPos::Up { 1 } else { 2 });
        }
        if self.sb != prev.sb {
            return Some(match self.sb {
                SwitchPos::Up => 3,
                SwitchPos::Mid => 4,
                SwitchPos::Down => 5,
            });
        }
        if self.sc != prev.sc {
            return Some(match self.sc {
                SwitchPos::Up => 6,
                SwitchPos::Mid => 7,
                SwitchPos::Down => 8,
            });
        }
        if self.sd != prev.sd {
            return Some(if self.sd == SwitchPos::Up { 9 } else { 10 });
        }
        if self.swe != prev.swe {
            return Some(if self.swe == SwitchPos::Up { 11 } else { 12 });
        }
        if self.swf != prev.swf {
            return Some(if self.swf == SwitchPos::Up { 13 } else { 14 });
        }
        if self.sg != prev.sg {
            return Some(if self.sg == SwitchPos::Up { 15 } else { 16 });
        }
        if self.sh != prev.sh {
            return Some(if self.sh == SwitchPos::Up { 17 } else { 18 });
        }
        if self.swi != prev.swi {
            return Some(if self.swi == SwitchPos::Up { 19 } else { 20 });
        }
        if self.swj != prev.swj {
            return Some(if self.swj == SwitchPos::Up { 21 } else { 22 });
        }
        if self.swk != prev.swk {
            return Some(if self.swk == SwitchPos::Up { 23 } else { 24 });
        }
        if self.swl != prev.swl {
            return Some(if self.swl == SwitchPos::Up { 25 } else { 26 });
        }
        None
    }

    /// Detect if any switch toggled, returning the switch index 1..4 (1:SA, 2:SB, 3:SC, 4:SD).
    pub fn detect_dr_switch_change(&self, prev: &Switches) -> Option<u8> {
        if self.sa != prev.sa {
            Some(1)
        } else if self.sb != prev.sb {
            Some(2)
        } else if self.sc != prev.sc {
            Some(3)
        } else if self.sd != prev.sd {
            Some(4)
        } else {
            None
        }
    }
}

pub const DETENT_DEADBAND: u16 = 16;

/// Full processed input snapshot.
pub struct InputState {
    pub gimbals: Gimbals,
    pub pots: Pots,
    pub switches: Switches,
    pub battery_mv: u16,
    pub aux_pots: [i16; 10],
    #[allow(dead_code)]
    pub raw: [u16; adc::NUM_CHANNELS],
}

impl InputState {
    /// Convenience accessor for flight controls given a stick mode.
    #[inline(always)]
    pub fn flight_controls(&self, mode: crate::safety::StickMode) -> FlightControls {
        self.gimbals.to_flight_controls(mode)
    }

    /// Backwards compatibility accessor for physical gimbals.
    #[inline(always)]
    pub fn sticks(&self) -> &Gimbals {
        &self.gimbals
    }
}

/// Calibration data for a single analog axis.
#[derive(Copy, Clone, Debug)]
pub struct AxisCalib {
    pub min: u16,
    pub center: u16,
    pub max: u16,
    pub filtered_raw: u32,
    pub invert: bool,
}

impl AxisCalib {
    pub const fn new(min: u16, center: u16, max: u16, invert: bool) -> Self {
        Self {
            min,
            center,
            max,
            filtered_raw: 0,
            invert,
        }
    }

    /// Responsive low-latency jitter filter:
    /// Passes changes >= 6 counts with 0 latency; applies 4-sample MMA filter for resting micro-noise.
    pub fn filter_raw(&mut self, raw: u16) -> u16 {
        if self.filtered_raw == 0 {
            self.filtered_raw = raw as u32 * 4;
        }

        let previous = (self.filtered_raw / 4) as u16;
        let diff = (raw as i32 - previous as i32).abs();

        if diff < 6 {
            self.filtered_raw = (self.filtered_raw - previous as u32) + raw as u32;
        } else {
            self.filtered_raw = raw as u32 * 4;
        }
        (self.filtered_raw / 4) as u16
    }

    /// Normalize raw ADC count (0..4095) around center point to -1000..+1000.
    pub fn normalize(&mut self, raw: u16) -> i16 {
        let smoothed_raw = self.filter_raw(raw);

        let val = if smoothed_raw <= self.center {
            let span = (self.center - self.min).max(100) as i32;
            let delta = smoothed_raw as i32 - self.center as i32;
            ((delta * 1000) / span).clamp(-1000, 0)
        } else {
            let span = (self.max - self.center).max(100) as i32;
            let delta = smoothed_raw as i32 - self.center as i32;
            ((delta * 1000) / span).clamp(0, 1000)
        };

        if self.invert {
            -val as i16
        } else {
            val as i16
        }
    }

    /// Normalize a center-detented potentiometer with a deadband around the mechanical detent.
    /// Guarantees a solid 0 output when resting in the physical notch, while maintaining full
    /// independent linear travel (-1000 to +1000) even when negative and positive spans are asymmetric.
    pub fn normalize_detent(&mut self, raw: u16, deadband: u16) -> i16 {
        let smoothed_raw = self.filter_raw(raw);

        let val = if smoothed_raw.abs_diff(self.center) <= deadband {
            0
        } else if smoothed_raw < self.center {
            let active_center = self.center.saturating_sub(deadband);
            let span = active_center.saturating_sub(self.min).max(100) as i32;
            let delta = smoothed_raw as i32 - active_center as i32;
            ((delta * 1000) / span).clamp(-1000, 0)
        } else {
            let active_center = self.center.saturating_add(deadband);
            let span = self.max.saturating_sub(active_center).max(100) as i32;
            let delta = smoothed_raw as i32 - active_center as i32;
            ((delta * 1000) / span).clamp(0, 1000)
        };

        if self.invert {
            -val as i16
        } else {
            val as i16
        }
    }
}

use crate::adc::{ADC_CENTER, ADC_MAX, ADC_MIN};
use crate::storage::{AdcInputMode, RadioConfig};

#[derive(Copy, Clone, Debug)]
pub struct InputCalibration {
    pub rh: AxisCalib, // PA0: Right Horizontal
    pub rv: AxisCalib, // PA1: Right Vertical
    pub lv: AxisCalib, // PA2: Left Vertical
    pub lh: AxisCalib, // PA3: Left Horizontal
    pub aux: [AxisCalib; 10], // 0..3: SA..SD, 4..5: VRA..VRB, 6..9: VRC..VRF (PC1..PC4)
    pub adc_modes: [u8; 10],
    pub ext_adc_en: bool,
    pub filtered_battery_mv: u32,
}

impl InputCalibration {
    pub const fn default_factory() -> Self {
        Self {
            rh: AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, true),
            rv: AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, true),
            lv: AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false),
            lh: AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false),
            aux: [
                AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false), // SA (PA4)
                AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false), // SB (PA5)
                AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false), // SC (PB0)
                AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false), // SD (PB1)
                AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false), // VRA (PA6)
                AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false), // VRB (PA7)
                AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false), // VRC (PC1)
                AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false), // VRD (PC2)
                AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false), // VRE (PC3)
                AxisCalib::new(ADC_MIN, ADC_CENTER, ADC_MAX, false), // VRF (PC4)
            ],
            adc_modes: [0; 10],
            ext_adc_en: false,
            filtered_battery_mv: 0,
        }
    }
}

use core::cell::UnsafeCell;

struct InputManagerCell(UnsafeCell<InputCalibration>);
unsafe impl Sync for InputManagerCell {}

static INPUT_MANAGER: InputManagerCell = InputManagerCell(UnsafeCell::new(InputCalibration::default_factory()));

/// Apply a full set of stick and pot calibration endpoints.
pub fn apply_calibration(config: &RadioConfig) {
    let calib = unsafe { &mut *INPUT_MANAGER.0.get() };

    calib.ext_adc_en = config.ext_adc_en != 0;
    for i in 0..10 {
        calib.adc_modes[i] = config.get_adc_mode_val(i);
    }

    // Physical Gimbals: PA0 (RH), PA1 (RV), PA2 (LV), PA3 (LH)
    // RH: PA0
    calib.rh.invert = true;
    calib.rh.min = config.sticks[0].min;
    calib.rh.center = config.sticks[0].center;
    calib.rh.max = config.sticks[0].max;

    // RV: PA1
    calib.rv.invert = true;
    calib.rv.min = config.sticks[1].min;
    calib.rv.center = config.sticks[1].center;
    calib.rv.max = config.sticks[1].max;

    // LV: PA2
    calib.lv.invert = false;
    calib.lv.min = config.sticks[2].min;
    calib.lv.center = config.sticks[2].center;
    calib.lv.max = config.sticks[2].max;

    // LH: PA3
    calib.lh.invert = false;
    calib.lh.min = config.sticks[3].min;
    calib.lh.center = config.sticks[3].center;
    calib.lh.max = config.sticks[3].max;

    // 6 Auxiliary Analog Channels (SA, SB, SC, SD, VRA, VRB)
    for i in 0..6 {
        let mode = AdcInputMode::resolve(i, config.adc_modes[i]);
        calib.aux[i].invert = false;
        calib.aux[i].min = config.aux_pots[i].min;
        calib.aux[i].max = config.aux_pots[i].max;

        if mode == AdcInputMode::PotDetent {
            // Fixed at the captured physical detent center
            calib.aux[i].center = config.aux_pots[i].center;
        } else {
            // Standard continuous pot (or switch): clean calculated arithmetic midpoint
            calib.aux[i].center =
                ((config.aux_pots[i].min as u32 + config.aux_pots[i].max as u32) / 2) as u16;
        }
    }

    // 4 Extended Analog Channels (VRC..VRF on PC1..PC4)
    for i in 6..10 {
        calib.aux[i].invert = false;
        calib.aux[i].min = ADC_MIN;
        calib.aux[i].center = ADC_CENTER;
        calib.aux[i].max = ADC_MAX;
    }
}

/// Map raw analog reading to a 6-position flight mode index (1..6).
pub fn decode_switch_6pos_num(raw: u16) -> u8 {
    if raw < 682 {
        1
    } else if raw < 1365 {
        2
    } else if raw < 2048 {
        3
    } else if raw < 2730 {
        4
    } else if raw < 3413 {
        5
    } else {
        6
    }
}

/// Decode a 3-position switch from raw ADC counts.
pub fn decode_switch_3pos(raw: u16) -> SwitchPos {
    if raw < 1365 {
        SwitchPos::Up
    } else if raw < 2730 {
        SwitchPos::Mid
    } else {
        SwitchPos::Down
    }
}

/// Decode a 2-position switch from raw ADC counts.
pub fn decode_switch_2pos(raw: u16) -> SwitchPos {
    if raw < 2048 {
        SwitchPos::Up
    } else {
        SwitchPos::Down
    }
}

/// Map an aux channel index (0..9) to raw ADC channel index on FS-i6S.
#[inline(always)]
pub fn aux_index_to_raw_adc(ch: usize) -> usize {
    match ch {
        0 => 4,  // SA: PA4
        1 => 5,  // SB: PA5
        2 => 8,  // SC: PB0 (swapped on i6S)
        3 => 9,  // SD: PB1
        4 => 6,  // VRA: PA6
        5 => 7,  // VRB: PA7 (swapped on i6S)
        6 => 11, // VRC / SWI: PC1 (ADC11)
        7 => 12, // VRD / SWJ: PC2 (ADC12)
        8 => 13, // VRE / SWK: PC3 (ADC13)
        9 => 14, // VRF / SWL: PC4 (ADC14)
        _ => 4,
    }
}

/// Initialize input subsystem, load Flash calibration, and measure resting center for spring-loaded gimbals.
pub fn init() {
    // Wait for continuous DMA scanner to complete its initial cycle
    adc::wait_first_conversion();

    // 1. Load persisted calibration from Flash
    let cfg = crate::storage::load_config();
    apply_calibration(&cfg);

    // 2. Average 16 scans over ~4ms for rock-solid zero reference
    let mut sum_pitch = 0u32;
    let mut sum_roll = 0u32;
    let mut sum_yaw = 0u32;
    const SAMPLES: u32 = 16;

    for _ in 0..SAMPLES {
        let raw = adc::read_raw();
        sum_roll += raw[0] as u32;  // PA0 = Roll (Aileron)
        sum_pitch += raw[1] as u32; // PA1 = Pitch (Elevator)
        sum_yaw += raw[3] as u32;   // PA3 = LH (Yaw)
        for _ in 0..3_000 {
            cortex_m::asm::nop();
        }
    }

    let avg_roll = (sum_roll / SAMPLES) as u16;
    let avg_pitch = (sum_pitch / SAMPLES) as u16;
    let avg_yaw = (sum_yaw / SAMPLES) as u16;

    let calib = unsafe { &mut *INPUT_MANAGER.0.get() };
    // Slightly refine spring-loaded resting center if within reasonable range (1500..2500)
    if (1500..=2500).contains(&avg_roll) {
        calib.rh.center = avg_roll;
    }
    if (1500..=2500).contains(&avg_pitch) {
        calib.rv.center = avg_pitch;
    }
    if (1500..=2500).contains(&avg_yaw) {
        calib.lh.center = avg_yaw;
    }
}

/// Calculate battery voltage in millivolts using the FlySky FS-i6S 10k/5.1k resistor divider:
/// Vbat = Vadc * ((10000 + 5100) / 5100) = Vadc * 2.9608
/// Vadc = (raw * 3300) / 4095
pub fn calculate_battery_mv(raw: u16) -> u16 {
    // (raw * 3300 * 29608) / (4095 * 10000) = (raw * 977064) / 409500
    let vbat_mv = (raw as u32 * 977064) / 409500;
    vbat_mv as u16
}

/// Poll the ADC and return complete, processed flight controls.
pub fn poll() -> InputState {
    let raw = adc::read_raw();
    let calib = unsafe { &mut *INPUT_MANAGER.0.get() };

    // Process physical gimbal axes: PA0 (RH), PA1 (RV), PA2 (LV), PA3 (LH)
    let gimbals = Gimbals {
        rh: calib.rh.normalize(raw[0]),
        rv: calib.rv.normalize(raw[1]),
        lv: calib.lv.normalize(raw[2]),
        lh: calib.lh.normalize(raw[3]),
    };

    // Process auxiliary analog channels (0..5 standard, 6..9 extended PC1..PC4)
    let max_aux = if calib.ext_adc_en { 10 } else { 6 };
    let mut aux_pots = [0i16; 10];
    for i in 0..max_aux {
        let raw_val = raw[aux_index_to_raw_adc(i)];
        let mode = AdcInputMode::resolve(i, calib.adc_modes[i]);

        aux_pots[i] = match mode {
            AdcInputMode::PotDetent => calib.aux[i].normalize_detent(raw_val, DETENT_DEADBAND),
            AdcInputMode::Pot => calib.aux[i].normalize(raw_val),
            AdcInputMode::TwoPos | AdcInputMode::InstantTrim => {
                if raw_val < 2048 {
                    -1000
                } else {
                    1000
                }
            }
            AdcInputMode::ThreePos => {
                if raw_val < 1365 {
                    -1000
                } else if raw_val < 2730 {
                    0
                } else {
                    1000
                }
            }
            AdcInputMode::SixPos => {
                let step = decode_switch_6pos_num(raw_val);
                -1000 + ((step as i16 - 1) * 400)
            }
            AdcInputMode::Default => calib.aux[i].normalize(raw_val),
        };
    }

    let pots = Pots {
        vr1: aux_pots[4], // PA6 (VRA)
        vr2: aux_pots[5], // PA7 (VRB)
    };

    // Switches dynamic decoding according to configured AdcInputMode:
    let sa = match AdcInputMode::resolve(0, calib.adc_modes[0]) {
        AdcInputMode::ThreePos => decode_switch_3pos(raw[4]),
        _ => decode_switch_2pos(raw[4]),
    };
    let sb = match AdcInputMode::resolve(1, calib.adc_modes[1]) {
        AdcInputMode::TwoPos => decode_switch_2pos(raw[5]),
        _ => decode_switch_3pos(raw[5]),
    };
    let sc = match AdcInputMode::resolve(2, calib.adc_modes[2]) {
        AdcInputMode::TwoPos => decode_switch_2pos(raw[8]),
        _ => decode_switch_3pos(raw[8]),
    };
    let sd = match AdcInputMode::resolve(3, calib.adc_modes[3]) {
        AdcInputMode::ThreePos => decode_switch_3pos(raw[9]),
        _ => decode_switch_2pos(raw[9]),
    };

    let (swi, swj, swk, swl) = if calib.ext_adc_en {
        let swi = match AdcInputMode::resolve(6, calib.adc_modes[6]) {
            AdcInputMode::ThreePos => decode_switch_3pos(raw[11]),
            AdcInputMode::TwoPos => decode_switch_2pos(raw[11]),
            _ => SwitchPos::Up,
        };
        let swj = match AdcInputMode::resolve(7, calib.adc_modes[7]) {
            AdcInputMode::ThreePos => decode_switch_3pos(raw[12]),
            AdcInputMode::TwoPos => decode_switch_2pos(raw[12]),
            _ => SwitchPos::Up,
        };
        let swk = match AdcInputMode::resolve(8, calib.adc_modes[8]) {
            AdcInputMode::ThreePos => decode_switch_3pos(raw[13]),
            AdcInputMode::TwoPos => decode_switch_2pos(raw[13]),
            _ => SwitchPos::Up,
        };
        let swl = match AdcInputMode::resolve(9, calib.adc_modes[9]) {
            AdcInputMode::ThreePos => decode_switch_3pos(raw[14]),
            AdcInputMode::TwoPos => decode_switch_2pos(raw[14]),
            _ => SwitchPos::Up,
        };
        (swi, swj, swk, swl)
    } else {
        (SwitchPos::Up, SwitchPos::Up, SwitchPos::Up, SwitchPos::Up)
    };

    let switches = Switches {
        sa,
        sb,
        sc,
        sd,
        swe: SwitchPos::Up,
        swf: SwitchPos::Up,
        sg: SwitchPos::Up,
        sh: SwitchPos::Up,
        swi,
        swj,
        swk,
        swl,
    };

    let instant_mv = calculate_battery_mv(raw[10]); // PC0
    let battery_mv = if calib.filtered_battery_mv == 0 {
        calib.filtered_battery_mv = (instant_mv as u32) << 8;
        instant_mv
    } else {
        // Exponential moving average filter (alpha = 1/32) to stabilize hundredths digit
        calib.filtered_battery_mv = calib.filtered_battery_mv - (calib.filtered_battery_mv >> 5) + ((instant_mv as u32) << 3);
        (calib.filtered_battery_mv >> 8) as u16
    };

    InputState {
        gimbals,
        pots,
        switches,
        battery_mv,
        aux_pots,
        raw,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_battery_scaling_fs_i6s() {
        // 0 ADC -> 0 mV
        assert_eq!(calculate_battery_mv(0), 0);

        // Max 12-bit ADC (4095) -> 3.3V * 2.9608 = 9770 mV (9.77V)
        assert_eq!(calculate_battery_mv(4095), 9770);

        // Half scale (2048) -> ~4.88V
        let half = calculate_battery_mv(2048);
        assert!(half >= 4880 && half <= 4895, "Actual half-scale: {}", half);

        // Typical 4x AA nominal (5.0V): Vadc = 5.0 / 2.9608 = 1.6888V -> ADC = 1.6888 / 3.3 * 4095 = 2096
        let v5 = calculate_battery_mv(2096);
        assert!(v5 >= 4990 && v5 <= 5010, "Actual 5V scale: {}", v5);
    }

    #[test]
    fn test_detent_normalization_and_deadband() {
        let mut axis = AxisCalib::new(1000, 2048, 3000, false);

        // Within deadband around center (2048 +/- 16) must be exactly 0
        assert_eq!(axis.normalize_detent(2048, DETENT_DEADBAND), 0);
        assert_eq!(axis.normalize_detent(2048 - 15, DETENT_DEADBAND), 0);
        assert_eq!(axis.normalize_detent(2048 + 15, DETENT_DEADBAND), 0);

        // Outside deadband moves smoothly toward -1000 and +1000
        let neg = axis.normalize_detent(1000, DETENT_DEADBAND);
        assert_eq!(neg, -1000);

        let pos = axis.normalize_detent(3000, DETENT_DEADBAND);
        assert_eq!(pos, 1000);
    }

    #[test]
    fn test_decode_switch_6pos_steps() {
        assert_eq!(decode_switch_6pos_num(100), 1);
        assert_eq!(decode_switch_6pos_num(1000), 2);
        assert_eq!(decode_switch_6pos_num(1700), 3);
        assert_eq!(decode_switch_6pos_num(2400), 4);
        assert_eq!(decode_switch_6pos_num(3100), 5);
        assert_eq!(decode_switch_6pos_num(3800), 6);
    }

    #[test]
    fn test_aux_index_to_raw_adc_mapping() {
        assert_eq!(aux_index_to_raw_adc(0), 4);  // SA: PA4
        assert_eq!(aux_index_to_raw_adc(1), 5);  // SB: PA5
        assert_eq!(aux_index_to_raw_adc(2), 8);  // SC: PB0
        assert_eq!(aux_index_to_raw_adc(3), 9);  // SD: PB1
        assert_eq!(aux_index_to_raw_adc(4), 6);  // VRA: PA6
        assert_eq!(aux_index_to_raw_adc(5), 7);  // VRB: PA7
        assert_eq!(aux_index_to_raw_adc(6), 11); // VRC / SWI: PC1 (ADC11)
        assert_eq!(aux_index_to_raw_adc(7), 12); // VRD / SWJ: PC2 (ADC12)
        assert_eq!(aux_index_to_raw_adc(8), 13); // VRE / SWK: PC3 (ADC13)
        assert_eq!(aux_index_to_raw_adc(9), 14); // VRF / SWL: PC4 (ADC14)
    }

    #[test]
    fn test_swl_switch_condition_change() {
        let mut prev = Switches::new();
        let mut curr = Switches::new();

        // Initially both Up
        assert_eq!(curr.detect_condition_change(&prev), None);

        // SWL toggled to Down -> condition 26
        curr.swl = SwitchPos::Down;
        assert_eq!(curr.detect_condition_change(&prev), Some(26));

        // SWL toggled back to Up -> condition 25
        prev.swl = SwitchPos::Down;
        curr.swl = SwitchPos::Up;
        assert_eq!(curr.detect_condition_change(&prev), Some(25));
    }

    #[test]
    fn test_extended_adc_poll() {
        let mut cfg = RadioConfig::default_factory();
        cfg.ext_adc_en = 1;
        // ch 6 (VRC / PC1): 2-pos switch (mode 1)
        // ch 7 (VRD / PC2): 3-pos switch (mode 2)
        // ch 8 (VRE / PC3): 6-pos switch (mode 3)
        // ch 9 (VRF / PC4): pot (mode 4)
        cfg.set_adc_mode_val(6, 1);
        cfg.set_adc_mode_val(7, 2);
        cfg.set_adc_mode_val(8, 3);
        cfg.set_adc_mode_val(9, 4);

        apply_calibration(&cfg);

        // Mock ADC readings
        let mut mock_adc = [2048u16; crate::adc::NUM_CHANNELS];
        mock_adc[11] = 3500; // PC1 (ch6): high count -> 2-pos switch Down
        mock_adc[12] = 2048; // PC2 (ch7): center count -> 3-pos switch Mid
        mock_adc[13] = 2500; // PC3 (ch8): count 2500 -> 6-pos switch step 4
        mock_adc[14] = 4095; // PC4 (ch9): max count -> Pot +1000

        crate::adc::set_test_adc_snapshot(mock_adc);

        let state = poll();
        assert_eq!(state.switches.swi, SwitchPos::Down);
        assert_eq!(state.switches.swj, SwitchPos::Mid);
        // aux_pots[8] for 6-pos step 4 = -1000 + (3 * 400) = 200
        assert_eq!(state.aux_pots[8], 200);
        // aux_pots[9] for pot at 4095 = 1000
        assert_eq!(state.aux_pots[9], 1000);
    }
}
