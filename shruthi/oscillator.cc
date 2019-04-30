// Copyright 2009 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "shruthi/oscillator.h"

namespace shruthi {

uint8_t user_wavetable[129 * 8 + 1];

#define UPDATE_PHASE \
  if (*sync_input_++) { \
    phase.integral = 0; \
    phase.fractional = 0; \
  } \
  phase = U24AddC(phase, phase_increment_int); \
  *sync_output_++ = phase.carry; \

// This variant has a larger register width, but yields faster code.
#define UPDATE_PHASE_MORE_REGISTERS \
  if (*sync_input++) { \
    phase.integral = 0; \
    phase.fractional = 0; \
  } \
  phase = U24AddC(phase, phase_increment_int); \
  *sync_output++ = phase.carry; \

#define BEGIN_SAMPLE_LOOP \
  uint24c_t phase; \
  uint24_t phase_increment_int; \
  phase_increment_int.integral = phase_increment_.integral; \
  phase_increment_int.fractional = phase_increment_.fractional; \
  phase.integral = phase_.integral; \
  phase.fractional = phase_.fractional; \
  uint8_t size = kAudioBlockSize; \
  uint8_t* sync_input = sync_input_; \
  uint8_t* sync_output = sync_output_; \
  while (size--) {

#define END_SAMPLE_LOOP \
  } \
  phase_.integral = phase.integral; \
  phase_.fractional = phase.fractional;

#define CALCULATE_DIVISION_FACTOR(divisor, result_quotient, result_quotient_shifts) \
  uint16_t div_table_index = divisor; \
  int8_t result_quotient_shifts = 0; \
  while (div_table_index > 255) { \
    div_table_index >>= 1; \
    --result_quotient_shifts; \
  } \
  while (div_table_index < 128) { \
    div_table_index <<= 1; \
    ++result_quotient_shifts; \
  } \
  div_table_index -= 128; \
  uint8_t result_quotient = ResourcesManager::Lookup<uint8_t, uint8_t>( \
    wav_res_division_table, div_table_index);
    
#define CALCULATE_BLEP_INDEX(increment, quotient, quotient_shifts, result_blep_index) \
uint16_t result_blep_index = increment; \
int8_t shifts = quotient_shifts; \
while (shifts < 0) { \
  result_blep_index >>= 1; \
  ++shifts; \
} \
while (shifts > 0) { \
  result_blep_index <<= 1; \
  --shifts; \
} \
result_blep_index = U16U8MulShift8(result_blep_index, quotient);

// ------- Silence (useful when processing external signals) -----------------
void Oscillator::RenderSilence(uint8_t* buffer) {
  uint8_t size = kAudioBlockSize;
  while (size--) {
    *buffer++ = 128;
  }
}

// ------- Interpolation between two waveforms from two wavetables -----------
// The position is determined by the note pitch, to prevent aliasing.
// NOTE: This is now only used to preserve the original bandlimited Saw
void Oscillator::RenderSimpleWavetable(uint8_t* buffer) {
  uint8_t balance_index = U8Swap4(note_ - 12);
  uint8_t gain_2 = balance_index & 0xf0;
  uint8_t gain_1 = ~gain_2;

  uint8_t wave_index = balance_index & 0xf;
  uint8_t base_resource_id = WAV_RES_BANDLIMITED_SAW_0;

  const prog_uint8_t* wave_1 = waveform_table[base_resource_id + wave_index];
  wave_index = U8AddClip(wave_index, 1, kNumZonesFullSampleRate);
  const prog_uint8_t* wave_2 = waveform_table[base_resource_id + wave_index];
  
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE_MORE_REGISTERS
    uint8_t sample = InterpolateTwoTables(
        wave_1, wave_2,
        phase.integral, gain_1, gain_2);
    // To produce pulse width-modulated variants, we shift a portion of the 
    // waveform within an increasingly large fraction of the period. 
    // Note that this is pure waveshapping - the phase information is not 
    // used to determine when/where to shift.
    //
    //     /|            /|        
    //    / |           / |        
    //   /  |    =>  /|/  |    
    //  /   |       /     |/    
    // /    |/                  
    //
    if (sample < parameter_) {
      // Add a discontinuity.
      sample += parameter_ >> 1;
    }
    *buffer++ = sample;
  END_SAMPLE_LOOP
}

// ------- Interpolation between two waveforms from two wavetables -----------
void Oscillator::RenderInterpolatedWavetable(uint8_t* buffer) {
  uint8_t index = shape_ >= WAVEFORM_WAVETABLE_9 ? 
      shape_ - WAVEFORM_WAVETABLE_9 + 8 :
      shape_ - WAVEFORM_WAVETABLE_1;
  
  // Which wavetable should we play?.
  const prog_uint8_t* wavetable_definition = 
      wav_res_wavetables + U8U8Mul(index, 18);
  // Get a 8:8 value with the wave index in the first byte, and the
  // balance amount in the second byte.
  uint8_t num_steps = ResourcesManager::Lookup<uint8_t, uint8_t>(
      wavetable_definition,
      0);
  uint16_t pointer = U8U8Mul(parameter_ << 1, num_steps);
  uint16_t wave_index_1 = ResourcesManager::Lookup<uint8_t, uint8_t>(
      wavetable_definition,
      1 + (pointer >> 8));
  uint16_t wave_index_2 = ResourcesManager::Lookup<uint8_t, uint8_t>(
      wavetable_definition,
      2 + (pointer >> 8));
  uint8_t gain = pointer & 0xff;
  const prog_uint8_t* wave_1 = wav_res_waves + U8U8Mul(
      wave_index_1,
      129);
  const prog_uint8_t* wave_2 = wav_res_waves + U8U8Mul(
      wave_index_2,
      129);
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE_MORE_REGISTERS
    *buffer++ = InterpolateTwoTables(
        wave_1,
        wave_2,
        phase.integral >> 1,
        ~gain,
        gain);
  END_SAMPLE_LOOP
}  
// The position is freely determined by the parameter
void Oscillator::RenderSweepingWavetableRam(uint8_t* buffer) {
  uint8_t balance_index = U8Swap4(parameter_);
  uint8_t wave_index = balance_index & 0xf;
  uint8_t gain_2 = balance_index & 0xf0;
  uint8_t gain_1 = ~gain_2;
  
  uint16_t offset = wave_index << 7;
  offset += wave_index;
  const uint8_t* wave_1 = user_wavetable + offset;
  const uint8_t* wave_2 = wave_1;
  if (offset < kUserWavetableSize - 129) {
    wave_2 += 129;
  }
  
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    *buffer++ = U8Mix(
        InterpolateSampleRam(wave_1, phase.integral >> 1),
        InterpolateSampleRam(wave_2, phase.integral >> 1),
        gain_1, gain_2);
  END_SAMPLE_LOOP
}

// ------- Casio CZ-like synthesis -------------------------------------------
void Oscillator::RenderCzSaw(uint8_t* buffer) {
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE_MORE_REGISTERS
    uint8_t phi = phase.integral >> 8;
    uint8_t clipped_phi = phase.integral < 0x2000 ? phase.integral >> 5 : 0xff;
    // Interpolation causes more aliasing here.
    *buffer++ = ReadSample(wav_res_sine,
        U8MixU16(phi, clipped_phi, parameter_ << 1));
  END_SAMPLE_LOOP
}

void Oscillator::RenderCzResoPulse(uint8_t* buffer) {
  uint16_t increment = phase_increment_.integral + (
      (phase_increment_.integral * uint32_t(parameter_)) >> 2);
  uint16_t phase_2 = data_.secondary_phase;
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    if (phase.carry) {
      phase_2 = (shape_ == WAVEFORM_CZ_RESO_PLS_LP) ? 0 : 32768;
    }
    phase_2 += increment;
    uint8_t carrier = InterpolateSample(wav_res_sine, phase_2);
    if (shape_ == WAVEFORM_CZ_RESO_PLS_PK) {
      carrier >>= 1;
      carrier += 128;
    }
    uint8_t window = 0;
    if (phase.integral < 0x4000) {
      window = 255;
    } else if (phase.integral < 0x8000) {
      window = ~(phase.integral - 0x4000) >> 6;
    }
    if (shape_ == WAVEFORM_CZ_RESO_PLS_HP) {
      *buffer++ = S8U8MulShift8(carrier + 128, window) + 128;
    }
    else { // WAVEFORM_CZ_RESO_PLS_LP or WAVEFORM_CZ_RESO_PLS_PK
      *buffer++ = U8U8MulShift8(carrier, window);
    }
  END_SAMPLE_LOOP
  data_.secondary_phase = phase_2;
}

void Oscillator::RenderCzReso(uint8_t* buffer) {
  uint16_t increment = phase_increment_.integral + (
      (phase_increment_.integral * uint32_t(parameter_)) >> 3);
  uint16_t phase_2 = data_.secondary_phase;
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    if (phase.carry) {
      phase_2 = 0;
    }
    phase_2 += increment;
    uint8_t carrier = InterpolateSample(wav_res_sine, phase_2);
    if (shape_ == WAVEFORM_CZ_RESO_SYNC) {
      *buffer++ = phase.integral < 0x8000 ? carrier : 128;
    } else { // WAVEFORM_CZ_RESO_TRI
      uint8_t window = 0;
      window = (phase.integral & 0x8000) ?
            ~static_cast<uint8_t>(phase.integral >> 7) :
            phase.integral >> 7;
      *buffer++ = U8U8MulShift8(carrier, window);
    }
  END_SAMPLE_LOOP
  data_.secondary_phase = phase_2;
}

void Oscillator::RenderCzResoSaw(uint8_t* buffer) {
  uint16_t increment = phase_increment_.integral + (
      (phase_increment_.integral * uint32_t(parameter_)) >> 3);
  uint16_t phase_2 = data_.secondary_phase;
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    if (phase.carry) {
      phase_2 = (shape_ == WAVEFORM_CZ_RESO_SAW_LP) ? 0 : 32768;
    }
    phase_2 += increment;
    uint8_t carrier = InterpolateSample(wav_res_sine, phase_2);
    uint8_t window = ~(phase.integral >> 8);
    if (shape_ == WAVEFORM_CZ_RESO_SAW_HP) {
      *buffer++ = S8U8MulShift8(carrier + 128, window) + 128;
    } else { // WAVEFORM_CZ_RESO_SAW_LP or WAVEFORM_CZ_RESO_SAW_PK
      *buffer++ = U8U8MulShift8(carrier, window);
    }
  END_SAMPLE_LOOP
  data_.secondary_phase = phase_2;  
}

// ------- FM ----------------------------------------------------------------
void Oscillator::RenderFm(uint8_t* buffer) {
  uint8_t offset = secondary_parameter_;
  if (offset < 12) {
    offset = 0;
  } else if (offset > 36) {
    offset = 24;
  } else {
    offset = offset - 12;
  }
  uint16_t multiplier = ResourcesManager::Lookup<uint16_t, uint8_t>(
      lut_res_fm_frequency_ratios, offset);
  uint16_t increment = (
      static_cast<int32_t>(phase_increment_.integral) * multiplier) >> 8;
  parameter_ <<= 1;
  
  // The modulator will have a 16 bits phase increment, so we need to drop
  // the fractional part to 0 to ensure that the modulator frequency is
  // the correct multiple of the carrier frequency.
  phase_.fractional = 0;
  uint16_t phase_2 = data_.secondary_phase;
  uint8_t last_output = data_.output_sample;
  uint8_t fb_phase_mod = 
    (shape_ == WAVEFORM_FM || parameter_ < 128) ? 0 : parameter_ - 128;
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    phase_2 += increment;
    uint8_t modulator = InterpolateSample(wav_res_sine,
        phase_2 + fb_phase_mod*last_output);
    uint16_t modulation = modulator * parameter_;
    last_output = InterpolateSample(wav_res_sine,
        phase.integral + modulation);
    *buffer++ = last_output;
  END_SAMPLE_LOOP
  data_.secondary_phase = phase_2;
  data_.output_sample = last_output;
}

// ------- 8-bit land --------------------------------------------------------
void Oscillator::Render8BitLand(uint8_t* buffer) {
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    uint8_t x = parameter_;
    *buffer++ = (((phase.integral >> 8) ^ (x << 1)) & (~x)) + (x >> 1);
  END_SAMPLE_LOOP
}

// ------- 8-bit land --------------------------------------------------------
void Oscillator::RenderCrushedSine(uint8_t* buffer) {
  uint8_t decimate = data_.cr.decimate;
  uint8_t held_sample = data_.cr.state;
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE_MORE_REGISTERS
    ++decimate;
    if (parameter_ <= 63) {
      if (decimate >= parameter_ + 1) {
        decimate = 0;
        held_sample = InterpolateSample(wav_res_sine, phase.integral);
      }
    } else {
      if (decimate >= 128 - parameter_) {
        decimate = 0;
        uint8_t tri = phase.integral >> 7;
        held_sample = phase.integral & 0x8000 ? ~tri : tri;
      }
    }
    *buffer++ = held_sample;
  END_SAMPLE_LOOP
  data_.cr.decimate = decimate;
  data_.cr.state = held_sample;
}

void Oscillator::RenderVowel(uint8_t* buffer) {
  data_.vw.update = (data_.vw.update + 1) & 3;
  if (!data_.vw.update) {
    uint8_t offset_1 = U8ShiftRight4(parameter_);
    offset_1 = U8U8Mul(offset_1, 7);
    uint8_t offset_2 = offset_1 + 7;
    uint8_t balance = parameter_ & 15;
    
    // Interpolate formant frequencies.
    for (uint8_t i = 0; i < 3; ++i) {
      data_.vw.formant_increment[i] = U8U4MixU12(
          ResourcesManager::Lookup<uint8_t, uint8_t>(
              wav_res_vowel_data, offset_1 + i),
          ResourcesManager::Lookup<uint8_t, uint8_t>(
              wav_res_vowel_data, offset_2 + i),
          balance);
      data_.vw.formant_increment[i] <<= 3;
    }
    
    // Interpolate formant amplitudes.
    for (uint8_t i = 0; i < 4; ++i) {
      uint8_t amplitude_a = ResourcesManager::Lookup<uint8_t, uint8_t>(
          wav_res_vowel_data,
          offset_1 + 3 + i);
      uint8_t amplitude_b = ResourcesManager::Lookup<uint8_t, uint8_t>(
          wav_res_vowel_data,
          offset_2 + 3 + i);
      data_.vw.formant_amplitude[i] = U8U4MixU8(
          amplitude_a,
          amplitude_b, balance);
    }
  }
  BEGIN_SAMPLE_LOOP
    int8_t result = 0;
    for (uint8_t i = 0; i < 3; ++i) {
      data_.vw.formant_phase[i] += data_.vw.formant_increment[i];
      result += ResourcesManager::Lookup<uint8_t, uint8_t>(
          i == 2 ? wav_res_formant_square : wav_res_formant_sine,
          ((data_.vw.formant_phase[i] >> 8) & 0xf0) |
            data_.vw.formant_amplitude[i]);
    }
    result = S8U8MulShift8(result, ~(phase.integral >> 8));
    phase.integral += phase_increment_.integral;
    int16_t phase_noise = int8_t(Random::state_msb()) *
        int8_t(data_.vw.noise_modulation);
    if ((phase.integral + phase_noise) < phase_increment_.integral) {
      data_.vw.formant_phase[0] = 0;
      data_.vw.formant_phase[1] = 0;
      data_.vw.formant_phase[2] = 0;
    }
    uint8_t x = S16ClipS8(4 * result) + 128;
    *buffer++ = x;
    *buffer++ = x;
    size--;
  END_SAMPLE_LOOP
}

// ------- New Triangle (Non-band-limited and with different waveshaping) ----
void Oscillator::RenderNewTriangle(uint8_t* buffer) {
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    uint8_t tri = phase.integral >> 7;
    uint8_t v = phase.integral & 0x8000 ? tri : ~tri;
    if (v < parameter_) { // fold triangle
      v = (parameter_ << 1) - v;
    }
    *buffer++ = v;
  END_SAMPLE_LOOP
}

// ------- Dirty Pwm (kills kittens) -----------------------------------------
void Oscillator::RenderDirtyPwm(uint8_t* buffer) {
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    *buffer++ = (phase.integral >> 8) < 127 + parameter_ ? 0 : 255;
  END_SAMPLE_LOOP
}

// ------- Polyblep Saw ------------------------------------------------------
// Heavily inspired by Oliviers experimental implementation for STM but
// dumbed down and much less generic (does not do polyblep for sync etc)
void Oscillator::RenderPolyBlepSaw(uint8_t* buffer) {
  
  // calculate (1/increment) for later multiplication with current phase
  CALCULATE_DIVISION_FACTOR(phase_increment_.integral, quotient, quotient_shifts)

  // Revert to pure saw (=single blep) to avoid cpu overload for high notes
  uint8_t mod_parameter = note_ > 107 ? 0 : parameter_;
  uint8_t high = phase_.integral >= 0x8000;

  uint8_t next_sample = data_.output_sample;
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    uint8_t this_sample = next_sample;

    // Compute naive waveform
    next_sample = (phase.integral < 0x8000) ?
      (phase.integral >> 8) :
      (phase.integral >> 8) - mod_parameter;

    if (phase.carry) {
      high = false;
      CALCULATE_BLEP_INDEX(phase.integral, quotient, quotient_shifts, blep_index)
      this_sample -= U8U8MulShift8(
        ResourcesManager::Lookup<uint8_t, uint8_t>(wav_res_blep_table, blep_index),
        255-mod_parameter /* scale blep to size of edge */);
      next_sample += U8U8MulShift8(
        ResourcesManager::Lookup<uint8_t, uint8_t>(wav_res_blep_table, 127-blep_index),
        255-mod_parameter /* scale blep to size of edge */);
    }
    else if (mod_parameter && !high && phase.integral >= 0x8000) {
      high = true;
      CALCULATE_BLEP_INDEX(phase.integral-0x8000, quotient, quotient_shifts, blep_index)
      this_sample -= U8U8MulShift8(
        ResourcesManager::Lookup<uint8_t, uint8_t>(wav_res_blep_table, blep_index),
        mod_parameter /* scale blep to size of edge */);
      next_sample += U8U8MulShift8(
        ResourcesManager::Lookup<uint8_t, uint8_t>(wav_res_blep_table, 127-blep_index),
        mod_parameter /* scale blep to size of edge */);
    }

    *buffer++ = this_sample;
  END_SAMPLE_LOOP

  data_.output_sample = next_sample;
}

// ------- Polyblep CS-80 Saw ------------------------------------------------
// Heavily inspired by Oliviers experimental implementation for STM but
// dumbed down and much less generic (does not do polyblep for sync etc)
void Oscillator::RenderPolyBlepCSaw(uint8_t* buffer) {

  // calculate (1/increment) for later multiplication with current phase
  CALCULATE_DIVISION_FACTOR(phase_increment_.integral, quotient, quotient_shifts)

  // Revert to pure saw (=single blep) to avoid cpu overload for high notes
  uint8_t revert_to_saw = note_ > 107;

  // PWM modulation (constrained to extend over at least one increment)
  uint8_t pwm_limit = phase_increment_.integral >> 8;
  uint16_t pwm_phase =
    (parameter_ > 0 && parameter_ < pwm_limit) ?
    static_cast<uint16_t>(pwm_limit) << 8 :
    static_cast<uint16_t>(parameter_) << 8;
  uint8_t high = phase_.integral >= pwm_phase;

  uint8_t next_sample = data_.output_sample;
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    uint8_t this_sample = next_sample;

    // Compute naive waveform
    next_sample = (revert_to_saw || phase.integral >= pwm_phase) ?
      (phase.integral >> 8) : 0;

    if (phase.carry) {
      high = false;
      CALCULATE_BLEP_INDEX(phase.integral, quotient, quotient_shifts, blep_index)
      this_sample -= ResourcesManager::Lookup<uint8_t, uint8_t>(
        wav_res_blep_table, blep_index);
      next_sample += ResourcesManager::Lookup<uint8_t, uint8_t>(
        wav_res_blep_table, 127-blep_index);
    }
    else if (!revert_to_saw && /* no positive edge for pure saw */
      phase.integral >= pwm_phase && !high) {
      high = true;
      CALCULATE_BLEP_INDEX(phase.integral-pwm_phase, quotient, quotient_shifts, blep_index)
      this_sample += U8U8MulShift8(
        ResourcesManager::Lookup<uint8_t, uint8_t>(wav_res_blep_table, blep_index),
        parameter_ /* scale blep to size of edge */);
      next_sample -= U8U8MulShift8(
        ResourcesManager::Lookup<uint8_t, uint8_t>(wav_res_blep_table, 127-blep_index),
        parameter_ /* scale blep to size of edge */);
    }

    *buffer++ = this_sample;
  END_SAMPLE_LOOP

  data_.output_sample = next_sample;
}

// ------- Polyblep Pwm ------------------------------------------------------
// Heavily inspired by Oliviers experimental implementation for STM but
// dumbed down and much less generic (does not do polyblep for sync etc)
void Oscillator::RenderPolyBlepPwm(uint8_t* buffer) {

  // calculate (1/increment) for later multiplication with current phase
  CALCULATE_DIVISION_FACTOR(phase_increment_.integral, quotient, quotient_shifts)

  // Revert to pure saw (=single blep) to avoid cpu overload for high notes
  uint8_t revert_to_saw = note_ > 107;
     
  // PWM modulation (constrained to extend over at least one increment) 
  uint8_t pwm_limit = 127 - (phase_increment_.integral >> 8);
  uint16_t pwm_phase = 
    (parameter_ < pwm_limit) ? /* prevent dual bleps at same increment */
    static_cast<uint16_t>(127 + parameter_) << 8 :
    static_cast<uint16_t>(127 + pwm_limit) << 8;
  uint8_t high = phase_.integral >= pwm_phase;
  
  uint8_t next_sample = data_.output_sample;
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    uint8_t this_sample = next_sample;

    // Compute naive waveform
    next_sample = revert_to_saw ? 
      (phase.integral >> 8) : (phase.integral < pwm_phase ? 0 : 255);

    if (phase.carry) {
      high = false;
      CALCULATE_BLEP_INDEX(phase.integral, quotient, quotient_shifts, blep_index)
      this_sample -= ResourcesManager::Lookup<uint8_t, uint8_t>(
        wav_res_blep_table, blep_index);
      next_sample += ResourcesManager::Lookup<uint8_t, uint8_t>(
        wav_res_blep_table, 127-blep_index);
    }
    else if (!revert_to_saw && /* no positive edge for pure saw */
      phase.integral >= pwm_phase && !high) {
      high = true;
      CALCULATE_BLEP_INDEX(phase.integral-pwm_phase, quotient, quotient_shifts, blep_index)
      this_sample += ResourcesManager::Lookup<uint8_t, uint8_t>(
        wav_res_blep_table, blep_index);
      next_sample -= ResourcesManager::Lookup<uint8_t, uint8_t>(
        wav_res_blep_table, 127-blep_index);
    }

    *buffer++ = this_sample;
  END_SAMPLE_LOOP

  data_.output_sample = next_sample;
}

// ------- Quad saw or pwm (mit aliasing) ------------------------------------
void Oscillator::RenderQuad(uint8_t* buffer) {
  uint16_t phase_spread = (
      static_cast<uint32_t>(phase_increment_.integral) * parameter_) >> 13;
  ++phase_spread;
  uint16_t phase_increment = phase_increment_.integral;
  uint16_t increments[3];
  for (uint8_t i = 0; i < 3; ++i) {
    phase_increment += phase_spread;
    increments[i] = phase_increment;
  }
  
  if (shape_ == WAVEFORM_QUAD_SAW_PAD) {
    BEGIN_SAMPLE_LOOP
      UPDATE_PHASE
      data_.qs.phase[0] += increments[0];
      data_.qs.phase[1] += increments[1];
      data_.qs.phase[2] += increments[2];
      uint8_t value = (phase.integral >> 10);
      value += (data_.qs.phase[0] >> 10);
      value += (data_.qs.phase[1] >> 10);
      value += (data_.qs.phase[2] >> 10);
      *buffer++ = value;
    END_SAMPLE_LOOP
  }
  else { //WAVEFORM_QUAD_PWM
    uint16_t pwm_phase = static_cast<uint16_t>(127 + parameter_) << 8;
    BEGIN_SAMPLE_LOOP
      UPDATE_PHASE
      data_.qs.phase[0] += increments[0];
      data_.qs.phase[1] += increments[1];
      data_.qs.phase[2] += increments[2];
      uint8_t value = phase.integral < pwm_phase ? 0 : 63;
      value += data_.qs.phase[0] < pwm_phase ? 0 : 63;
      value += data_.qs.phase[1] < pwm_phase ? 0 : 63;
      value += data_.qs.phase[2] < pwm_phase ? 0 : 63;
      *buffer++ = value;
    END_SAMPLE_LOOP
  }
}

// ------- Low-passed, then high-passed white noise --------------------------
void Oscillator::RenderFilteredNoise(uint8_t* buffer) {
  uint16_t rng_state = data_.no.rng_state;
  if (rng_state == 0) {
    ++rng_state;
  }
  uint8_t filter_coefficient = parameter_ << 2;
  if (filter_coefficient <= 4) {
    filter_coefficient = 4;
  }
  BEGIN_SAMPLE_LOOP
    if (*sync_input_++) {
      rng_state = data_.no.rng_reset_value;
    }
    rng_state = (rng_state >> 1) ^ (-(rng_state & 1) & 0xb400);
    uint8_t noise_sample = rng_state >> 8;
    // This trick is used to avoid having a DC component (no innovation) when
    // the parameter is set to its minimal or maximal value.
    data_.no.lp_noise_sample = U8Mix(
        data_.no.lp_noise_sample,
        noise_sample,
        filter_coefficient);
    if (parameter_ >= 64) {
      *buffer++ = noise_sample - data_.no.lp_noise_sample - 128;
    } else {
      *buffer++ = data_.no.lp_noise_sample;
    }
  END_SAMPLE_LOOP
  data_.no.rng_state = rng_state;
}

// The position is freely determined by the parameter
void Oscillator::RenderWavequence(uint8_t* buffer) {
  const prog_uint8_t* wave = wav_res_waves + U8U8Mul(
      parameter_,
      129);
  BEGIN_SAMPLE_LOOP
    UPDATE_PHASE
    *buffer++ = InterpolateSample(wave, phase.integral >> 1);
  END_SAMPLE_LOOP
}

/* static */
const Oscillator::RenderFn Oscillator::fn_table_[] PROGMEM = {
  &Oscillator::RenderSilence,

  &Oscillator::RenderPolyBlepSaw,
  &Oscillator::RenderPolyBlepPwm,
  &Oscillator::RenderNewTriangle,

  &Oscillator::RenderCzSaw,
  &Oscillator::RenderCzResoSaw,
  &Oscillator::RenderCzReso,
  &Oscillator::RenderCzResoPulse,
  &Oscillator::RenderCzReso,

  &Oscillator::RenderQuad,

  &Oscillator::RenderFm,

  &Oscillator::RenderInterpolatedWavetable,
  &Oscillator::RenderInterpolatedWavetable,
  &Oscillator::RenderInterpolatedWavetable,
  &Oscillator::RenderInterpolatedWavetable,
  &Oscillator::RenderInterpolatedWavetable,
  &Oscillator::RenderInterpolatedWavetable,
  &Oscillator::RenderInterpolatedWavetable,
  &Oscillator::RenderInterpolatedWavetable,
  &Oscillator::RenderSweepingWavetableRam,

  &Oscillator::Render8BitLand,
  &Oscillator::RenderCrushedSine,
  &Oscillator::RenderDirtyPwm,
  &Oscillator::RenderFilteredNoise,

  &Oscillator::RenderVowel,
  
  &Oscillator::RenderInterpolatedWavetable,
  &Oscillator::RenderSimpleWavetable,
  &Oscillator::RenderQuad,
  &Oscillator::RenderFm,
  &Oscillator::RenderPolyBlepCSaw,
  &Oscillator::RenderCzResoSaw,
  &Oscillator::RenderCzResoSaw,
  &Oscillator::RenderCzResoPulse,
  &Oscillator::RenderCzResoPulse
};

}  // namespace shruthi
