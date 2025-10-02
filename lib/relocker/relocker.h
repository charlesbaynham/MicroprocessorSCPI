#include <OSFS.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>

#include <GeneralAIO.h>

#include <Arduino.h>
#include <CommandHandler.h>

struct LockSettings
{
  float v_max; // max voltage to apply
  float v_min;
  uint16_t n_steps; // number of steps in scan
  float window_frac;
  float min_diff;
  float v_low_threshold;
  float v_rise_threshold;
  bool auto_relock;
  uint32_t wait_time;
};

struct LockResult
{
  uint16_t i_biggest_diff;
  uint16_t i_lock;
  uint16_t i_rise;
  uint16_t v_read_lock;
  uint16_t v_set_lock;
  bool found_rise;
  bool found_biggest_diff;
  bool success;
  uint16_t read_voltages[100];
};

class Relocker
{
public:
  const uint8_t PRINT_DIGITS = 6;
  const uint8_t DAC_CH;
  const uint8_t ADC_CH;

  LockSettings settings;
  OSFS::result r;
  char file_name[5];
  uint16_t auto_relock_attempt_count = 0;
  uint16_t auto_relock_success_count = 0;
  LockResult lock_result = {0, 0, 0, 0, 0, false, false, false, {0}};

private:
  DAC8564 &dac;
  ADS8684 &adc;
  uint32_t v_min_mu;
  uint32_t v_max_mu;
  uint32_t v_step_mu;
  uint16_t low_threshold_mu;
  uint16_t rise_threshold_mu;
  int16_t min_diff_mu;

  // Relocker(const Relocker &obj) = delete;
public:
  Relocker(uint8_t dac_ch, uint8_t adc_ch)
      : DAC_CH(dac_ch), ADC_CH(adc_ch),
        dac(getDAC()),
        adc(getADC()) {

        };

  void init()
  {
    itoa(DAC_CH, file_name, 10);
    r = OSFS::getFile(file_name, settings);
    if (r == OSFS::result::UNFORMATTED)
    {
      OSFS::format();
      loadDefaultSettings();
    };
    if (r == OSFS::result::FILE_NOT_FOUND)
    {
      loadDefaultSettings();
    }
    settings.auto_relock = false;
    recalculateMachineUnits();
  };

  void loadDefaultSettings()
  {
    settings = {1, -1, 100, 0.5, 0.3, 1.1, 0.03, false, 1000};
    OSFS::newFile(file_name, settings, true);
  }

  inline float getSetV(uint16_t i, float v_max, float v_step)
  {
    return v_max - (i * v_step);
  };

  bool run()
  {
    if (settings.auto_relock)
    {
      autoRelock();
    };
  }

  bool autoRelock()
  {
    bool above_threshold = checkAuto();
    if (above_threshold)
    {
      relock();
      auto_relock_attempt_count++;
      if (lock_result.success)
      {
        auto_relock_success_count++;
      }
      delay(100);
    };
    return above_threshold;
  }

  bool checkAuto()
  {
    uint16_t reading = adc.setAndReadADCMu(ADC_CH);
    bool high_check = reading > low_threshold_mu;
    bool low_check = reading < low_threshold_mu - 2 * rise_threshold_mu;
    return high_check || low_check;
  }

  bool find_window_params(uint32_t &output_v_lock_mu,
                          size_t &output_i_lock,
                          size_t i_biggest_diff,
                          uint16_t *read_voltages,
                          uint16_t v_low,
                          uint16_t rise_threshold_mu,
                          float window_frac)
  {
    // find the slow rise window edge. don't need to look near
    // the sharp rise
    size_t i_slow_rise = 0;
    bool found_slow_rise = false;

    for (size_t i = i_biggest_diff - 3; i > 0; i--)
    {
      // look for where the voltage rise above a threshold
      if (read_voltages[i] > v_low + rise_threshold_mu)
      {
        found_slow_rise = true;
        i_slow_rise = i;

        // once we've passed the threshold we can stop looking
        break;
      }
    }

    if (found_slow_rise)
    {
      // Calculate the index and voltage for the lock point
      output_i_lock = i_slow_rise + (i_biggest_diff - i_slow_rise) * window_frac;
      output_v_lock_mu = v_max_mu - i_slow_rise * v_step_mu;
      return true;
    }
    else
    {
      // We have to save something, so write zeros and return false to indicate
      // failure
      output_i_lock = 0;
      output_v_lock_mu = 0;
      return false;
    }
  }

  void relock()
  {
    uint16_t v_read;
    int32_t diff;
    resetResults();
    uint16_t i_biggest_diff = lock_result.i_biggest_diff;

    adc.setADCChannel(ADC_CH);

    // reset everything at the start of the loop
    int16_t biggest_diff = min_diff_mu;
    // Serial.print("min diff = ");
    // Serial.println(biggest_diff);
    uint16_t v_low = adc.MAX_BITS;

    dac.setDACChannelMu(DAC_CH, v_max_mu);
    delayMicroseconds(settings.wait_time);
    // delay(settings.wait_time / 1000);
    uint16_t v_act_mu = v_max_mu;

    // step the voltage and record adc value. we also look for the large
    // step in read voltage inside this loop
    for (uint32_t i = 0; i < settings.n_steps; i++)
    {
      dac.setDACChannelMu(DAC_CH,
                          v_act_mu); // start high and go down
      delayMicroseconds(
          20); // brief delay to make sure the controller/diode can keep up
      v_act_mu -= v_step_mu;
      v_read = adc.readCurrentADCChannelMu();
      lock_result.read_voltages[i] = v_read;
      v_low = (v_read < v_low)
                  ? v_read
                  : v_low; // keep track of the lowest value recorded

      if ((v_read > low_threshold_mu) && // only bother checking for the
                                         // step if above a threshold
          (i > 0))                       // don't look on the first step
      {
        // compare each value with previous
        diff = int16_t(v_read) - int16_t(lock_result.read_voltages[i - 1]);
        if (diff > biggest_diff)
        {
          biggest_diff = diff; // keep track of the largest voltage
                               // difference seen between steps
          i_biggest_diff = i - 1;
          lock_result.found_biggest_diff = true;
        }
      }
    }

    uint32_t v_new_lockpoint_mu;
    size_t i_new_lockpoint;
    bool lockpoint_found = lock_result.found_biggest_diff &&
                           find_window_params(
                               v_new_lockpoint_mu,
                               i_new_lockpoint,
                               i_biggest_diff,
                               lock_result.read_voltages,
                               v_low,
                               rise_threshold_mu,
                               settings.window_frac);

    // Save lock results, even if it failed
    lock_result.v_set_lock = v_new_lockpoint_mu;
    lock_result.i_lock = i_new_lockpoint;
    lock_result.found_rise = true;
    lock_result.i_rise = i;
    lock_result.i_biggest_diff = i_biggest_diff;

    dac.setDACChannelMu(DAC_CH, v_max_mu);
    delayMicroseconds(settings.wait_time);
    if ((lock_result.v_set_lock < v_max_mu) &&
        (lock_result.v_set_lock > v_min_mu))
    {
      dac.setDACChannelMu(DAC_CH, lock_result.v_set_lock);
    }
    else
    {
      dac.setDACChannelMu(DAC_CH, 0);
    }
    delayMicroseconds(settings.wait_time);
    lock_result.v_read_lock = adc.readCurrentADCChannelMu();
    lock_result.success =
        (lock_result.v_read_lock < v_low + rise_threshold_mu);

    if (lock_result.success && lock_result.found_rise &&
        lock_result.found_biggest_diff && (lock_result.v_read_lock < low_threshold_mu + 2 * rise_threshold_mu))
    {
      settings.v_low_threshold = adc.muToVolts(v_low + rise_threshold_mu);
      low_threshold_mu = adc.voltsToMu(settings.v_low_threshold);
    }
  }

  void recalculateMachineUnits()
  {
    v_min_mu = dac.voltsToMu(settings.v_min);
    v_max_mu = dac.voltsToMu(settings.v_max);
    v_step_mu = (v_max_mu - v_min_mu) / settings.n_steps;
    low_threshold_mu = adc.voltsToMu(settings.v_low_threshold);
    rise_threshold_mu = adc.voltsDiffToMu(settings.v_rise_threshold);
    min_diff_mu = int16_t(adc.voltsDiffToMu(settings.min_diff));
  }

  void printSettings()
  {
    Serial.print(settings.v_max, PRINT_DIGITS);
    Serial.print(',');
    Serial.print(settings.v_min, PRINT_DIGITS);
    Serial.print(',');
    Serial.print(settings.n_steps);
    Serial.print(',');
    Serial.print(settings.window_frac, PRINT_DIGITS);
    Serial.print(',');
    Serial.print(settings.min_diff, PRINT_DIGITS);
    Serial.print(',');
    Serial.print(settings.v_low_threshold, PRINT_DIGITS);
    Serial.print(',');
    Serial.print(settings.v_rise_threshold, PRINT_DIGITS);
    Serial.print(',');
    Serial.print(settings.wait_time);
    Serial.print(',');
    Serial.print(settings.auto_relock);
  }

  void printResult()
  {
    Serial.print(lock_result.i_biggest_diff);
    Serial.print(',');
    Serial.print(lock_result.i_rise);
    Serial.print(',');
    Serial.print(lock_result.i_lock);
    Serial.print(',');
    Serial.print(adc.muToVolts(lock_result.v_read_lock), PRINT_DIGITS);
    Serial.print(',');
    Serial.print(dac.muToVolts(lock_result.v_set_lock), PRINT_DIGITS);
    Serial.print(',');
    Serial.print(lock_result.found_rise);
    Serial.print(',');
    Serial.print(lock_result.found_biggest_diff);
    Serial.print(',');
    Serial.print(lock_result.success);
  }

  void resetResults()
  {
    lock_result.i_biggest_diff = settings.n_steps - 1;
    lock_result.i_rise = 0;
    lock_result.found_biggest_diff = false;
    lock_result.found_rise = false;
    lock_result.success = false;
  }

  void printReadVoltages()
  {
    Serial.print(adc.muToVolts(lock_result.read_voltages[0]));
    for (uint8_t i = 1; i < settings.n_steps; i++)
    {
      Serial.print(',');
      Serial.print(adc.muToVolts(lock_result.read_voltages[i]), PRINT_DIGITS);
    }
  }

  void printAutoRelockStats()
  {
    Serial.print(auto_relock_attempt_count);
    Serial.print(',');
    Serial.print(auto_relock_success_count);
    auto_relock_attempt_count = 0;
    auto_relock_success_count = 0;
  }

  int readSerial()
  {
    // Serial.print("checking for state... ");
    if (Serial.available() > 0)
    {
      return Serial.parseInt();
    }
    else
    {
      return 0;
    }
  }

  void updateSettings()
  {
    OSFS::newFile(file_name, settings, true);
    recalculateMachineUnits();
  }

  OSFS::result clearSettings() { return OSFS::deleteFile(file_name); }
};

// class Watcher {
//  public:
//   uint8_t adc_ch;
//   ADS8684 adc;
//   const uint16_t THRESHOLD = adc.voltsToMu(2.5);

//   Watcher(uint8_t adc_ch, ADS8684 adc) : adc_ch(adc_ch), adc(adc) {};

//   bool check() { return adc.setAndReadADCMu(adc_ch) > THRESHOLD; };
// };