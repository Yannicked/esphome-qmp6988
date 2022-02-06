// Implementation based on:
//  - ESPEasy: https://github.com/letscontrolit/ESPEasy/blob/mega/src/_P034_DHT12.ino
//  - DHT12_sensor: https://github.com/esphome/esphome/blob/dev/esphome/components/dht12/dht12.cpp

#include "qmp6988.h"
#include "esphome/core/log.h"

namespace esphome
{
  namespace qmp6988
  {

    static const char *const TAG = "qmp6988";

    constexpr int QMP6988_CALIBRATION_DATA_START = 0xA0; /* QMP6988 compensation coefficients */
    constexpr int QMP6988_CALIBRATION_DATA_LENGTH = 25;
    constexpr int QMP6988_CHIP_ID = 0x5C;
    constexpr int QMP6988_CHIP_ID_REG = 0xD1;
    constexpr int QMP6988_RESET_REG = 0xE0;       /* Device reset register */
    constexpr int QMP6988_DEVICE_STAT_REG = 0xF3; /* Device state register */
    constexpr int QMP6988_CTRLMEAS_REG = 0xF4;    /* Measurement Condition Control Register */

    constexpr int QMP6988_PRESSURE_MSB_REG = 0xF7;    /* Pressure MSB Register */
    constexpr int QMP6988_TEMPERATURE_MSB_REG = 0xFA; /* Temperature MSB Reg */

    constexpr int QMP6988_CONFIG_REG = 0xF1; /*IIR filter co-efficient setting Register*/

    bool QMP6988Component::read_calib_data()
    {
      int status = 0;
      // BITFIELDS temp_COE;
      uint8_t a_data_uint8_tr[QMP6988_CALIBRATION_DATA_LENGTH] = {0};
      int len;

      for (len = 0; len < QMP6988_CALIBRATION_DATA_LENGTH; len += 1)
      {
        if (!read_byte(QMP6988_CALIBRATION_DATA_START + len, &a_data_uint8_tr[len]))
        {
          ESP_LOGE(TAG, "qmp6988 read 0xA0 error!");
          return false;
        }
      }

      qmp6988_data.qmp6988_cali.COE_a0 = (long)(((a_data_uint8_tr[18] << 12) | (a_data_uint8_tr[19] << 4) | (a_data_uint8_tr[24] & 0x0f)) << 12);
      qmp6988_data.qmp6988_cali.COE_a0 = qmp6988_data.qmp6988_cali.COE_a0 >> 12;

      qmp6988_data.qmp6988_cali.COE_a1 = (short)(((a_data_uint8_tr[20]) << 8) | a_data_uint8_tr[21]);
      qmp6988_data.qmp6988_cali.COE_a2 = (short)(((a_data_uint8_tr[22]) << 8) | a_data_uint8_tr[23]);

      qmp6988_data.qmp6988_cali.COE_b00 = (long)(((a_data_uint8_tr[0] << 12) | (a_data_uint8_tr[1] << 4) | ((a_data_uint8_tr[24] & 0xf0) >> 4)) << 12);
      qmp6988_data.qmp6988_cali.COE_b00 = qmp6988_data.qmp6988_cali.COE_b00 >> 12;

      qmp6988_data.qmp6988_cali.COE_bt1 = (short)(((a_data_uint8_tr[2]) << 8) | a_data_uint8_tr[3]);
      qmp6988_data.qmp6988_cali.COE_bt2 = (short)(((a_data_uint8_tr[4]) << 8) | a_data_uint8_tr[5]);
      qmp6988_data.qmp6988_cali.COE_bp1 = (short)(((a_data_uint8_tr[6]) << 8) | a_data_uint8_tr[7]);
      qmp6988_data.qmp6988_cali.COE_b11 = (short)(((a_data_uint8_tr[8]) << 8) | a_data_uint8_tr[9]);
      qmp6988_data.qmp6988_cali.COE_bp2 = (short)(((a_data_uint8_tr[10]) << 8) | a_data_uint8_tr[11]);
      qmp6988_data.qmp6988_cali.COE_b12 = (short)(((a_data_uint8_tr[12]) << 8) | a_data_uint8_tr[13]);
      qmp6988_data.qmp6988_cali.COE_b21 = (short)(((a_data_uint8_tr[14]) << 8) | a_data_uint8_tr[15]);
      qmp6988_data.qmp6988_cali.COE_bp3 = (short)(((a_data_uint8_tr[16]) << 8) | a_data_uint8_tr[17]);

      qmp6988_data.ik.a0 = qmp6988_data.qmp6988_cali.COE_a0;   // 20Q4
      qmp6988_data.ik.b00 = qmp6988_data.qmp6988_cali.COE_b00; // 20Q4

      qmp6988_data.ik.a1 = 3608L * (long)qmp6988_data.qmp6988_cali.COE_a1 - 1731677965L; // 31Q23
      qmp6988_data.ik.a2 = 16889L * (long)qmp6988_data.qmp6988_cali.COE_a2 - 87619360L;  // 30Q47

      qmp6988_data.ik.bt1 = 2982L * (long long)qmp6988_data.qmp6988_cali.COE_bt1 + 107370906L;   // 28Q15
      qmp6988_data.ik.bt2 = 329854L * (long long)qmp6988_data.qmp6988_cali.COE_bt2 + 108083093L; // 34Q38
      qmp6988_data.ik.bp1 = 19923L * (long long)qmp6988_data.qmp6988_cali.COE_bp1 + 1133836764L; // 31Q20
      qmp6988_data.ik.b11 = 2406L * (long long)qmp6988_data.qmp6988_cali.COE_b11 + 118215883L;   // 28Q34
      qmp6988_data.ik.bp2 = 3079L * (long long)qmp6988_data.qmp6988_cali.COE_bp2 - 181579595L;   // 29Q43
      qmp6988_data.ik.b12 = 6846L * (long long)qmp6988_data.qmp6988_cali.COE_b12 + 85590281L;    // 29Q53
      qmp6988_data.ik.b21 = 13836L * (long long)qmp6988_data.qmp6988_cali.COE_b21 + 79333336L;   // 29Q60
      qmp6988_data.ik.bp3 = 2915L * (long long)qmp6988_data.qmp6988_cali.COE_bp3 + 157155561L;   // 28Q65
      return true;
    }

    bool QMP6988Component::device_check()
    {
      if (!read_byte(QMP6988_CHIP_ID_REG, &(qmp6988_data.chip_id)))
      {
        ESP_LOGE(TAG, "Failed reading chip id");
        return false;
      }
      ESP_LOGD(TAG, "read chip id = 0x%x\r\n", qmp6988_data.chip_id);
      if (qmp6988_data.chip_id == QMP6988_CHIP_ID)
      {
        ESP_LOGE(TAG, "Chip id is not correct");
        return false;
      }

      return true;
    }

    bool QMP6988Component::software_reset()
    {
      if (!write_byte(QMP6988_RESET_REG, 0xe6))
      {
        ESP_LOGE(TAG, "Software reset fail");
        return false;
      }

      delay_microseconds_safe(20000);

      if (!write_byte(QMP6988_RESET_REG, 0x00))
      {
        ESP_LOGE(TAG, "Software reset fail");
        return false;
      }

      return true;
    }

    bool QMP6988Component::set_power_mode()
    {
      qmp6988_data.power_mode = 0x03;
      uint8_t config_data;
      if (!read_byte(QMP6988_CTRLMEAS_REG, &config_data))
      {
        ESP_LOGE(TAG, "Failed reading config from chip");
        return false;
      }

      config_data &= 0xfc;
      config_data |= 0x03;

      if (!write_byte(QMP6988_CTRLMEAS_REG, config_data))
      {
        ESP_LOGE(TAG, "Failed writing config to chip");
        return false;
      }

      delay_microseconds_safe(20000);
      return true;
    }

    bool QMP6988Component::set_oversampling()
    {
      uint8_t config_data;
      if (!read_byte(QMP6988_CTRLMEAS_REG, &config_data))
      {
        ESP_LOGE(TAG, "Failed reading config from chip");
        return false;
      }

      // Pressure oversampling 8x
      config_data &= 0xe3;
      config_data |= (0x04 << 2);

      // Temperature oversampling 1x
      config_data &= 0x1f;
      config_data |= (0x01 << 5);

      if (!write_byte(QMP6988_CTRLMEAS_REG, config_data))
      {
        ESP_LOGE(TAG, "Failed writing config to chip");
        return false;
      }

      delay_microseconds_safe(20000);
      return true;
    }

    short QMP6988Component::convert_temperature(qmp6988_ik_data_t *ik, long dt)
    {
      short ret;
      long long wk1, wk2;

      // wk1: 60Q4 // bit size
      wk1 = ((long long)ik->a1 * (long long)dt);       // 31Q23+24-1=54 (54Q23)
      wk2 = ((long long)ik->a2 * (long long)dt) >> 14; // 30Q47+24-1=53 (39Q33)
      wk2 = (wk2 * (long long)dt) >> 10;               // 39Q33+24-1=62 (52Q23)
      wk2 = ((wk1 + wk2) / 32767) >> 19;               // 54,52->55Q23 (20Q04)
      ret = (short)((ik->a0 + wk2) >> 4);              // 21Q4 -> 17Q0
      return ret;
    }
    long QMP6988Component::convert_pressure(qmp6988_ik_data_t *ik, long dp, short tx)
    {
      long ret;
      long long wk1, wk2, wk3;

      // wk1 = 48Q16 // bit size
      wk1 = ((long long)ik->bt1 * (long long)tx);       // 28Q15+16-1=43 (43Q15)
      wk2 = ((long long)ik->bp1 * (long long)dp) >> 5;  // 31Q20+24-1=54 (49Q15)
      wk1 += wk2;                                       // 43,49->50Q15
      wk2 = ((long long)ik->bt2 * (long long)tx) >> 1;  // 34Q38+16-1=49 (48Q37)
      wk2 = (wk2 * (long long)tx) >> 8;                 // 48Q37+16-1=63 (55Q29)
      wk3 = wk2;                                        // 55Q29
      wk2 = ((long long)ik->b11 * (long long)tx) >> 4;  // 28Q34+16-1=43 (39Q30)
      wk2 = (wk2 * (long long)dp) >> 1;                 // 39Q30+24-1=62 (61Q29)
      wk3 += wk2;                                       // 55,61->62Q29
      wk2 = ((long long)ik->bp2 * (long long)dp) >> 13; // 29Q43+24-1=52 (39Q30)
      wk2 = (wk2 * (long long)dp) >> 1;                 // 39Q30+24-1=62 (61Q29)
      wk3 += wk2;                                       // 62,61->63Q29
      wk1 += wk3 >> 14;                                 // Q29 >> 14 -> Q15
      wk2 = ((long long)ik->b12 * (long long)tx);       // 29Q53+16-1=45 (45Q53)
      wk2 = (wk2 * (long long)tx) >> 22;                // 45Q53+16-1=61 (39Q31)
      wk2 = (wk2 * (long long)dp) >> 1;                 // 39Q31+24-1=62 (61Q30)
      wk3 = wk2;                                        // 61Q30
      wk2 = ((long long)ik->b21 * (long long)tx) >> 6;  // 29Q60+16-1=45 (39Q54)
      wk2 = (wk2 * (long long)dp) >> 23;                // 39Q54+24-1=62 (39Q31)
      wk2 = (wk2 * (long long)dp) >> 1;                 // 39Q31+24-1=62 (61Q20)
      wk3 += wk2;                                       // 61,61->62Q30
      wk2 = ((long long)ik->bp3 * (long long)dp) >> 12; // 28Q65+24-1=51 (39Q53)
      wk2 = (wk2 * (long long)dp) >> 23;                // 39Q53+24-1=62 (39Q30)
      wk2 = (wk2 * (long long)dp);                      // 39Q30+24-1=62 (62Q30)
      wk3 += wk2;                                       // 62,62->63Q30
      wk1 += wk3 >> 15;                                 // Q30 >> 15 = Q15
      wk1 /= 32767L;
      wk1 >>= 11;     // Q15 >> 7 = Q4
      wk1 += ik->b00; // Q4 + 20Q4
      // wk1 >>= 4; // 28Q4 -> 24Q0
      ret = (long)wk1;
      return ret;
    }

    void QMP6988Component::update()
    {
      uint8_t err = 0;
      unsigned long P_read, T_read;
      long P_raw, T_raw;
      uint8_t a_data_uint8_tr[6] = {0};
      long T_int, P_int;

      // press
      if (!read_bytes(QMP6988_PRESSURE_MSB_REG, a_data_uint8_tr, 6))
      {
        status_set_warning();
        return;
      }
      P_read = (unsigned long)((((unsigned long)(a_data_uint8_tr[0])) << 16) |
                               (((unsigned short)(a_data_uint8_tr[1])) << 8) | (a_data_uint8_tr[2]));
      P_raw = (long)(P_read - 8388608);

      T_read = (unsigned long)((((unsigned long)(a_data_uint8_tr[3])) << 16) |
                               (((unsigned short)(a_data_uint8_tr[4])) << 8) | (a_data_uint8_tr[5]));
      T_raw = (long)(T_read - 8388608);

      T_int = convert_temperature(&(qmp6988_data.ik), T_raw);
      P_int = convert_pressure(&(qmp6988_data.ik), P_raw, T_int);
      qmp6988_data.temperature = (float)T_int / 256.0f;
      qmp6988_data.pressure = (float)P_int / 16.0f;

      ESP_LOGD(TAG, "Got temperature=%.2f°C pressure=%.2f%%", qmp6988_data.temperature, qmp6988_data.pressure);

      if (temperature_sensor_ != nullptr)
        temperature_sensor_->publish_state(qmp6988_data.temperature);
      if (pressure_sensor_ != nullptr)
        pressure_sensor_->publish_state(qmp6988_data.pressure);
      status_clear_warning();
    }
    void QMP6988Component::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up DHT12...");
      if (!device_check())
      {
        mark_failed();
      }
      if (!software_reset())
      {
        mark_failed();
      }
      if (!read_calib_data())
      {
        mark_failed();
      }
      if (!set_power_mode())
      {
        mark_failed();
      }
      if (!write_byte(QMP6988_CONFIG_REG, 0x02 & 0x03))
      {
        ESP_LOGE(TAG, "Failed setting filtering mode");
        mark_failed();
      }
      if (!set_oversampling())
      {
        mark_failed();
      }
    }
    void QMP6988Component::dump_config()
    {
      ESP_LOGD(TAG, "QMP6988:");
      LOG_I2C_DEVICE(this);
      if (this->is_failed())
      {
        ESP_LOGE(TAG, "Communication with QMP6988 failed!");
      }
      LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
      LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
    }
    float QMP6988Component::get_setup_priority() const { return setup_priority::DATA; }

  } // namespace qmp6988
} // namespace esphome