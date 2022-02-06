#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome
{
  namespace qmp6988
  {

    typedef struct _qmp6988_cali_data
    {
      long COE_a0;
      short COE_a1;
      short COE_a2;
      long COE_b00;
      short COE_bt1;
      short COE_bt2;
      short COE_bp1;
      short COE_b11;
      short COE_bp2;
      short COE_b12;
      short COE_b21;
      short COE_bp3;
    } qmp6988_cali_data_t;

    typedef struct _qmp6988_fk_data
    {
      float a0, b00;
      float a1, a2, bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
    } qmp6988_fk_data_t;

    typedef struct _qmp6988_ik_data
    {
      long a0, b00;
      long a1, a2;
      long long bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
    } qmp6988_ik_data_t;

    typedef struct _qmp6988_data
    {
      uint8_t slave;
      uint8_t chip_id;
      uint8_t power_mode;
      float temperature;
      float pressure;
      float altitude;
      qmp6988_cali_data_t qmp6988_cali;
      qmp6988_ik_data_t ik;
    } qmp6988_data_t;

    class QMP6988Component : public PollingComponent, public i2c::I2CDevice
    {
    public:
      void setup() override;
      void dump_config() override;
      float get_setup_priority() const override;
      void update() override;

      void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
      void set_pressure_sensor(sensor::Sensor *pressure_sensor) { pressure_sensor_ = pressure_sensor; }

    protected:
      qmp6988_data_t qmp6988_data;
      bool read_calib_data();
      bool device_check();
      bool software_reset();
      bool set_power_mode();
      bool set_oversampling();
      short convert_temperature(qmp6988_ik_data_t *ik, long dt);
      long convert_pressure(qmp6988_ik_data_t *ik, long dp, short tx);
      bool read_data_(uint8_t *data);

      sensor::Sensor *temperature_sensor_;
      sensor::Sensor *pressure_sensor_;
    };

  } // namespace qmp6988
} // namespace esphome