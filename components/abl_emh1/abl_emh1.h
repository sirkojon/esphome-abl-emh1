#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/emh1_modbus/emh1_modbus.h"

namespace esphome {
namespace abl_emh1 {

static const uint8_t REDISCOVERY_THRESHOLD = 5;

class ABLeMH1: public PollingComponent, public emh1_modbus::eMH1ModbusDevice {
 public:
  void set_l1_current_sensor(sensor::Sensor *l1_current_sensor) { l1_current_sensor_ = l1_current_sensor; }
  void set_l2_current_sensor(sensor::Sensor *l2_current_sensor) { l2_current_sensor_ = l2_current_sensor; }
  void set_l3_current_sensor(sensor::Sensor *l3_current_sensor) { l3_current_sensor_ = l3_current_sensor; }
	void set_max_current_sensor(sensor::Sensor *max_current_sensor) { max_current_sensor_ = max_current_sensor; }
  void set_serial_number_sensor(sensor::Sensor *serial_number_sensor) { serial_number_sensor_ = serial_number_sensor; }
	void set_outlet_state_sensor(sensor::Sensor *outlet_state_sensor) { outlet_state_sensor_ = outlet_state_sensor; }
	void set_mode_name_text_sensor(text_sensor::TextSensor *mode_name_text_sensor) { mode_name_text_sensor_ = mode_name_text_sensor; }
	void set_errors_text_sensor(text_sensor::TextSensor *errors_text_sensor) { errors_text_sensor_ = errors_text_sensor; }
  uint8_t get_no_response_count() { return no_response_count_; }

  void update() override;
  void on_emh1_modbus_data(const uint8_t &function, const std::vector<uint8_t> &data) override;
  void dump_config() override;

 protected:
  sensor::Sensor *l1_current_sensor_;
  sensor::Sensor *l2_current_sensor_;
  sensor::Sensor *l3_current_sensor_;
  sensor::Sensor *max_current_sensor_;
  sensor::Sensor *serial_number_sensor_;
  sensor::Sensor *outlet_state_sensor_;

  text_sensor::TextSensor *mode_name_text_sensor_;
  text_sensor::TextSensor *errors_text_sensor_;
  uint8_t no_response_count_ = REDISCOVERY_THRESHOLD;

  void decode_device_info_(const std::vector<uint8_t> &data);
  void decode_status_report_(const std::vector<uint8_t> &data);
  void decode_config_settings_(const std::vector<uint8_t> &data);
  void publish_state_(sensor::Sensor *sensor, float value);
  void publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state);
  void publish_device_offline_();
  std::string error_bits_to_string_(uint32_t bitmask);
};

}  // namespace abl_emh1
}  // namespace esphome
