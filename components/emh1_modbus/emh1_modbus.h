#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace emh1_modbus {

struct eMH1MessageT {
  uint8_t DeviceId;
	uint8_t FunctionCode;
	uint16_t Destination;
	uint16_t DataLength;
	uint8_t LRC;
	uint8_t WriteBytes;
	uint8_t Data[100];
};

class eMH1ModbusDevice;

class eMH1Modbus : public uart::UARTDevice, public PollingComponent {
 public:
  eMH1Modbus() = default;

  void setup() override;
  void loop() override;
  void update() override;

  void dump_config() override;

  void register_device(eMH1ModbusDevice *device) { this->devices_.push_back(device); }
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }

  float get_setup_priority() const override;

  void send_current(uint8_t address, uint8_t x);
  void send_enable(uint8_t address, uint8_t x);
  void query_status_report(uint8_t address);
  void get_serial(uint8_t address);
  uint8_t hexencode_ascii(uint8_t val, char* outStr, uint8_t offset);
	uint8_t hexencode_ascii(uint16_t val, char* outStr, uint8_t offset);
	uint8_t hexencode_ascii(uint8_t* val, char* outStr, uint8_t offset, uint8_t cnt);

 protected:
  void send_message();
  bool parse_emh1_modbus_byte_(uint8_t byte);
  GPIOPin *flow_control_pin_{nullptr};

  eMH1MessageT emh1_tx_message{};
  std::vector<char> rx_buffer_;
  uint32_t last_emh1_modbus_byte_{0};
  std::vector<eMH1ModbusDevice *> devices_;

  uint16_t last_sent_destination_{0};
  uint16_t last_sent_data_length_{0};
  size_t next_device_index_{0};
  bool bus_is_busy_{false};
  uint32_t last_request_time_{0};
  eMH1ModbusDevice *last_device_polled_{nullptr};
};

class eMH1ModbusDevice {
 public:
  void set_parent(eMH1Modbus *parent) { parent_ = parent; }
  void set_address(uint8_t address) { address_ = address; }
  virtual void on_emh1_modbus_data(uint16_t function, uint16_t datalength, const uint8_t *data) = 0;
  virtual void do_update() = 0;
  virtual void on_timeout() = 0;
  uint8_t get_address() { return address_; }
  
	void query_status_report() { this->parent_->query_status_report(this->address_); }
  void get_serial() { this->parent_->get_serial(this->address_); }

 protected:
  friend eMH1Modbus;

  eMH1Modbus *parent_;
  uint8_t address_;
};

}  // namespace emh1_modbus
}  // namespace esphome
