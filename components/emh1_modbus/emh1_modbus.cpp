#include "emh1_modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome.h"

namespace esphome {
namespace emh1_modbus {

static const char *const TAG = "emh1_modbus";
static const uint32_t RESPONSE_TIMEOUT_MS = 2000;

void eMH1Modbus::setup() {
  if (this->flow_control_pin_ != nullptr) {
     this->flow_control_pin_->setup();
  }
}

// loop() receives incoming replies from ABL
void eMH1Modbus::loop() {
  const uint32_t now = millis();
  if (this->bus_is_busy_ && (now - this->last_request_time_ > RESPONSE_TIMEOUT_MS)) {
    if (this->last_device_polled_ != nullptr) {
      ESP_LOGW(TAG, "Timeout waiting for response from address 0x%02X", this->last_device_polled_->get_address());
      this->last_device_polled_->on_timeout();
    } else {
      ESP_LOGW(TAG, "Timeout waiting for response from unknown device");
    }
    this->bus_is_busy_ = false;
  }

  if (now - this->last_emh1_modbus_byte_ > 50) {
    this->rx_buffer_.clear();
    this->last_emh1_modbus_byte_ = now;
  }

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    if (this->parse_emh1_modbus_byte_(byte)) {
      this->last_emh1_modbus_byte_ = now;
    } else {
      this->rx_buffer_.clear();
    }
  }
}

// convert char[2] hexencoded ascii to a single uint8_t value
uint8_t ascii2uint8(const char* value) {
  char c1 = value[0];
  uint8_t highBits = (c1 > '9')?(c1-55):(c1-48);
  char c2 = value[1];
  uint8_t lowBits = (c2 > '9')?(c2-55):(c2-48);
  return (highBits << 4 | lowBits);
}

// convert char[4] hexencoded ascii to a single uint16_t value
uint16_t ascii2uint16(const char* value) {
  uint16_t res = 0;
  char c;
  uint16_t bits;
  for (uint8_t x=0; x<4; x++) {
    c = value[x];
    bits = (c > '9')?(c-55):(c-48);
    res = (res << 4 | bits);
  }
  return res;
}

// calculate LRC checksum over char[], result is single uint8_t value
uint8_t lrc(const char* value, uint8_t l) {
  uint8_t lrc_ = 0;
  for (int i = 0; i < l-1; i = i + 2) {
    lrc_ -= ascii2uint8(&value[i]);
  }
  return lrc_;
}

// parse_emh1_modbus_byte will read a byte from RS485
// if it's the last byte of a transmission, it will
// proceed to parse the buffer, trying to make sense
// of the received package
//
bool eMH1Modbus::parse_emh1_modbus_byte_(uint8_t byte) {
	size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  if (byte != 0x0A) // 0x0A == LF == End of transmission
	  return true;
	this->rx_buffer_.push_back('\0');
  char *frame = &this->rx_buffer_[0];

	// check LRC
	uint8_t lrc1 = ascii2uint8(&frame[at-3]);
  uint8_t lrc2 = lrc(&frame[1], at-4);
  this->bus_is_busy_ = false;
	if (lrc1 != lrc2) {
		ESP_LOGW(TAG, "LRC check failed, discarding transmission");
		return false;
	}

	// check contents of first byte
	switch (frame[0]) {
	  case ':':
   	  ESP_LOGD(TAG, "Ignore Master transmission: %s", frame);
		  return false;
		case '>':
    	ESP_LOGD(TAG, "Received client transmission: %s", frame);
			break;
		default:
      ESP_LOGW(TAG, "Unknown broadcast data: %s", frame);
		  return false;
	}

  // Check Device ID
	uint8_t device_id = ascii2uint8(&frame[1]);

	// Check Function Code
  uint8_t r = ascii2uint8(&frame[3]);
	uint16_t v;
	switch(r) {
	  case 0x03:
      // ESP_LOGD(TAG, "Response to read operation");
			r = ascii2uint8(&frame[5]);
	    ESP_LOGD(TAG, "Receiving %u bytes", r);
			if (r == this->last_sent_data_length_ * 2) {
				// ESP_LOGD(TAG, "Send data upwards");
        uint8_t data[100];
				for (uint8_t x = 0; x<r; x++) {
				  data[x] = ascii2uint8(&frame[7+x*2]);
				}
  			bool found = false;
  			for (auto *device : this->devices_) {
    		  if (device->address_ == device_id) {
            device->on_emh1_modbus_data(this->last_sent_destination_, this->last_sent_data_length_, data);
						found = true;
            break;
      		}
    		}
  			if (!found) {
    		  ESP_LOGW(TAG, "Got eMH1 frame from unknown device address 0x%02X", device_id);
  			}
			} else {
				ESP_LOGW(TAG, "Response data size mismatch, expected %u got %u bytes", this->last_sent_data_length_ * 2, r);
			}
			break;
		case 0x10:
      ESP_LOGD(TAG, "Response to write operation");
      // Read eMH1 starting address
	    v = ascii2uint16(&frame[5]);
	    ESP_LOGD(TAG, "Starting address: 0x%04X", v);
			break;
	  case 0x90:
      ESP_LOGW(TAG, "Error response");
			break;
		default:
      ESP_LOGW(TAG, "Unknown response type");
  }
	this->rx_buffer_.clear();
  ESP_LOGD(TAG, "Cleared buffer");

	return true;
}

void eMH1Modbus::dump_config() {
  ESP_LOGCONFIG(TAG, "eMH1Modbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  this->check_uart_settings(38400);
}

float eMH1Modbus::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void eMH1Modbus::update() {
  if (this->bus_is_busy_) {
    return;  // Wait for the current transaction to finish
  }
  if (this->devices_.empty()) {
    return;
  }

  // Poll the next device in the list
  this->last_device_polled_ = this->devices_[this->next_device_index_];
  this->last_device_polled_->do_update();

  // Move to the next device for the next poll cycle
  this->next_device_index_ = (this->next_device_index_ + 1) % this->devices_.size();
}

void eMH1Modbus::query_status_report(uint8_t address) {
  ESP_LOGD(TAG, "Query Status Report for 0x%02X", address);
	eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = address;
	tx_message->FunctionCode = 0x03;
	tx_message->Destination = 0x002E;
	tx_message->DataLength = 0x0005;
  this->send_message();
}

void eMH1Modbus::get_serial(uint8_t address) {
  ESP_LOGD(TAG, "Query Serial Number for 0x%02X", address);
	eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = address;
	tx_message->FunctionCode = 0x03;
	tx_message->Destination = 0x0050;
	tx_message->DataLength = 0x0008;
  this->send_message();
}

// set Max current
void eMH1Modbus::send_current(uint8_t address, uint8_t x) {
	eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = address;
	tx_message->FunctionCode = 0x10;		// write operation
	tx_message->Destination = 0x0014;		// Set Ic Max
	tx_message->DataLength = 0x0001;		// 1 16-bit register
	tx_message->WriteBytes = 0x02;			// quantity of value bytes
	uint16_t v = std::floor(16.67*x);
  ESP_LOGD(TAG, "Set Max Current for 0x%02X to %d Amps (0x%04X)", address, x, v);
	uint8_t v1 = 0 + (v >> 8);
	uint8_t v2 = 0 + (v & 0x00FF);
	tx_message->Data[0] = v1;
	tx_message->Data[1] = v2;
	this->send_message();
}

// send enable/disable
void eMH1Modbus::send_enable(uint8_t address, uint8_t x) {
	eMH1MessageT *tx_message = &this->emh1_tx_message;
  tx_message->DeviceId = address;
	tx_message->FunctionCode = 0x10;		// write operation
	tx_message->Destination = 0x0005;		// EV-Status
	tx_message->DataLength = 0x0001;		// 1 16-bit register
	tx_message->WriteBytes = 0x02;			// quantity of value bytes
	if (x == 1) {
		tx_message->Data[0] = 0xA1;
		tx_message->Data[1] = 0xA1;
    ESP_LOGD(TAG, "Enable charger 0x%02X", address);
	} else {
		tx_message->Data[0] = 0xE0;
		tx_message->Data[1] = 0xE0;
    ESP_LOGD(TAG, "Disable charger 0x%02X", address);
  }
	this->send_message();
}

// convert single uint8_t value to hex-encoded ascii, append to outStr
uint8_t eMH1Modbus::hexencode_ascii(uint8_t val, char* outStr, uint8_t offset) {
  uint8_t highBits = (val & 0xF0) >> 4;
  uint8_t lowBits = (val & 0x0F);
  outStr[offset] = (highBits > 0x09)?(highBits+55):(highBits+48);
  outStr[offset+1] = (lowBits > 0x09)?(lowBits+55):(lowBits+48);
	return offset+2;
}

// convert single uint16_t value to hex-encoded ascii, append to outStr
uint8_t eMH1Modbus::hexencode_ascii(uint16_t val, char* outStr, uint8_t offset) {
  uint8_t highBits = (val & 0xF000) >> 12;
  uint8_t lowBits = (val & 0x0F00) >> 8;
  outStr[offset] = (highBits > 0x09)?(highBits+55):(highBits+48);
  outStr[offset+1] = (lowBits > 0x09)?(lowBits+55):(lowBits+48);
  highBits = (val & 0x00F0) >> 4;
  lowBits = (val & 0x000F);
  outStr[offset+2] = (highBits > 0x09)?(highBits+55):(highBits+48);
  outStr[offset+3] = (lowBits > 0x09)?(lowBits+55):(lowBits+48);
	return offset+4;
}

// convert array of uint8_t to hex-encoded ascii, append to outStr
uint8_t eMH1Modbus::hexencode_ascii(uint8_t* val, char* outStr, uint8_t offset, uint8_t cnt) {
  for (uint8_t x=0; x<cnt; x++) { 
    uint8_t highBits = (val[x] & 0xF0) >> 4;
    uint8_t lowBits = (val[x] & 0x0F);
    outStr[2*x+offset] = (highBits > 0x09)?(highBits+55):(highBits+48);
    outStr[2*x+offset+1] = (lowBits > 0x09)?(lowBits+55):(lowBits+48);
  }
	return offset+cnt*2;
}

// Send a query or command to ABL eMH1 via RS485
void eMH1Modbus::send_message() {
  // Send Modbus query as ASCII text (modbus-ascii !)
	eMH1MessageT *tx_message = &this->emh1_tx_message;

  this->last_sent_destination_ = tx_message->Destination;
  this->last_sent_data_length_ = tx_message->DataLength;
  this->bus_is_busy_ = true;
  this->last_request_time_ = millis();

	char buffer[200];
	uint8_t size = 0;
	size = hexencode_ascii(tx_message->DeviceId, buffer, size);
	size = hexencode_ascii(tx_message->FunctionCode, buffer, size);
	size = hexencode_ascii(tx_message->Destination, buffer, size);
	size = hexencode_ascii(tx_message->DataLength, buffer, size);

	if (tx_message->FunctionCode == 0x03) {
		tx_message->LRC = lrc(buffer, size);
	  size = hexencode_ascii(tx_message->LRC, buffer, size);
	} else {
	  size = hexencode_ascii(tx_message->WriteBytes, buffer, size);
	  size = hexencode_ascii(tx_message->Data, buffer, size, tx_message->WriteBytes);
		tx_message->LRC = lrc(buffer, size);
	  size = hexencode_ascii(tx_message->LRC, buffer, size);
  }
	buffer[size] = '\0';
  ESP_LOGD(TAG, "TX -> :%s", buffer);
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);
	this->write(':');
  this->write_array((const uint8_t *)buffer, size);
	this->write(0x0D);
	this->write(0x0A);
  this->flush();
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
}

}  // namespace emh1_modbus
}  // namespace esphome
