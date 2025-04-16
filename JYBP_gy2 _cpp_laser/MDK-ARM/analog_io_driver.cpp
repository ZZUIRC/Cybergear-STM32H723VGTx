#include "analog_io_driver.h"

AnalogIoDriver::AnalogIoDriver()
    : can_(NULL), master_can_id_(0), target_can_id_(0), send_count_(0)
{
}

AnalogIoDriver::AnalogIoDriver(uint8_t master_can_id, uint8_t target_can_id)
    : can_(NULL),
      master_can_id_(master_can_id),
      target_can_id_(target_can_id),
      send_count_(0)
{
}

AnalogIoDriver::~AnalogIoDriver()
{
}

void AnalogIoDriver::init(AnalogIoCanInterface* can, uint16_t wait_response_time_usec)
{
  can_ = can;
  wait_response_time_usec_ = wait_response_time_usec;
}

bool AnalogIoDriver::read_analog_channels(uint8_t is_voltage)
{
  // Send request: ID=0x601, data all zeros
  uint8_t data[8] = {0x00};
  send_command(target_can_id_, 0x01, master_can_id_, 8, data);

  // Receive and process response
  return receive_analog_data(channel_status_, is_voltage);
}

void AnalogIoDriver::send_command(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data)
{
  uint32_t id = (cmd_id << 24) | (option << 8) | can_id;
  if (!can_->send_message(id, data, len, true))
  {
    return;
  }
  HAL_Delay(1);  // Short delay to ensure message is sent
  ++send_count_;
}

bool AnalogIoDriver::receive_analog_data(AnalogChannelStatus& status, uint8_t is_voltage)
{
  unsigned long id;
  uint8_t len;
  if (!can_->read_message(id, receive_buffer_, len))
  {
    return false;
  }

  // Verify CAN ID
  uint8_t receive_can_id = id & 0xFF;
  if (receive_can_id != master_can_id_)
  {
    return false;
  }

  uint8_t motor_can_id = (id & 0xFF00) >> 8;
  if (motor_can_id != target_can_id_)
  {
    return false;
  }

  // Process packet if ID matches expected response (0x0601)
  if ((id & 0xFFFF) == 0x0601)
  {
    process_analog_packet(receive_buffer_, len, is_voltage);
    return true;
  }

  return false;
}

void AnalogIoDriver::process_analog_packet(const uint8_t* data, unsigned long len, uint8_t is_voltage)
{
  if (len != 8)
  {
    return;
  }

  // Parse channel 6 (bytes 1-2) and channel 8 (bytes 5-6)
  uint16_t ch6_raw = (data[1] << 8) | data[2];
  uint16_t ch8_raw = (data[5] << 8) | data[6];

  // Convert to float (mV to V or μA to A)
  channel_status_.ch6_value = (float)ch6_raw / (is_voltage ? 1000.0f : 1000000.0f);
  channel_status_.ch8_value = (float)ch8_raw / (is_voltage ? 1000.0f : 1000000.0f);
  channel_status_.stamp_usec = HAL_GetTick();
}