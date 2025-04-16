#ifndef ANALOG_IO_DRIVER_H
#define ANALOG_IO_DRIVER_H

#include "main.h"
#include <stdint.h>

/**
 * @brief Analog channel status structure
 */
struct AnalogChannelStatus
{
  unsigned long stamp_usec;  //!< timestamp (usec)
  float ch6_value;          //!< Channel 6 value (V or A)
  float ch8_value;          //!< Channel 8 value (V or A)
};

/**
 * @brief Analog I/O driver class
 */
class AnalogIoDriver
{
public:
  /**
   * @brief Construct a new Analog I/O Driver object
   */
  AnalogIoDriver();

  /**
   * @brief Construct a new Analog I/O Driver object
   *
   * @param master_can_id  master CAN ID (this host)
   * @param target_can_id  target CAN ID (analog I/O board)
   */
  AnalogIoDriver(uint8_t master_can_id, uint8_t target_can_id);

  /**
   * @brief Destructor
   */
  virtual ~AnalogIoDriver();

  /**
   * @brief Initialize the driver
   *
   * @param ican AnalogIoCanInterface object for communication
   * @param wait_response_time_usec wait response time after send command (usec)
   */
  void init(AnalogIoCanInterface* ican, uint16_t wait_response_time_usec = 0);

  /**
   * @brief Read analog channels 6 and 8
   *
   * @param is_voltage 1 for voltage input (mV to V), 0 for current input (μA to A)
   * @return true if data is successfully read, false otherwise
   */
  bool read_analog_channels(uint8_t is_voltage);

  /**
   * @brief Get the analog channel status
   *
   * @return AnalogChannelStatus current channel status
   */
  AnalogChannelStatus get_channel_status() const { return channel_status_; }

private:
  /**
   * @brief Send command to analog I/O board
   *
   * @param can_id target CAN ID
   * @param cmd_id command ID
   * @param option additional option data
   * @param len length of data
   * @param data data to send
   */
  void send_command(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data);

  /**
   * @brief Receive and process data from CAN interface
   *
   * @param status analog channel status
   * @param is_voltage 1 for voltage, 0 for current
   * @return true if data is successfully received and processed, false otherwise
   */
  bool receive_analog_data(AnalogChannelStatus& status, uint8_t is_voltage);

  /**
   * @brief Process received CAN packet
   *
   * @param data received data
   * @param len data length
   * @param is_voltage 1 for voltage, 0 for current
   */
  void process_analog_packet(const uint8_t* data, unsigned long len, uint8_t is_voltage);

  AnalogIoCanInterface* can_;          //!< CAN connection instance
  uint16_t wait_response_time_usec_;   //!< wait response time (usec)
  uint8_t master_can_id_;              //!< master CAN ID
  uint8_t target_can_id_;              //!< target CAN ID
  uint8_t receive_buffer_[8];          //!< receive buffer
  AnalogChannelStatus channel_status_;  //!< current channel status
  unsigned long send_count_;           //!< send count
};

#endif  // ANALOG_IO_DRIVER_H