// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <robotiq_driver_noros/serial.hpp>

namespace robotiq_driver
{
/** One force/torque sample expressed in the sensor frame. */
struct FTReading
{
  std::array<double, 3> force;   // Fx, Fy, Fz [N]
  std::array<double, 3> torque;  // Mx, My, Mz [Nm]
};

/**
 * @brief Driver for the Robotiq FT 300-S force torque sensor over an RS-485/USB converter.
 *
 * The sensor manual (section 4.3) describes two communication modes:
 *  - data stream: 16-byte frames pushed at 100 Hz, started by writing 0x0200 to register 410
 *    (FC16) and stopped by a run of 0xff bytes. The sensor answers no request while streaming.
 *    This is the recommended way to read force and torque.
 *  - Modbus RTU: register reads with FC03 (19200 8N1, slave 9). Slower, and only answered while
 *    the stream is stopped.
 *
 * No member of this class throws. A control loop cannot afford an exception escaping into it, so
 * every failure - a read timeout, a bad CRC, an unplugged converter, a request the sensor rejects
 * - is reported by the return value, with the reason in last_error().
 */
class FTSensor
{
public:
  explicit FTSensor(std::unique_ptr<Serial> serial);
  ~FTSensor();

  void set_slave_address(uint8_t slave_address) noexcept;

  /** Why the last call that reported failure failed. Only meaningful after such a call. */
  [[nodiscard]] const std::string& last_error() const noexcept;

  /** Open the serial connection to the sensor. */
  bool connect() noexcept;

  /** Stop the stream if it is running and close the serial connection. */
  void disconnect() noexcept;

  /**
   * @brief Close and reopen the port, and restart the stream if it was running.
   *
   * The recovery path for a converter that was unplugged or re-enumerated: reads keep reporting
   * nothing until the port is opened again. The caller decides when it has seen enough failures.
   *
   * @return True if the sensor is delivering data again.
   */
  bool reconnect() noexcept;

  /**
   * @brief Start the data stream.
   *
   * The sensor sends no acknowledgement, so the command is confirmed by the first intact frame.
   * A sensor that is already streaming reports success without being restarted.
   *
   * @return True if a valid frame arrived.
   */
  bool start_stream() noexcept;

  /**
   * @brief Stop the data stream and discard what is left in the input buffer.
   *
   * Safe to call on a sensor that is not streaming, and the way to quiet a sensor left streaming
   * by an earlier run: it answers no request until the stream is stopped.
   *
   * @return True once the sensor is quiet, false if it keeps streaming.
   */
  bool stop_stream() noexcept;

  /**
   * @brief Read the next frame of the data stream.
   *
   * Resynchronizes on the frame header, so leftover bytes and a mid-frame start are recovered
   * from. A missing sample is not an error: a sensor that stopped streaming, a frame that failed
   * its CRC and a dead port all read as nothing, and the caller decides how many it tolerates.
   */
  std::optional<FTReading> read() noexcept;

  /**
   * @brief Read holding registers with FC03. Only answered while the stream is stopped.
   *
   * Retried a few times, since a stray byte desynchronizes one exchange but not the next. A
   * request the sensor rejects with a Modbus exception response reports the code in last_error().
   */
  std::optional<std::vector<uint16_t>> read_registers(uint16_t first_register, uint8_t num_registers) noexcept;

  /**
   * @brief Force and torque read from registers 180-185 with FC03.
   *
   * The fallback for when the stream is stopped; the stream is both faster and cheaper.
   */
  std::optional<FTReading> poll() noexcept;

  /** Production year of the sensor, register 514. Doubles as an "is this an FT 300-S" probe. */
  std::optional<uint16_t> production_year() noexcept;

private:
  /** One FC03 exchange. `rejected` tells a request the sensor answered "no" to from one that fell
      over on the wire, since only the second is worth another attempt. */
  std::optional<std::vector<uint16_t>> request_registers(uint16_t first_register, uint8_t num_registers,
                                                         bool& rejected) noexcept;

  std::vector<uint8_t> create_read_command(uint16_t first_register, uint8_t num_registers) const;
  std::vector<uint8_t> create_write_command(uint16_t first_register, const std::vector<uint16_t>& data) const;

  /** The serial calls, with the exceptions they throw turned into a reported failure. */
  bool open_port() noexcept;
  void close_port() noexcept;
  bool write(const std::vector<uint8_t>& data) noexcept;
  std::optional<std::vector<uint8_t>> read_bytes(size_t size) noexcept;

  /** Read and drop bytes until the input buffer runs dry. False if bytes keep coming. */
  bool drain() noexcept;

  void note(std::string reason) noexcept;

  std::unique_ptr<Serial> serial_ = nullptr;
  uint8_t slave_address_ = 0x09;
  bool streaming_ = false;
  std::string last_error_;
};
}  // namespace robotiq_driver
