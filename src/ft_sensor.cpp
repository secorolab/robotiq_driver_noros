// SPDX-License-Identifier: BSD-3-Clause

#include <robotiq_driver_noros/crc_utils.hpp>
#include <robotiq_driver_noros/data_utils.hpp>
#include <robotiq_driver_noros/ft_sensor.hpp>

namespace robotiq_driver
{
constexpr uint8_t kReadFunctionCode = 0x03;
constexpr uint8_t kWriteFunctionCode = 0x10;

// A slave that rejects a request answers with the function code's high bit set and an error code.
constexpr uint8_t kExceptionFlag = 0x80;

// Write 0x0200 to register 410 to start the data stream.
constexpr uint16_t kStreamRegister = 410;
constexpr uint16_t kStreamStartValue = 0x0200;

// Force and torque as holding registers, and the production year.
constexpr uint16_t kFirstDataRegister = 180;
constexpr uint8_t kNumDataRegisters = 6;
constexpr uint16_t kProductionYearRegister = 514;

// A stream frame is <0x20><0x4e><LSB_data1><MSB_data1> ... <LSB_data6><MSB_data6><LSB_crc><MSB_crc>.
constexpr uint8_t kFrameHeaderFirst = 0x20;
constexpr uint8_t kFrameHeaderSecond = 0x4e;
constexpr size_t kFrameSize = 16;

// Two frames' worth of bytes is enough to find a header in a stream we joined mid-frame.
constexpr size_t kMaxScannedBytes = 2 * kFrameSize;

// The stream is confirmed by the first frame; give the sensor a few frame periods to start.
constexpr int kStreamStartAttempts = 5;

// A run of 0xff characters interrupts the stream. The sensor only reacts to an uninterrupted run
// of them, so the 50 the manual asks for go out as one burst; sent one at a time they are ignored.
constexpr uint8_t kStopStreamByte = 0xff;
constexpr size_t kStopStreamBurst = 50;
constexpr int kStopStreamAttempts = 3;

// A read response starts with the slave ID, the function code and the number of data bytes, and
// ends with a 2-byte CRC. An exception response replaces the data bytes with nothing at all.
constexpr size_t kResponseHeaderSize = 3;
constexpr size_t kCrcSize = 2;

// A stray byte desynchronizes one exchange but not the next one.
constexpr int kMaxRetries = 5;

// Bound the drain so a sensor that never stops streaming cannot hold us forever.
constexpr size_t kMaxDrainedBytes = 4096;

// Force is transmitted as N * 100, torque as Nm * 1000.
constexpr double kForceScale = 100.0;
constexpr double kTorqueScale = 1000.0;

namespace
{
/** Scale six raw signed 16-bit values into a reading. */
FTReading scale(const std::vector<uint16_t>& raw)
{
  auto value = [&raw](size_t i) { return static_cast<int16_t>(raw[i]); };
  return { { value(0) / kForceScale, value(1) / kForceScale, value(2) / kForceScale },
           { value(3) / kTorqueScale, value(4) / kTorqueScale, value(5) / kTorqueScale } };
}

/** Six little-endian 16-bit values, as they arrive in a stream frame. */
std::vector<uint16_t> to_registers(const std::vector<uint8_t>& data)
{
  std::vector<uint16_t> raw;
  raw.reserve(kNumDataRegisters);
  for (size_t i = 0; i < kNumDataRegisters; ++i)
  {
    raw.push_back(static_cast<uint16_t>(data[2 * i] | (data[2 * i + 1] << 8)));
  }
  return raw;
}

/** True if the last two bytes are the CRC of everything before them. */
bool crc_matches(const std::vector<uint8_t>& message)
{
  const std::vector<uint8_t> body{ message.begin(), message.end() - kCrcSize };
  const uint16_t crc = crc_utils::compute_crc(body);
  return message[message.size() - 2] == data_utils::get_msb(crc) &&
         message[message.size() - 1] == data_utils::get_lsb(crc);
}

/** The Modbus exception codes a slave can answer with. */
const char* exception_name(uint8_t code)
{
  switch (code)
  {
    case 0x01:
      return "illegal function";
    case 0x02:
      return "illegal data address";
    case 0x03:
      return "illegal data value";
    case 0x04:
      return "slave device failure";
    case 0x05:
      return "acknowledge";
    case 0x06:
      return "slave device busy";
    case 0x08:
      return "memory parity error";
    default:
      return "unknown exception";
  }
}
}  // namespace

FTSensor::FTSensor(std::unique_ptr<Serial> serial) : serial_{ std::move(serial) }
{
}

FTSensor::~FTSensor()
{
  disconnect();
}

void FTSensor::set_slave_address(uint8_t slave_address) noexcept
{
  slave_address_ = slave_address;
}

const std::string& FTSensor::last_error() const noexcept
{
  return last_error_;
}

bool FTSensor::connect() noexcept
{
  return open_port();
}

void FTSensor::disconnect() noexcept
{
  if (streaming_)
  {
    stop_stream();
  }
  close_port();
}

bool FTSensor::reconnect() noexcept
{
  const bool was_streaming = streaming_;
  streaming_ = false;

  close_port();
  if (!open_port())
  {
    return false;
  }
  return was_streaming ? start_stream() : true;
}

bool FTSensor::start_stream() noexcept
{
  if (!write(create_write_command(kStreamRegister, { kStreamStartValue })))
  {
    return false;
  }

  // The sensor answers the command by streaming, not by echoing it, and a sensor left streaming
  // by a previous run ignores the command altogether. Either way a frame is the only proof.
  streaming_ = true;
  for (int attempt = 0; attempt < kStreamStartAttempts; ++attempt)
  {
    if (read())
    {
      return true;
    }
  }
  note("the sensor did not start streaming");
  return false;
}

bool FTSensor::stop_stream() noexcept
{
  for (int attempt = 0; attempt < kStopStreamAttempts; ++attempt)
  {
    if (write(std::vector<uint8_t>(kStopStreamBurst, kStopStreamByte)) && drain())
    {
      streaming_ = false;
      return true;
    }
  }
  note("the sensor keeps streaming");
  return false;
}

std::optional<FTReading> FTSensor::read() noexcept
{
  uint8_t previous = 0x00;
  for (size_t scanned = 0; scanned < kMaxScannedBytes; ++scanned)
  {
    const auto byte = read_bytes(1);
    if (!byte)
    {
      return std::nullopt;  // nothing arrived: the sensor is not streaming, or the port is gone
    }
    if (previous != kFrameHeaderFirst || byte->front() != kFrameHeaderSecond)
    {
      previous = byte->front();
      continue;
    }

    // Header found: the rest of the frame is six data registers and the CRC.
    const auto body = read_bytes(kFrameSize - 2);
    if (!body)
    {
      return std::nullopt;
    }

    std::vector<uint8_t> frame = { kFrameHeaderFirst, kFrameHeaderSecond };
    frame.insert(frame.end(), body->begin(), body->end());
    if (!crc_matches(frame))
    {
      note("a stream frame failed its CRC");
      previous = 0x00;  // corrupt frame, keep looking for the next header
      continue;
    }

    return scale(to_registers(*body));
  }

  note("no stream frame header found");
  return std::nullopt;
}

std::optional<std::vector<uint16_t>> FTSensor::read_registers(uint16_t first_register, uint8_t num_registers) noexcept
{
  for (int attempt = 0; attempt < kMaxRetries; ++attempt)
  {
    bool rejected = false;
    if (auto registers = request_registers(first_register, num_registers, rejected))
    {
      return registers;
    }
    if (rejected)
    {
      break;  // the sensor answered; asking it again will not change the answer
    }
    drain();  // whatever is left in the buffer belongs to the attempt that just failed
  }
  return std::nullopt;
}

std::optional<FTReading> FTSensor::poll() noexcept
{
  const auto registers = read_registers(kFirstDataRegister, kNumDataRegisters);
  if (!registers)
  {
    return std::nullopt;
  }
  return scale(*registers);
}

std::optional<uint16_t> FTSensor::production_year() noexcept
{
  const auto registers = read_registers(kProductionYearRegister, 1);
  if (!registers)
  {
    return std::nullopt;
  }
  return registers->front();
}

std::optional<std::vector<uint16_t>> FTSensor::request_registers(uint16_t first_register, uint8_t num_registers,
                                                                 bool& rejected) noexcept
{
  if (!write(create_read_command(first_register, num_registers)))
  {
    return std::nullopt;
  }

  auto response = read_bytes(kResponseHeaderSize);
  if (!response)
  {
    return std::nullopt;
  }

  // A rejected request comes back as the function code with its high bit set, one error code and
  // the CRC - shorter than the answer we asked for, so the length has to follow the header.
  rejected = (*response)[1] == (kReadFunctionCode | kExceptionFlag);
  const auto rest = read_bytes(rejected ? kCrcSize : 2 * num_registers + kCrcSize);
  if (!rest)
  {
    return std::nullopt;
  }
  response->insert(response->end(), rest->begin(), rest->end());

  if (!crc_matches(*response))
  {
    rejected = false;  // a garbled response says nothing about what the sensor thinks
    note("the response failed its CRC");
    return std::nullopt;
  }
  if (rejected)
  {
    const uint8_t code = (*response)[2];
    note("the sensor rejected the request: " + std::string{ exception_name(code) } + " (exception code " +
         std::to_string(code) + ")");
    return std::nullopt;
  }
  if ((*response)[0] != slave_address_ || (*response)[1] != kReadFunctionCode ||
      (*response)[2] != 2 * num_registers)
  {
    const std::vector<uint8_t> header{ response->begin(), response->begin() + kResponseHeaderSize };
    note("unexpected response header " + data_utils::to_hex(header));
    return std::nullopt;
  }

  // The manual claims Robotiq sends the data part of a Modbus message least significant byte
  // first, but the sensor answers FC03 most significant byte first, as plain Modbus RTU does.
  // Only the data stream is little endian.
  std::vector<uint16_t> registers;
  registers.reserve(num_registers);
  for (size_t i = 0; i < num_registers; ++i)
  {
    const size_t at = kResponseHeaderSize + 2 * i;
    registers.push_back(static_cast<uint16_t>(((*response)[at] << 8) | (*response)[at + 1]));
  }
  return registers;
}

std::vector<uint8_t> FTSensor::create_read_command(uint16_t first_register, uint8_t num_registers) const
{
  std::vector<uint8_t> request = { slave_address_,
                                   kReadFunctionCode,
                                   data_utils::get_msb(first_register),
                                   data_utils::get_lsb(first_register),
                                   0x00,
                                   num_registers };
  const auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));
  return request;
}

std::vector<uint8_t> FTSensor::create_write_command(uint16_t first_register, const std::vector<uint16_t>& data) const
{
  const uint16_t num_registers = data.size();

  std::vector<uint8_t> request = { slave_address_,
                                   kWriteFunctionCode,
                                   data_utils::get_msb(first_register),
                                   data_utils::get_lsb(first_register),
                                   data_utils::get_msb(num_registers),
                                   data_utils::get_lsb(num_registers),
                                   static_cast<uint8_t>(2 * num_registers) };
  for (auto d : data)
  {
    request.push_back(data_utils::get_msb(d));
    request.push_back(data_utils::get_lsb(d));
  }

  const auto crc = crc_utils::compute_crc(request);
  request.push_back(data_utils::get_msb(crc));
  request.push_back(data_utils::get_lsb(crc));
  return request;
}

bool FTSensor::open_port() noexcept
{
  try
  {
    serial_->open();
    if (!serial_->is_open())
    {
      note("the serial port did not open");
      return false;
    }
    return true;
  }
  catch (const std::exception& e)
  {
    note(e.what());
    return false;
  }
}

void FTSensor::close_port() noexcept
{
  try
  {
    if (serial_->is_open())
    {
      serial_->close();
    }
  }
  catch (const std::exception& e)
  {
    note(e.what());  // the port is unusable either way
  }
}

bool FTSensor::write(const std::vector<uint8_t>& data) noexcept
{
  try
  {
    serial_->write(data);
    return true;
  }
  catch (const std::exception& e)
  {
    note(e.what());
    return false;
  }
}

std::optional<std::vector<uint8_t>> FTSensor::read_bytes(size_t size) noexcept
{
  try
  {
    return serial_->read(size);
  }
  catch (const std::exception& e)
  {
    // A read timeout, an unplugged converter and a closed port all arrive here as one of the
    // serial library's three unrelated exception types. None of them may reach the caller.
    note(e.what());
    return std::nullopt;
  }
}

bool FTSensor::drain() noexcept
{
  for (size_t dropped = 0; dropped < kMaxDrainedBytes; ++dropped)
  {
    if (!read_bytes(1))
    {
      return true;  // nothing left to read: the buffer is empty and the sensor is quiet
    }
  }
  return false;  // bytes keep coming, so the sensor is still streaming
}

void FTSensor::note(std::string reason) noexcept
{
  last_error_ = std::move(reason);
}
}  // namespace robotiq_driver
