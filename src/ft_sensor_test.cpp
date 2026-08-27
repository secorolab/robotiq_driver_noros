// SPDX-License-Identifier: BSD-3-Clause

#include <serial/serial.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <robotiq_driver_noros/default_serial.hpp>
#include <robotiq_driver_noros/ft_sensor.hpp>

#include "robotiq_driver_noros/command_line_utility.hpp"

constexpr auto kComPort = "/dev/ttyUSB0";
constexpr auto kBaudRate = 19200;
constexpr auto kTimeout = 0.1;
constexpr auto kSlaveAddress = 0x09;
constexpr auto kSamples = 100;

// At 100 Hz this is a tenth of a second of silence: long enough to mean more than a lost frame.
constexpr auto kMissesBeforeReconnect = 10;

using robotiq_driver::DefaultSerial;
using robotiq_driver::FTReading;
using robotiq_driver::FTSensor;
using robotiq_driver::Serial;

namespace
{
void print(const FTReading& reading)
{
  std::cout << std::fixed << std::setprecision(3)      //
            << "F = [" << reading.force[0] << ", "     //
            << reading.force[1] << ", "                //
            << reading.force[2] << "] N   M = ["       //
            << reading.torque[0] << ", "               //
            << reading.torque[1] << ", "               //
            << reading.torque[2] << "] Nm" << std::endl;
}

/** A Serial that replays a scripted byte sequence, so the frame parsing runs without a sensor. */
class ScriptedSerial : public Serial
{
public:
  explicit ScriptedSerial(std::vector<uint8_t> bytes) : bytes_{ bytes.begin(), bytes.end() }
  {
  }

  void open() override
  {
    open_ = true;
  }
  bool is_open() const override
  {
    return open_;
  }
  void close() override
  {
    open_ = false;
  }

  std::vector<uint8_t> read(size_t size) override
  {
    if (bytes_.size() < size)
    {
      THROW(serial::IOException, "out of scripted bytes");  // what a read timeout looks like
    }
    std::vector<uint8_t> data{ bytes_.begin(), bytes_.begin() + size };
    bytes_.erase(bytes_.begin(), bytes_.begin() + size);
    return data;
  }

  void write(const std::vector<uint8_t>& data) override
  {
    written.insert(written.end(), data.begin(), data.end());
  }

  void set_port(const std::string& port) override
  {
    port_ = port;
  }
  std::string get_port() const override
  {
    return port_;
  }
  void set_timeout(std::chrono::milliseconds timeout) override
  {
    timeout_ = timeout;
  }
  std::chrono::milliseconds get_timeout() const override
  {
    return timeout_;
  }
  void set_baudrate(uint32_t baudrate) override
  {
    baudrate_ = baudrate;
  }
  uint32_t get_baudrate() const override
  {
    return baudrate_;
  }

  std::vector<uint8_t> written;

private:
  std::deque<uint8_t> bytes_;
  bool open_ = false;
  std::string port_;
  std::chrono::milliseconds timeout_{ 0 };
  uint32_t baudrate_ = 0;
};

bool check(bool condition, const std::string& description)
{
  std::cout << (condition ? "ok   " : "FAIL ") << description << std::endl;
  return condition;
}

bool close_to(double value, double expected)
{
  return std::fabs(value - expected) < 1e-9;
}

/**
 * Frame parsing against a scripted stream: a mid-frame start, a good frame, then a frame with a
 * broken CRC. Fx..Mz are 1.23 N, -4.56 N, 7.89 N, 0.012 Nm, -0.034 Nm, 0.056 Nm.
 */
int selftest()
{
  const std::vector<uint8_t> frame = { 0x20, 0x4e, 0x7b, 0x00, 0x38, 0xfe, 0x15, 0x03,
                                       0x0c, 0x00, 0xde, 0xff, 0x38, 0x00, 0xe4, 0x5d };

  std::vector<uint8_t> bytes = { 0x11, 0x22, 0x20, 0x20 };  // tail of a frame we joined halfway
  bytes.insert(bytes.end(), frame.begin(), frame.end());
  bytes.insert(bytes.end(), frame.begin(), frame.end());
  bytes.back() ^= 0xff;  // corrupt the CRC of the second frame

  auto serial = std::make_unique<ScriptedSerial>(bytes);
  auto* written = &serial->written;
  FTSensor sensor{ std::move(serial) };

  bool passed = true;

  const auto reading = sensor.read();
  passed &= check(reading.has_value(), "a frame is found after leading garbage");
  if (reading)
  {
    passed &= check(close_to(reading->force[0], 1.23) && close_to(reading->force[1], -4.56) &&
                        close_to(reading->force[2], 7.89),
                    "force is decoded as N");
    passed &= check(close_to(reading->torque[0], 0.012) && close_to(reading->torque[1], -0.034) &&
                        close_to(reading->torque[2], 0.056),
                    "torque is decoded as Nm");
  }

  passed &= check(!sensor.read().has_value(), "a frame with a broken CRC is rejected");
  passed &= check(!sensor.read().has_value(), "a silent port reads as no sample, not an exception");

  sensor.set_slave_address(0x09);
  sensor.start_stream();
  const std::vector<uint8_t> expected = { 0x09, 0x10, 0x01, 0x9a, 0x00, 0x01, 0x02, 0x02, 0x00, 0xcd, 0xca };
  passed &= check(*written == expected, "start_stream writes the command from the manual");

  // The sensor ignores 0xff bytes that arrive one at a time, so they have to go out as one burst.
  written->clear();
  sensor.stop_stream();
  passed &= check(*written == std::vector<uint8_t>(50, 0xff), "stop_stream writes 50 0xff bytes in one burst");

  // Registers answer FC03 most significant byte first: 0x07e6 is the year 2022.
  FTSensor with_year{ std::make_unique<ScriptedSerial>(std::vector<uint8_t>{ 0x09, 0x03, 0x02, 0x07, 0xe6, 0xda,
                                                                            0x3f }) };
  const auto year = with_year.production_year();
  passed &= check(year.has_value() && *year == 2022, "an FC03 response decodes most significant byte first");

  // 0x83 is FC03 with the exception flag, 0x02 is "illegal data address".
  FTSensor rejecting{ std::make_unique<ScriptedSerial>(std::vector<uint8_t>{ 0x09, 0x83, 0x02, 0x41, 0x33 }) };
  passed &= check(!rejecting.read_registers(1, 1).has_value(), "a rejected request reports no registers");
  passed &= check(rejecting.last_error().find("illegal data address") != std::string::npos,
                  "a rejected request reports the exception code: " + rejecting.last_error());

  return passed ? 0 : 1;
}
}  // namespace

int main(int argc, char* argv[])
{
  CommandLineUtility cli;

  std::string port = kComPort;
  cli.registerHandler(
      "--port", [&port](const char* value) { port = value; }, false);

  int baudrate = kBaudRate;
  cli.registerHandler(
      "--baudrate", [&baudrate](const char* value) { baudrate = std::stoi(value); }, false);

  double timeout = kTimeout;
  cli.registerHandler(
      "--timeout", [&timeout](const char* value) { timeout = std::stod(value); }, false);

  int slave_address = kSlaveAddress;
  cli.registerHandler(
      "--slave-address", [&slave_address](const char* value) { slave_address = std::stoi(value); }, false);

  int samples = kSamples;
  cli.registerHandler(
      "--samples", [&samples](const char* value) { samples = std::stoi(value); }, false);

  bool run_selftest = false;
  cli.registerHandler("--selftest", [&run_selftest]() { run_selftest = true; });

  cli.registerHandler("-h", [&]() {
    std::cout << "Usage: ./ft_sensor_test [OPTIONS]\n"
              << "Options:\n"
              << "  --port VALUE                      Set the com port (default " << kComPort << ")\n"
              << "  --baudrate VALUE                  Set the baudrate (default " << kBaudRate << "bps)\n"
              << "  --timeout VALUE                   Set the read/write timeout (default " << kTimeout << "s)\n"
              << "  --slave-address VALUE             Set the slave address (default " << kSlaveAddress << ")\n"
              << "  --samples VALUE                   Number of samples to stream (default " << kSamples << ")\n"
              << "  --selftest                        Check the frame parsing without a sensor\n"
              << "  -h                                Show this help message\n";
    exit(0);
  });

  if (!cli.parse(argc, argv))
  {
    return 1;
  }

  if (run_selftest)
  {
    return selftest();
  }

  auto serial = std::make_unique<DefaultSerial>();
  try
  {
    serial->set_port(port);
    serial->set_baudrate(baudrate);
    serial->set_timeout(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout)));
  }
  catch (const std::exception& e)
  {
    // Configuring the port is the caller's job and can be rejected; the driver itself never throws.
    std::cout << "Failed to configure " << port << ": " << e.what() << std::endl;
    return 1;
  }

  auto sensor = std::make_unique<FTSensor>(std::move(serial));
  sensor->set_slave_address(slave_address);

  std::cout << "Using the following parameters: " << std::endl;
  std::cout << " - port: " << port << std::endl;
  std::cout << " - baudrate: " << baudrate << "bps" << std::endl;
  std::cout << " - read/write timeout: " << timeout << "s" << std::endl;
  std::cout << " - slave address: " << slave_address << std::endl;

  if (!sensor->connect())
  {
    std::cout << "The sensor is not connected: " << sensor->last_error() << std::endl;
    return 1;
  }
  std::cout << "The sensor is connected." << std::endl;

  // A sensor left streaming by an earlier run answers no request, so quiet it down first.
  std::cout << "Stopping any running data stream..." << std::endl;
  if (!sensor->stop_stream())
  {
    std::cout << "Failed: " << sensor->last_error() << std::endl;
    return 1;
  }

  if (const auto year = sensor->production_year())
  {
    std::cout << "Production year: " << *year << std::endl;
  }
  else
  {
    std::cout << "Failed to read the production year: " << sensor->last_error() << std::endl;
  }

  std::cout << "Polling force and torque over Modbus RTU..." << std::endl;
  if (const auto reading = sensor->poll())
  {
    print(*reading);
  }
  else
  {
    std::cout << "Failed to poll: " << sensor->last_error() << std::endl;
  }

  std::cout << "Starting the data stream..." << std::endl;
  if (!sensor->start_stream())
  {
    std::cout << "Failed: " << sensor->last_error() << std::endl;
    return 1;
  }

  int missed = 0;
  int consecutive_misses = 0;
  int reconnects = 0;
  for (int i = 0; i < samples; ++i)
  {
    const auto reading = sensor->read();
    if (reading)
    {
      consecutive_misses = 0;
      print(*reading);
      continue;
    }

    ++missed;
    if (++consecutive_misses < kMissesBeforeReconnect)
    {
      continue;
    }

    // Nothing has arrived for a while: the converter may have been unplugged and come back.
    std::cout << "No samples (" << sensor->last_error() << "), reconnecting..." << std::endl;
    consecutive_misses = 0;
    ++reconnects;
    if (!sensor->reconnect())
    {
      std::cout << "Reconnect failed: " << sensor->last_error() << std::endl;
    }
  }
  std::cout << "Missed " << missed << " of " << samples << " samples, " << reconnects << " reconnects." << std::endl;

  std::cout << "Stopping the data stream..." << std::endl;
  sensor->stop_stream();

  return 0;
}
