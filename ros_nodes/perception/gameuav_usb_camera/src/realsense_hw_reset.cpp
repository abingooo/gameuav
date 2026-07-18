#include <librealsense2/rs.hpp>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
struct Options
{
  std::string serial;
  double find_timeout_sec = 8.0;
  double settle_timeout_sec = 7.0;
};

void print_usage(const char *argv0)
{
  std::cerr << "Usage: " << argv0
            << " [--serial SERIAL] [--find-timeout SEC] [--settle-timeout SEC]\n";
}

double parse_double(const char *value, const char *name)
{
  char *end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0')
  {
    throw std::runtime_error(std::string("invalid ") + name + ": " + value);
  }
  return parsed;
}

Options parse_args(int argc, char **argv)
{
  Options options;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h")
    {
      print_usage(argv[0]);
      std::exit(0);
    }
    if (arg == "--serial" && i + 1 < argc)
    {
      options.serial = argv[++i];
    }
    else if (arg == "--find-timeout" && i + 1 < argc)
    {
      options.find_timeout_sec = parse_double(argv[++i], "find timeout");
    }
    else if (arg == "--settle-timeout" && i + 1 < argc)
    {
      options.settle_timeout_sec = parse_double(argv[++i], "settle timeout");
    }
    else
    {
      throw std::runtime_error("unknown or incomplete argument: " + arg);
    }
  }
  return options;
}

std::string get_info(const rs2::device &device, rs2_camera_info info)
{
  if (!device.supports(info))
  {
    return "";
  }
  return device.get_info(info);
}

bool matches(const rs2::device &device, const std::string &serial)
{
  if (serial.empty())
  {
    return true;
  }
  return get_info(device, RS2_CAMERA_INFO_SERIAL_NUMBER) == serial;
}

rs2::device wait_for_device(rs2::context &context, const std::string &serial, double timeout_sec)
{
  const auto deadline = std::chrono::steady_clock::now()
      + std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000.0));

  while (std::chrono::steady_clock::now() < deadline)
  {
    const auto devices = context.query_devices();
    for (auto &&device : devices)
    {
      if (matches(device, serial))
      {
        return device;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  throw std::runtime_error(serial.empty()
      ? "no RealSense device found"
      : "RealSense device not found: " + serial);
}
}  // namespace

int main(int argc, char **argv)
{
  try
  {
    const Options options = parse_args(argc, argv);
    rs2::context context;
    rs2::device device = wait_for_device(context, options.serial, options.find_timeout_sec);

    const std::string serial = get_info(device, RS2_CAMERA_INFO_SERIAL_NUMBER);
    const std::string name = get_info(device, RS2_CAMERA_INFO_NAME);
    const std::string port = get_info(device, RS2_CAMERA_INFO_PHYSICAL_PORT);

    std::cout << "[realsense_hw_reset] resetting " << name
              << " serial=" << serial << " port=" << port << std::endl;
    device.hardware_reset();

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(options.settle_timeout_sec * 1000.0)));
    wait_for_device(context, serial, options.find_timeout_sec);
    std::cout << "[realsense_hw_reset] device is visible after reset" << std::endl;
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "[realsense_hw_reset] " << error.what() << std::endl;
    return 1;
  }
}
