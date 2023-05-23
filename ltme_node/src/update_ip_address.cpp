#include <iostream>

#include "ldcp/device.h"
#include "ldcp/location.h"
#include <arpa/inet.h>

static const int FIRMWARE_BLOCK_SIZE = 512;
static const in_port_t DEFAULT_PORT = 2105;
ldcp_sdk::NetworkLocation parseDeviceAddress(const std::string &device_address);

class LtmeIoException : public std::exception
{ 
public:
  const char * what () const throw () override {
    return "Failed to read or write data from LTME device";
  }
};

int main(int argc, char* argv[])
{
  if (argc != 3) {
    std::string path(argv[0]);
    std::string file_name;
    auto index = path.find_last_of('/');
    file_name = path.substr((index != std::string::npos) ? index + 1 : 0);

    std::cout << "Usage: " << file_name << " <current device address> <new device address>" << std::endl;
    std::cout << "Example: " << file_name << " 192.168.10.160 10.0.0.160" << std::endl;
    return 0;
  }

  try {
    auto current_address = parseDeviceAddress(argv[1]);
    auto new_address = parseDeviceAddress(argv[2]);
    std::cout << "Connecting device at "
            << inet_ntoa({ current_address.address() }) << ":" << std::to_string(ntohs(current_address.port()))
            << "..." << std::endl;

    ldcp_sdk::Device device(current_address);
    if (device.open() != ldcp_sdk::no_error) {
      std::cerr << "failed" << std::endl;
      return -1;
    }

    std::string operation_mode;
    device.queryOperationMode(operation_mode);
    if (operation_mode == "normal") {
      std::cout << "Device is running in normal mode" << std::endl;
      if (device.setNetworkAddress(new_address.address()) != ldcp_sdk::no_error) {
        throw LtmeIoException();
      }
      if (device.persistSettings() != ldcp_sdk::no_error) {
        throw LtmeIoException();
      }
      std::cout << "done" << std::endl;
    }
    else {
      std::cout << "Device is not running in normal mode. Exiting..." << std::endl;
    }
    device.reboot();
    device.close();
  } catch(std::invalid_argument& e) {
    std::cerr << "Failed to parse device address: " << e.what() << std::endl;
    return -1;
  } catch(const LtmeIoException& e) {
    std::cerr << "Failed to set network address: " << e.what() << std::endl;
  }
  return 0;
}

ldcp_sdk::NetworkLocation parseDeviceAddress(const std::string &device_address)
{
  std::string address_str;
  std::string port_str;

  if (std::size_t position = device_address.find(':'); position != std::string::npos) {
    address_str = device_address.substr(0, position);
    port_str = device_address.substr(position + 1);
  }

  in_addr_t address;
  in_port_t port;

  address = inet_addr(address_str.c_str());
  if (address == htonl(INADDR_NONE)) {
    throw std::invalid_argument("Invalid device address: " + address_str);
  }
  port = htons(std::stoi(port_str));

  return ldcp_sdk::NetworkLocation(address, port);
}
