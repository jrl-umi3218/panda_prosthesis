#include <mc_rtc/io_utils.h>
#include "BoneTagSerial.h"
#include <array>
#include <fstream>
#include <iostream>
#include <string>

#include <chrono>
#include <signal.h>
#include <thread>

volatile static bool run = true;
io::BoneTagSerial serial;

void intHandler(int dummy)
{
  run = false;
}

bool connect(std::string serial_port_base, int serial_port_number)
{
  std::string serial_port = serial_port_base + std::to_string(serial_port_number);
  try
  {
    std::cout << "Trying to connect to " << serial_port << std::endl;
    serial.open(serial_port_base + std::to_string(serial_port_number));
    return true;
  }
  catch(std::runtime_error & e)
  {
    serial.close();
    std::cout << "Cannot connect to " << serial_port << std::endl;
    if(serial_port_number < 10)
    {
      return connect(serial_port_base, serial_port_number + 1);
    }
    return false;
  }
  return false;
}

int main(int argc, char ** argv)
{
  if(argc != 4)
  {
    std::cout << "USAGE => ./" << argv[0] << " <debug_bytes|0;1> <debug_raw|0;1> <debug_results|0;1>" << std::endl;
    exit(-1);
  }
  bool debug_bytes = atoi(argv[1]);
  bool debug_raw = atoi(argv[2]);
  bool debug_results = atoi(argv[3]);

  // signal(SIGINT, intHandler);
  serial.debug_bytes = debug_bytes;
  serial.debug_raw = debug_raw;
  serial.debug_results = debug_results;
  std::string serial_port_base = "/dev/ttyUSB";
  int serial_port_number = 0;
  if(!connect(serial_port_base, serial_port_number))
  {
    std::cerr << "Failed to connect" << std::endl;
    exit(-1);
  }
  while(run)
  {
    const auto & data = serial.read();
    std::cout << mc_rtc::io::to_string(data) << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  serial.close();
}
