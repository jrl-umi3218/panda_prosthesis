#include <mc_rtc/io_utils.h>
#include "BoneTagSerial.h"
#include <array>
#include <fstream>
#include <iostream>

int main()
{
  io::BoneTagSerial serial;
  serial.open("/dev/ttyACM0");
  while(true)
  {
    const auto & data = serial.read();
    std::cout << mc_rtc::io::to_string(data) << std::endl;
  }
  serial.close();
}
