#include "BoneTagSerial.h"
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>

namespace io
{
#define SYNC_FOUND prev_byte == 'A' && curr_byte == 'T'
#define NUM_BYTES 16
uint8_t prev_byte = 0;
uint8_t curr_byte = 0;
std::array<uint8_t, NUM_BYTES> input_data;

BoneTagSerial::BoneTagSerial()
{
  rawData.fill(0);
}

BoneTagSerial::~BoneTagSerial()
{
  close();
}

void BoneTagSerial::open(const std::string & descriptor)
{
  if(f.is_open())
  {
    f.close();
  }
  f.open(descriptor, std::fstream::in);
  if(!f.is_open())
  {
    throw std::runtime_error(fmt::format("[BoneTagSerial] Failed to open file descriptor {}", descriptor));
  }
  descriptor_ = descriptor;
}

void BoneTagSerial::close()
{
  f.close();
}

bool BoneTagSerial::connected() const noexcept
{
  return f.is_open() && f.good();
}
void BoneTagSerial::sync()
{
  while(true || f.good())
  {
    curr_byte = f.get();

    if(SYNC_FOUND)
    {
      return;
    }
    prev_byte = curr_byte;
  }
  if(!f.good())
  {
    throw std::runtime_error(fmt::format("[BoneTagSerial] Failed to sync (stream error flags are set)"));
  }
}
void BoneTagSerial::print_input_data()
{
  std::cout << "[";
  for(size_t i = 0; i < NUM_BYTES; i++)
  {
    std::cout << "\033[33m" << (int)input_data[i] << "\033[0m";
    if(i < NUM_BYTES - 1)
    {
      std::cout << " , ";
    }
  }
  std::cout << "]" << std::endl;
}
void BoneTagSerial::get_input_data(bool print_bytes)
{
  for(size_t i = 0; i < NUM_BYTES; i++)
  {
    input_data[i] = f.get();
  }
  if(print_bytes)
  {
    print_input_data();
  }
}
void BoneTagSerial::parse_data(bool print_raw_data)
{
  unsigned compt = 0;
  for(size_t i = 0; i < NUM_BYTES / 2; i++)
  {
    uint16_t currentRawData = input_data[compt] << 9;
    currentRawData >>= 1;
    currentRawData = currentRawData & 32767;
    currentRawData += input_data[compt + 1];
    currentRawData = currentRawData >> 5;
    rawData[i] = currentRawData;

    compt += 2;
  }
  if(print_raw_data)
  {
    std::cout << "rawData: " << mc_rtc::io::to_string(rawData) << std::endl;
  }
}

template<typename T>
T diff(const T & a, const T & b)
{
  return (a > b) ? (a - b) : (b - a);
}

void BoneTagSerial::parse_result(bool print_result)
{
  result[0] = 500 * diff(rawData[2], rawData[1]) / (rawData[2] + rawData[1]);
  result[1] = 500 * diff(rawData[3], rawData[0]) / (rawData[3] + rawData[0]);
  result[2] = 250 * diff(rawData[6], rawData[5]) / (rawData[6] + rawData[5]);
  result[3] = 250 * diff(rawData[7], rawData[4]) / (rawData[7] + rawData[4]);
  if(print_result)
  {
    for(size_t i = 0; i < result.size(); i++)
    {
      std::cout << "Result[" << i << "] = " << result[i] << std::endl;
    }
  }
}
void BoneTagSerial::get_results(bool print_bytes, bool print_raw, bool print_result)
{
  get_input_data(print_bytes);
  parse_data(print_raw);
  parse_result(print_result);
}
const BoneTagSerial::Data & BoneTagSerial::read()
{
  if(f.good())
  {
    sync();
    get_results(debug_bytes, debug_raw, debug_results);
  }
  else
  {
    throw std::runtime_error(fmt::format("[BoneTagSerial] Failed to read (stream error flags are set)"));
  }

  return result;
}

} // namespace io
