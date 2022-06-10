#include "BoneTagSerial.h"
#include <mc_rtc/logging.h>

namespace io
{
BoneTagSerial::BoneTagSerial()
{
  result.fill(0);
}

BoneTagSerial::~BoneTagSerial()
{
  close();
}

void BoneTagSerial::open(const std::string & descriptor)
{
  f.open(descriptor);
  if(!f.is_open())
  {
    mc_rtc::log::error_and_throw("[BoneTagSerial] Failed to open file descriptor {}", descriptor);
  }
  descriptor_ = descriptor;
}

void BoneTagSerial::close()
{
  f.close();
}

void BoneTagSerial::synchronize()
{
  // Find the synchronization char marking the start of the stream
  // FIXME (protocol) 'T' may be randomly contained withing the sensor readout
  // range leading to wrong measurements
  while(f.get() != 'T')
  {
  }
}

const BoneTagSerial::Data & BoneTagSerial::read()
{
  synchronize();

  std::array<uint8_t, 20> data;
  for(unsigned comp = 0; comp < 20; ++comp)
  {
    data[comp] = f.get();
  }

  f.flush();

  unsigned compt = 0;
  // Data is encoded on 12 bits, stored in 2 consecutive bytes:
  // - First 8 bits are the first 8 bits of the integer
  // - Second 8 bits: first 5 bits contains the rest of the number, last 3 bits are zero padding
  // Thus the number to be retrieved is stored as a 16 bits integer
  for(unsigned res = 0; res < 10; ++res)
  {
    // auto temp = static_cast<double>(data[compt]);
    result[res] = data[compt++] << 8; // Shift the first 8 received bits to the leftmost byte of the integer
    result[res] += data[compt++]; // Add the remaining 5 bits (+3 zero padding bits)
    result[res] = result[res] >> 3; // Remove the zero padding
  }
  return result;
}

} // namespace io
