#include "BoneTagSerial.h"
#include <mc_rtc/logging.h>
#include <mc_rtc/io_utils.h>

namespace io
{
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
  f.close();
  f.open(descriptor);
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
  return f.is_open();
}

void BoneTagSerial::synchronize()
{
  // Find the synchronization char marking the start of the stream
  // FIXME (protocol) 'T' may be randomly contained withing the sensor readout
  // range leading to wrong measurements
  char synch = 0;
  while(synch != 'A')
  {
    synch = f.get();
    std::cout << synch << std::endl;
    if(synch == 'A')
    {
      synch = 0;
      while(synch != 'T')
      {
        synch = f.get();
      }
      synch = 'A';
    }
  }
}

const BoneTagSerial::Data & BoneTagSerial::read()
{
  synchronize();

  std::array<uint8_t, 16> data;
  for(unsigned comp = 0; comp < 16; ++comp)
  {
    data[comp] = f.get();
  }

  f.flush();

  /* for (size_t i = 0 ; i < 8; i++) { */
  /*   rawData[i] = 0; */
  /* } */

  unsigned compt = 0;
  // Data is encoded on 12 bits, stored in 2 consecutive bytes:
  // - First 8 bits are the first 8 bits of the integer
  // - Second 8 bits: first 5 bits contains the rest of the number, last 3 bits are zero padding
  // Thus the number to be retrieved is stored as a 16 bits integer
  for(unsigned res = 0; res <= 7; ++res)
  {
    rawData[res] = data[compt] << 9; // Shift the first 9 received bits to the leftmost byte of the integer
    rawData[res] >>= 1;
    rawData[res] &= 32767;
    rawData[res] += data[compt + 1]; // Add the remaining 5 bits (+3 zero padding bits)
    rawData[res] = rawData[res] >> 5; // Remove the zero padding
    /* if(res >= 0) */
    /* { */
      compt += 2;
    /* } */
  }
  std::cout << "rawData: " << mc_rtc::io::to_string(rawData) << std::endl;
  result[0] = 500 * (rawData[2] - rawData[1]) / (rawData[2] + rawData[1]);
  result[1] = 500 * (rawData[3] - rawData[0]) / (rawData[3] + rawData[0]);
  result[2] = 500 * (rawData[6] - rawData[5]) / (rawData[6] + rawData[5]);
  result[3] = 500 * (rawData[7] - rawData[4]) / (rawData[7] + rawData[4]);

  return result;
}

} // namespace io
