#include <array>
#include <fstream>
#include <iostream>

namespace io
{
struct BoneTagSerial
{
  using Data = std::array<uint16_t, 10>;

  BoneTagSerial();
  ~BoneTagSerial();
  void open(const std::string & descriptor);
  void close();
  const Data & read();

protected:
  void synchronize();

protected:
  std::fstream f;
  std::array<uint16_t, 10> result;
};
} // namespace io
