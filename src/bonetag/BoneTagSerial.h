#include <array>
#include <fstream>
#include <iostream>

namespace io
{
struct BoneTagSerial
{
  using Data = std::array<uint16_t, 4>;

  BoneTagSerial();
  ~BoneTagSerial();
  void open(const std::string & descriptor);
  void close();
  bool connected() const noexcept;
  const Data & read();

  inline const std::string & descriptor() const noexcept
  {
    return descriptor_;
  }

protected:
  void synchronize();

protected:
  std::fstream f;
  std::string descriptor_;
  std::array<uint16_t, 8> rawData;
  Data result;
};
} // namespace io
