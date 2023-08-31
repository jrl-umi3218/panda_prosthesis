#ifndef BONETAGSERIAL_H
#define BONETAGSERIAL_H
#include <array>
#include <fstream>
#include <iostream>
namespace io
{
struct BoneTagSerial
{
  using Data = std::array<uint16_t, 4>;
  using RawData = std::array<uint16_t, 8>;

  BoneTagSerial();
  ~BoneTagSerial();
  void open(const std::string & descriptor);
  void close();
  bool connected() const noexcept;
  const Data & read();

  bool debug_bytes = false;
  bool debug_raw = false;
  bool debug_results = false;

  inline const std::string & descriptor() const noexcept
  {
    return descriptor_;
  }

protected:
  void sync();
  void get_input_data(bool print_bytes);
  void get_results(bool print_bytes, bool print_raw, bool print_result); 
  void parse_data(bool print_raw_data);
  void parse_result(bool print_result);
  void print_input_data();

protected:
  std::fstream f;
  std::string descriptor_;
  RawData rawData;
  Data result;
};
} // namespace io

#endif /* BONETAGSERIAL_H */
