#include "BoneTagSerial.h"
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <fcntl.h>
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
namespace io
{

#define BUFFER_SIZE 1024

static unsigned char buffer[BUFFER_SIZE];
bool packet_started = 0;
char packet_bytes[BUFFER_SIZE];
int packet_bytes_index = 0;
// Packet received_packet;
// SensorData sensors_data_decoded;
std::array<uint16_t, SENSOR_COUNT> last_received_data; // used for filtering

int baud_to_int(speed_t baud)
{
  switch(baud)
  {
    case B0:
      return 0;
    case B50:
      return 50;
    case B75:
      return 75;
    case B110:
      return 110;
    case B134:
      return 134;
    case B150:
      return 150;
    case B200:
      return 200;
    case B300:
      return 300;
    case B600:
      return 600;
    case B1200:
      return 1200;
    case B1800:
      return 1800;
    case B2400:
      return 2400;
    case B4800:
      return 4800;
    case B9600:
      return 9600;
    case B19200:
      return 19200;
    case B38400:
      return 38400;
    case B57600:
      return 57600;
    case B115200:
      return 115200;
    case B230400:
      return 230400;
    case B460800:
      return 460800;
    case B500000:
      return 500000;
    case B576000:
      return 576000;
    case B921600:
      return 921600;
    case B1000000:
      return 1000000;
    case B1152000:
      return 1152000;
    case B1500000:
      return 1500000;
    case B2000000:
      return 2000000;
    case B2500000:
      return 2500000;
    case B3000000:
      return 3000000;
    case B3500000:
      return 3500000;
    case B4000000:
      return 4000000;
    default:
      return -1; // Unknown baud rate
  }
}
speed_t int_to_baud(int baud)
{
  switch(baud)
  {
    case 0:
      return B0;
    case 50:
      return B50;
    case 75:
      return B75;
    case 110:
      return B110;
    case 134:
      return B134;
    case 150:
      return B150;
    case 200:
      return B200;
    case 300:
      return B300;
    case 600:
      return B600;
    case 1200:
      return B1200;
    case 1800:
      return B1800;
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return (speed_t)-1; // Unknown baud rate
  }
}
BoneTagSerial::BoneTagSerial()
{
  init_crc32_table();
}

BoneTagSerial::~BoneTagSerial()
{
  close_serial_port();
}

void BoneTagSerial::open_serial_port(const std::string & portName, const int baudRate)
{
  serialPort = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if(serialPort < 0)
  {
    throw std::runtime_error(fmt::format("[BoneTagSerial] Failed to open serial port {}", portName));
  }
  // Configure the serial port
  struct termios tty;
  if(tcgetattr(serialPort, &tty) != 0)
  {
    throw std::runtime_error(fmt::format("[BoneTagSerial] Error getting terminal attributes"));
    close(serialPort);
  }

  // Set baud rate to 9600
  cfsetispeed(&tty, int_to_baud(baudRate));
  cfsetospeed(&tty, int_to_baud(baudRate));

  // Configure 8N1 (8 data bits, no parity, 1 stop bit)
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
  tty.c_iflag &= ~IGNBRK; // Disable break processing
  tty.c_lflag = 0; // No canonical mode, no echo
  tty.c_oflag = 0; // No remapping, no delays
  tty.c_cc[VMIN] = 1; // Read at least 1 character
  tty.c_cc[VTIME] = 1; // Timeout after 0.1 seconds

  // Apply the settings to the serial port
  if(tcsetattr(serialPort, TCSANOW, &tty) != 0)
  {
    throw std::runtime_error(fmt::format("[BoneTagSerial] Error setting terminal attributes!"));
    close(serialPort);
  }
  isConnected = true;
}

void BoneTagSerial::close_serial_port()
{
  close(serialPort);
  isConnected = false;
}

bool BoneTagSerial::connected()
{
  if(fcntl(serialPort, F_GETFL) < 0)
  {
    isConnected = false;
  }
  /*
  int status;

  if(ioctl(serialPort, TIOCMGET, &status) == -1)
  {
    // Error in getting port status
    mc_rtc::log::error("[BoneTagSerial] Error getting serial port status !");
    return false;
  }
  //mc_rtc::log::error("[BoneTagSerial] Serial port status : {}",
                     //(status & TIOCM_CAR) != 0 ? "Connected" : "Disconnected");
  // Check the Data Carrier Detect (DCD) signal
  return (status & TIOCM_CAR) != 0;*/

  // bool status = serialPort.is_open() && serialPort.good();
  // return true;
  return isConnected;
}

void BoneTagSerial::read_serial_port()
{
  mc_rtc::log::info("[BoneTagSerial] Serial reading started");

  while(true)
  {
    int BytesRead = 0;
    do
    {
      BytesRead = read(serialPort, buffer, sizeof(buffer) - 1);
      // if we got something from serial, parse it, and indicate that serial is functioning
      if(BytesRead > 0)
      {

        parse_buffer(buffer, BytesRead);
        cycles_waited = 0;
        serial_status = true;
      }
      // if we have waited too many cycles, indicate that serial is not functioning
      else
      {
        if(++cycles_waited > cycles_timeout) serial_status = false;

        // throw std::runtime_error(fmt::format("[BoneTagSerial] Failed to read (too many cycles)"));
      }
    } while(BytesRead > 0);
  }
}

void BoneTagSerial::apply_filter(std::array<uint16_t, SENSOR_COUNT> & raw_data)
{
  // filtered_current = round(coef * raw_current + (1 - coef) * filtered_previous)
  size_t index = 0;
  for(auto & value : raw_data)
  {
    value = round(alphaFilter_ * value + (1 - alphaFilter_) * last_received_data[index]);
    index++;
  }
}

bool BoneTagSerial::validate_data(std::array<uint16_t, SENSOR_COUNT> & raw_data)
{
  for(auto & value : raw_data)
  {
    // if(value > 4100)
    // {
    //   mc_rtc::log::warning("[BoneTagSerial] Invalid read (read {}, max {})", value, 4100);
    //   return false;
    // }
  }
  return true;
}

void BoneTagSerial::parse_buffer(unsigned char * buff, size_t buff_len)
{
  for(size_t i = 0; i < buff_len; i++)
  {
    // std::cout << buff[i];
    if(!packet_started && buff[i] == 'A' && i + 1 < buff_len && buff[i + 1] == 'T')
    {
      packet_started = true;
      packet_bytes_index = 0;
      packet_bytes[packet_bytes_index++] = buff[i];
    }
    else if(packet_started)
    {
      packet_bytes[packet_bytes_index++] = buff[i];
      if(buff[i] == 'T' && i + 1 < buff_len && buff[i + 1] == 'A')
      {
        packet_bytes[packet_bytes_index++] = buff[++i];
        packet_started = false;

        auto decode_result = decode_packet(reinterpret_cast<const uint8_t *>(packet_bytes), packet_bytes_index);

        int decode_status = decode_result.second;
        if(decode_status == 0)
        {

          apply_filter(decode_result.first);
          bool is_valid_data = validate_data(decode_result.first);
          if(is_valid_data)
          {
            std::lock_guard<std::mutex> lock(received_data_mutex);
            received_data.push_back(decode_result.first);
            last_received_data = decode_result.first;
            lastSensorFrame_ = decode_result.first;
            gotFullFrame_ = true;
            /*std::ostringstream oss;
            for(auto value : received_packet.sensors_data.sensor_data)
            {
              oss << value << " ";
            }
            mc_rtc::log::info("[BoneTagSerial] New data : {} ", oss.str());*/
          }
        }
        else if(decode_status == -1)
        {
          // Packet size error
          mc_rtc::log::error("[BoneTagSerial] Packet size error, {}", packet_bytes_index);
        }

        else if(decode_status == -3)
        {
          mc_rtc::log::error("[BoneTagSerial] Crc validation error");
          // Crc validation error
        }

        packet_bytes_index = 0;
      }
    }
  }
}

} // namespace io
