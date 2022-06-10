#include <mc_rtc/io_utils.h>
#include <array>
#include <fstream>
#include <iostream>

std::array<uint16_t, 10> read_datagram()
{
  std::string str;
  std::fstream f;
  // XXX Set an udev rule to consistently name the device
  // [73995.717650] usb 3-13: New USB device found, idVendor=0451, idProduct=bef3, bcdDevice= 1.00

  f.open("/dev/ttyACM0");

  // Find the synchronization char marking the start of the stream
  while(f.get() != 'T')
  {
  }

  // uint8_t
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
  std::array<uint16_t, 10> result;
  for(unsigned res = 0; res < 10; ++res)
  {
    // auto temp = static_cast<double>(data[compt]);
    result[res] = data[compt] << 8; // Shift the first 8 received bits to the leftmost byte of the integer
    result[res] += data[compt + 1]; // Add the remaining 5 bits (+3 zero padding bits)
    result[res] = result[res] >> 3; // Remove the zero padding
    compt += 2;
  }
  f.close();
  return result;
}

int main()
{
  std::array<uint16_t, 10> result;
  while(true)
  {
    std::cout << mc_rtc::io::to_string(read_datagram()) << std::endl;
  }
}

/**
 *             while(synch~='T')

                    synch = fscanf(s1,'%c',1); % Chaque caractère reçu est un chiffre codé en ASCII

            end

            for comp = 1:20

        data(comp)=fscanf(s1,'%c',1);

            end

        synch = 0;

        flushinput(s1);



for res = 1:10

        TEMP = double(data(compt));

        result(res,position) = bitshift(TEMP,8);

        result(res,position) = result(res,position)+data(compt+1);

        result(res,position) = bitshift(result(res,position),-3);

        compt = compt+2;

        result

end
 */
