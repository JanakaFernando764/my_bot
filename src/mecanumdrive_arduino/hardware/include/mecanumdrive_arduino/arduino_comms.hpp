#ifndef MECANUMDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define MECANUMDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:
  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      std::cerr << "The ReadByte() call has timed out." << std::endl;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }

  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_encoder_values(int &fl, int &fr, int &bl, int &br)
  {
    std::string response = send_msg("e\r");

    std::string delimiter = " ";
    size_t pos1 = response.find(delimiter);
    size_t pos2 = response.find(delimiter, pos1 + 1);
    size_t pos3 = response.find(delimiter, pos2 + 1);

    // Extract each token manually
    std::string token_fl = response.substr(0, pos1);
    std::string token_fr = response.substr(pos1 + delimiter.length(), pos2 - pos1 - delimiter.length());
    std::string token_bl = response.substr(pos2 + delimiter.length(), pos3 - pos2 - delimiter.length());
    std::string token_br = response.substr(pos3 + delimiter.length());

    fl = std::atoi(token_fl.c_str());
    fr = std::atoi(token_fr.c_str());
    bl = std::atoi(token_bl.c_str());
    br = std::atoi(token_br.c_str());
  }

  void set_motor_values(int fl, int fr, int bl, int br)
  {
    std::stringstream ss;
    ss << "m " << fl << " " << fr << " " << bl << " " << br << "\r";
    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

#endif // MECANUMDRIVE_ARDUINO_ARDUINO_COMMS_HPP