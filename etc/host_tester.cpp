#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "../include/protocol.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <math.h>
#include <string.h>
#include <arpa/inet.h>  //ntoh

#include <chrono>

struct Parameter {
  std::string devicename;
  unsigned baud_rate;
} params_;

int serial_fd_ = -1;

bool open_serial_port() {
  if ((serial_fd_ = ::open(params_.devicename.c_str(), O_RDWR | O_NOCTTY)) < 0) {
    printf("Cannot open serial device %s to head_mcu!", params_.devicename.c_str());
    return false;
  }
  return true;
}

void set_serial_properties() {
  tcflush(serial_fd_, TCIOFLUSH); // flush previous bytes

  struct termios tio;
  if(tcgetattr(serial_fd_, &tio) < 0)
    perror("tcgetattr");

  tio.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  tio.c_oflag &= ~(ONLCR | OCRNL);
  tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  switch (params_.baud_rate)
  {
  case 9600:   cfsetospeed(&tio, B9600);   break;
  case 19200:  cfsetospeed(&tio, B19200);  break;
  case 38400:  cfsetospeed(&tio, B38400);  break;
  case 115200: cfsetospeed(&tio, B115200); break;
  case 230400: cfsetospeed(&tio, B230400); break;
  case 460800: cfsetospeed(&tio, B460800); break;
  case 500000: cfsetospeed(&tio, B500000); break;
  default:
    printf("Baudrate of %d not supported, using 115200!\n", params_.baud_rate);
    cfsetospeed(&tio, B115200);
    break;
  }
  cfsetispeed(&tio, cfgetospeed(&tio));

  if(tcsetattr(serial_fd_, TCSANOW, &tio) < 0) {
    printf("Could not set terminal attributes!\n");
    perror("tcsetattr");
  }
}

void send_command(const Command& cmd)
{
  if(::write(serial_fd_, &cmd, sizeof(decltype(cmd))) < 0){
    printf("could not write Command %d\n", cmd.type);
    return;
  }
  printf("Probably successfully sent command %d\n", cmd.type);
}


int main(int argc, char** argv)
{
    // TODO: Make parameter
    params_.devicename = "/dev/ttyACM0";
    params_.baud_rate = 115200;

    if(!open_serial_port()) {
    throw std::runtime_error("Could not open serial port " + params_.devicename);
    }

    set_serial_properties();
    printf("Opened device %s with baudrate %d\n", params_.devicename.c_str(),params_.baud_rate);


    Command cmd = {Command::Type::indicator, 0b10};
    send_command(cmd);
    while (true)
    {
        sleep(1);
    }
    ::close(serial_fd_);
    return 0;
}
