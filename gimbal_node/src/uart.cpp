#include "uart.h"
using namespace std;


//! port name of the serial device
std::string port_name_;
//! baudrate of the serial device
int baudrate_;
//! stop bits of the serial device, as default
int stop_bits_;
//! data bits of the serial device, as default
int data_bits_;
//! parity bits of the serial device, as default
char parity_bits_;
//! serial handler
int serial_fd_;
//! set flag of serial handler
fd_set serial_fd_set_;
//! termios config for serial handler
struct termios new_termios_, old_termios_;





bool define_uart(std::string port_name,int baudrate) 
{
    port_name_=port_name;
    baudrate_=baudrate;
    data_bits_=8;
    parity_bits_='N';
    stop_bits_=1;
}

bool init_uart() {

  ROS_INFO ("Attempting to open device:%s with baudrate:%d ",port_name_.c_str(),baudrate_);
  if (port_name_.c_str() == nullptr) {
    port_name_ = "/dev/pts/22";
  }
  if (open_uart() && config_uart()) {
    FD_ZERO(&serial_fd_set_);
    FD_SET(serial_fd_, &serial_fd_set_);
    ROS_INFO("...Serial started successfully.");
    return true;
  } else {
    ROS_WARN  ("...Failed to start serial%s ",port_name_.c_str());
    close_uart();
    return false;
  }
}

bool close_uart() {
  close(serial_fd_);
  serial_fd_ = -1;
  return true;
}


bool open_uart() {
#ifdef __arm__
  serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NONBLOCK);
#elif __x86_64__
  serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#else
  serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#endif

  if (serial_fd_ < 0) {
    ROS_WARN("cannot open device%s,%d ",port_name_.c_str(),serial_fd_);
    return false;
  }
  return true;
}

bool config_uart() {
  int st_baud[] = {B4800, B9600, B19200, B38400,
                   B57600, B115200, B230400, B921600};
  int std_rate[] = {4800, 9600, 19200, 38400, 57600, 115200,
                    230400, 921600, 1000000, 1152000, 3000000};
  int i, j;
  /* save current port parameter */
  if (tcgetattr(serial_fd_, &old_termios_) != 0) {
    ROS_WARN ("fail to save current port");
    return false;
  }
  memset(&new_termios_, 0, sizeof(new_termios_));

  /* config the size of char */
  new_termios_.c_cflag |= CLOCAL | CREAD;
  new_termios_.c_cflag &= ~CSIZE;

  /* config data bit */
  switch (data_bits_) {
    case 7:new_termios_.c_cflag |= CS7;
      break;
    case 8:new_termios_.c_cflag |= CS8;
      break;
    default:new_termios_.c_cflag |= CS8;
      break; //8N1 default config
  }
  /* config the parity bit */
  switch (parity_bits_) {
    /* odd */
    case 'O':
    case 'o':new_termios_.c_cflag |= PARENB;
      new_termios_.c_cflag |= PARODD;
      break;
      /* even */
    case 'E':
    case 'e':new_termios_.c_cflag |= PARENB;
      new_termios_.c_cflag &= ~PARODD;
      break;
      /* none */
    case 'N':
    case 'n':new_termios_.c_cflag &= ~PARENB;
      break;
    default:new_termios_.c_cflag &= ~PARENB;
      break; //8N1 default config
  }
  /* config baudrate */
  j = sizeof(std_rate) / 4;
  for (i = 0; i < j; ++i) {
    if (std_rate[i] == baudrate_) {
      /* set standard baudrate */
      cfsetispeed(&new_termios_, st_baud[i]);
      cfsetospeed(&new_termios_, st_baud[i]);
      break;
    }
  }
  /* config stop bit */
  if (stop_bits_ == 1)
    new_termios_.c_cflag &= ~CSTOPB;
  else if (stop_bits_ == 2)
    new_termios_.c_cflag |= CSTOPB;
  else
    new_termios_.c_cflag &= ~CSTOPB; //8N1 default config

/* config waiting time & min number of char */
  new_termios_.c_cc[VTIME] = 1;
  new_termios_.c_cc[VMIN] = 18;

  /* using the raw data mode */
  new_termios_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  new_termios_.c_oflag &= ~OPOST;

  /* flush the hardware fifo */
  tcflush(serial_fd_, TCIFLUSH);

  /* activite the configuration */
  if ((tcsetattr(serial_fd_, TCSANOW, &new_termios_)) != 0) {
    ROS_WARN("failed to activate serial configuration");
    return false;
  }
  return true;

}

int read_uart(uint8_t *buf, int len) {
  int ret = -1;
  //ROS_INFO("Run read");
  if (NULL == buf) {
      ROS_INFO("run NULL");
    return -1;
  } else {
//    LOG_INFO<<"buf:"<< sizeof(*buf);
    ret = read(serial_fd_, buf, len);
   // ROS_INFO("Read once length:%d",ret);
    while (ret == 0) {
      ROS_WARN("Connection closed, try to reconnect.");
      while (!init_uart()) {
        usleep(500000);
      }
      ROS_INFO ("Reconnect Success.");
      ret = read(serial_fd_, buf, len);
    }
//      LOG_INFO<<"run  return ret:"<<ret;
    return ret;
  }
}

int write_uart(const uint8_t *buf, int len) {
  return write(serial_fd_, buf, len);
}



