#ifndef UART_H
#define UART_H

#include "ros/ros.h"
#include <string>

#include <termios.h>
#include <fcntl.h>  
#include <sys/select.h>


bool define_uart(std::string port_name,int baudrate);
bool init_uart();
bool close_uart();
bool open_uart();
bool config_uart();
int read_uart(uint8_t *buf, int len);
int write_uart(const uint8_t *buf, int len);

#endif
