#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "uart.h"
#include "protocol_define.h"
#include "thread"

bool init_protocol();
void receive_pool();
CmdMessage *Receive();
uint16_t CRC16Calc(const uint8_t *data_ptr, size_t length);


bool Take(CmdMessage *recv_container);
bool Send(CmdMessage &send_container);
#endif