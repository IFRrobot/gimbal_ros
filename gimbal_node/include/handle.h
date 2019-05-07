
#ifndef HANDLE_H
#define HANDLE_H

#include "ros/ros.h"
#include "protocol_define.h"
#include "protocol.h"
#include "thread"
#include <gimbal_node/gimbal.h>

void handle_spin();
bool HandleInit(ros::NodeHandle nh);
void publish_listener();
#endif 