#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <gimbal_node/gimbal.h>

#include <linux/input.h>
#include "thread"
using namespace std;

#define CMD_SET_STABLE_MODE 0x00
#define CMD_SET_GIMBAL_SPEED 0x01
#define CMD_SET_GIMBAL_POSITION 0x02


#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64


#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

#define KEYCODE_ENTER_CAP 0x0A

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

void ros_pub();   
gimbal_node::gimbal gimbal_control;
  bool dirty=false;
  double pitch_speed=0;
  double yaw_speed=0;
  double pitch_position=0;
  double yaw_position=0;
  double p_key_value=0.2;
  double v_key_value=0.02;
  bool stable=true;
  ros::Publisher gimbal_control_pub_;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::NodeHandle nh_;

  gimbal_control_pub_ = nh_.advertise<gimbal_node::gimbal>("gimbal_control", 1);
 
  thread t1(ros_pub);
  t1.detach();
  signal(SIGINT,quit);
  
  char c,last_c;



// get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use wasd to control the gimbal position.");
  puts("Use WASD to control the gimbal speed.");
  puts("Use enter to switch the gimbal stable mode.");

  
  while(ros::ok())
  {
    // get the next event from the keyboard  
   double secs =ros::Time::now().toSec();

    if(read(kfd, &c,1)< 0)
    {
      perror("read():");
      exit(-1);
    }
  secs=ros::Time::now().toSec()-secs;
  if(secs>0.2)
  {
  last_c=0x00;
  pitch_speed=0;
  yaw_speed=0;
  }
  //ROS_INFO("value: 0x%02X\n", c);

    switch(c)
    {
      case KEYCODE_A:
	      yaw_position=yaw_position+p_key_value;    
        gimbal_control.mode=CMD_SET_GIMBAL_POSITION;
        dirty = true;
        break;
      case KEYCODE_D:
	      yaw_position=yaw_position-p_key_value;  
        gimbal_control.mode=CMD_SET_GIMBAL_POSITION;
        dirty = true;  
        break;
      case KEYCODE_W:
	      pitch_position=pitch_position+p_key_value;
        gimbal_control.mode=CMD_SET_GIMBAL_POSITION;
        dirty = true;
        break;
      case KEYCODE_S:
	      pitch_position=pitch_position-p_key_value;
        gimbal_control.mode=CMD_SET_GIMBAL_POSITION;
        dirty = true;
        break;

      
      case KEYCODE_A_CAP:
        if(last_c==KEYCODE_A_CAP) yaw_speed=yaw_speed+v_key_value;
        else 
        { 
        yaw_speed=0;
        pitch_speed=0;
        }
        gimbal_control.mode=CMD_SET_GIMBAL_SPEED;
        dirty = true; 
        break;
      case KEYCODE_D_CAP:
	      if(last_c==KEYCODE_D_CAP) yaw_speed=yaw_speed-v_key_value; 
        else 
        { 
        yaw_speed=0;
        pitch_speed=0;
        }
        gimbal_control.mode=CMD_SET_GIMBAL_SPEED;
        dirty = true; 
        break;
      case KEYCODE_W_CAP:
        if(last_c==KEYCODE_W_CAP) pitch_speed=pitch_speed+v_key_value;
        else 
        { 
        yaw_speed=0;
        pitch_speed=0;
        }
        gimbal_control.mode=CMD_SET_GIMBAL_SPEED;
        dirty = true; 
        break;
      case KEYCODE_S_CAP:
	      if(last_c==KEYCODE_S_CAP) pitch_speed=pitch_speed-v_key_value;
        else 
        { 
        yaw_speed=0;
        pitch_speed=0;
        }
        gimbal_control.mode=CMD_SET_GIMBAL_SPEED;
        dirty = true;
        break;

      case KEYCODE_ENTER_CAP:
        stable=!stable;
        gimbal_control.mode=CMD_SET_STABLE_MODE;
        dirty = true; 
        break;
    }
   last_c=c;
  }
  return(0);
}

void ros_pub()
{  
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    
      if(pitch_position>=35) pitch_position=35;
      if(pitch_position<=-55)pitch_position=-55;
      if(yaw_position>=125) yaw_position=125;
      if(yaw_position<=-125) yaw_position=-125;  
      if(pitch_speed>=1) pitch_speed=1;
      if(pitch_speed<=-1)pitch_speed=-1;
      if(yaw_speed>=1) yaw_speed=1;
      if(yaw_speed<=-1) yaw_speed=-1; 

      if(gimbal_control.mode==CMD_SET_STABLE_MODE)
      {
      gimbal_control.stable_mode=stable;
      gimbal_control.pitch_speed=0;
      gimbal_control.yaw_speed =0;
      gimbal_control.pitch_position=0;
      gimbal_control.yaw_position =0;
      pitch_position=0;
      yaw_position=0;
      pitch_speed=0;
      yaw_speed=0;
      ROS_INFO("stable=%d",gimbal_control.stable_mode);
      }

      if(gimbal_control.mode==CMD_SET_GIMBAL_POSITION)
      {
      gimbal_control.pitch_speed=0;
      gimbal_control.yaw_speed =0;
      gimbal_control.pitch_position=pitch_position;
      gimbal_control.yaw_position =yaw_position;
      pitch_speed=0;
      yaw_speed=0;
      ROS_INFO("P_yaw=%f,P_pitch=%f",yaw_position,pitch_position);
      }

      if(gimbal_control.mode==CMD_SET_GIMBAL_SPEED)
      {
      gimbal_control.pitch_speed=pitch_speed;
      gimbal_control.yaw_speed =yaw_speed;
      gimbal_control.pitch_position=0;
      gimbal_control.yaw_position =0;
      ROS_INFO("V_yaw=%f,V_pitch=%f",yaw_speed,pitch_speed);
      }
      gimbal_control_pub_.publish(gimbal_control);    
     if(dirty==false)
     {
     pitch_speed=0;
     yaw_speed=0;
     }
     dirty=false;
    ros::spinOnce();
    loop_rate.sleep();
  }
}







  
