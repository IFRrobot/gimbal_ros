#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <gimbal_node/gimbal.h>

#include <linux/input.h>
#include "thread"
using namespace std;



#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

#define KEYCODE_ENTER_CAP 0x0A

int stable_mode=0;
int pitch_mode=0;
int yaw_mode=0;

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
  double p_key_value=5;
  double v_key_value=2;
  bool stable=true;
  ros::Publisher gimbal_control_pub_;
 double secs;
 char last_c;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::NodeHandle nh_;

  gimbal_control_pub_ = nh_.advertise<gimbal_node::gimbal>("gimbal_control", 1);
 
  thread t1(ros_pub);
  t1.detach();
  signal(SIGINT,quit);
  
  char c;
  char s[3];


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
  puts("stable pitch yaw");
  puts("Use enter to switch the mode.");

  
  while(ros::ok())
  {
    // get the next event from the keyboard  
    secs =ros::Time::now().toSec();

    if(read(kfd, &c,1)< 0)
    {
      perror("read():");
      exit(-1);
    }
  /*secs=ros::Time::now().toSec()-secs;
  if(secs>0.2)
  {
  last_c=0x00;
  pitch_speed=0;
  yaw_speed=0;
  ROS_INFO("clear");
  }*/
  //ROS_INFO("value: 0x%02X\n", c);

    switch(c)
    {
      case KEYCODE_A:
      if(yaw_mode==0) 
      {
      yaw_position=yaw_position+p_key_value;    
      yaw_speed=0;
      }
      if(yaw_mode==1)
      {
        if(last_c==KEYCODE_A) yaw_speed=yaw_speed+v_key_value;
        else 
        { 
        yaw_speed=0;
        }
        yaw_position=0;
      }
      break;

      case KEYCODE_D:
	    if(yaw_mode==0) 
      {
      yaw_position=yaw_position-p_key_value;  
      yaw_speed=0;
      }
      if(yaw_mode==1)
      {
        if(last_c==KEYCODE_D) yaw_speed=yaw_speed-v_key_value; 
        else 
        { 
        yaw_speed=0;
        }
        yaw_position=0;
      }
      break;
      
   
    
      case KEYCODE_W:
	      if(pitch_mode==0)
        {
        pitch_position=pitch_position+p_key_value;
        pitch_speed=0;
        }
        if(pitch_mode==1)
        {
          if(last_c==KEYCODE_W) pitch_speed=pitch_speed+v_key_value;
          else 
          { 
          pitch_speed=0;
          }
          pitch_position=0;
        }
        break;
      case KEYCODE_S:
	      if(pitch_mode==0) 
        {
        pitch_position=pitch_position-p_key_value;
        pitch_speed=0;
        }
        if(pitch_mode==1)
        {
          if(last_c==KEYCODE_S) pitch_speed=pitch_speed-v_key_value;
          else 
          { 
          pitch_speed=0;
          }
          pitch_position=0;
        }
        break;
    
      case KEYCODE_ENTER_CAP:
      printf("input:\n");
      fgets(s,4,stdin);
      if(s[0]=='1') stable_mode=1;
      if(s[0]=='0') stable_mode=0;
      if(s[1]=='1') pitch_mode=1;
      if(s[1]=='0') pitch_mode=0;
      if(s[2]=='1') yaw_mode=1;
      if(s[2]=='0') yaw_mode=0;    
      printf("stable:%d pitch:%d yaw:%d\n",stable_mode,pitch_mode,yaw_mode);
        break; 
      dirty = true; 
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
    secs=ros::Time::now().toSec()-secs;
    if(secs>0.2)
    {
    last_c=0x00;
    pitch_speed=0;
    yaw_speed=0;
    //ROS_INFO("clear");
    }

      if(pitch_position>=35) pitch_position=35;
      if(pitch_position<=-55)pitch_position=-55;
      if(yaw_position>=125) yaw_position=125;
      if(yaw_position<=-125) yaw_position=-125;  
      if(pitch_speed>=10) pitch_speed=10;
      if(pitch_speed<=-10)pitch_speed=-10;
      if(yaw_speed>=10) yaw_speed=10;
      if(yaw_speed<=-10) yaw_speed=-10; 

      gimbal_control.mode=(stable_mode*4)+(pitch_mode*2)+yaw_mode;
      gimbal_control.stable_mode=stable_mode;
      gimbal_control.pitch_speed=pitch_speed;
      gimbal_control.yaw_speed =yaw_speed;
      gimbal_control.pitch_position=pitch_position;
      gimbal_control.yaw_position =yaw_position;
     
      gimbal_control_pub_.publish(gimbal_control);    

     
    ros::spinOnce();
    loop_rate.sleep();
  
  }
}







  
