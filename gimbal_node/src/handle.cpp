#include "handle.h"
using namespace std;

uint8_t receive_mode;

SetGimbalStable stable_mode;

union VEL_DATA
{
    uint8_t velocity[sizeof(SetGimbalSpeed)];
    SetGimbalSpeed vel;
};
VEL_DATA vel_data; 

union POS_DATA
{
    uint8_t position[sizeof(SetGimbalPosition)];
    SetGimbalPosition pos;
};
POS_DATA pos_data; 



ros::Subscriber gimbal_control_sub ;

void gimbalcontrolCallback(const gimbal_node::gimbal& control_data)
{
    receive_mode=control_data.mode;

     if(control_data.mode==CMD_SET_STABLE_MODE)
    {
       stable_mode.stable=control_data.stable_mode;
    }

     if(control_data.mode==CMD_SET_GIMBAL_SPEED)
    {
        vel_data.vel.v_pitch=control_data.pitch_speed;
        vel_data.vel.v_yaw=control_data.yaw_speed;
    }

    if(control_data.mode==CMD_SET_GIMBAL_POSITION)
    {
        pos_data.pos.p_pitch=control_data.pitch_position;
        pos_data.pos.p_yaw=control_data.yaw_position;
    }

   

}

bool HandleInit(ros::NodeHandle nh)
{
    gimbal_control_sub = nh.subscribe("gimbal_control", 1, gimbalcontrolCallback);
    return true;
}




void CmdSend(const int8_t id,const int8_t len,uint8_t *data_ptr)
{
    CmdMessage send_message;
    send_message.head.id = id;
    send_message.head.len = len;
    memcpy(send_message.data,data_ptr, len); // when send map , len is 0 , will pass 
    send_message.data_ptr = data_ptr;
    Send(send_message);
}


void CmdProcess()
{
CmdMessage *senddata=new CmdMessage();    
//ROS_INFO("%x",receive_mode);

    if(receive_mode==CMD_SET_STABLE_MODE)
    {
    uint8_t stable_temp[sizeof(SetGimbalStable)]={};
    stable_temp[0]=stable_mode.stable;
    senddata->head.id=CMD_SET_STABLE_MODE;
    senddata->head.len=sizeof(SetGimbalStable);
    CmdSend(senddata->head.id ,senddata->head.len,stable_temp);
    }

    if(receive_mode==CMD_SET_GIMBAL_SPEED)
    {
    senddata->head.id=CMD_SET_GIMBAL_SPEED;
    senddata->head.len=sizeof(SetGimbalSpeed);
    CmdSend(senddata->head.id ,senddata->head.len,vel_data.velocity);
    }   

    if(receive_mode==CMD_SET_GIMBAL_POSITION)
    {
    senddata->head.id=CMD_SET_GIMBAL_POSITION;
    senddata->head.len=sizeof(SetGimbalPosition);
    CmdSend(senddata->head.id ,senddata->head.len,pos_data.position);
    }

/*for(int i=0;i<sizeof(SetGimbalPosition);i++)
{
printf("%x ",pos_data.position[i]);
}
printf("yaw=%f,pitch=%f\n",pos_data.pos.p_yaw,pos_data.pos.p_pitch);*/
  
//for(int i=0;i<senddata->head.len;i++) ROS_WARN("|%x|",senddata->data[i]);

}


void handle_spin()
{
    CmdMessage *recv_container=new CmdMessage();
    //if(Take(recv_container))
    //{
    CmdProcess(); //cmd process
    //}
    
}



