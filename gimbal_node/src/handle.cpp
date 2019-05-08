#include "handle.h"
using namespace std;



union GIMBAL_SEND_DATA
{
    uint8_t set_gimbal_data[sizeof(SetGimbalData)];
    SetGimbalData setdata;
};
GIMBAL_SEND_DATA gimbal_send_data; 

union GIMBAL_RECEIVE_DATA
{
    uint8_t get_gimbal_data[sizeof(GetGimbalData)];
    GetGimbalData getdata;
};
GIMBAL_RECEIVE_DATA gimbal_receive_data; 


ros::Subscriber gimbal_control_sub ;

void gimbalcontrolCallback(const gimbal_node::gimbal& control_data)
{
    uint8_t pitch_mode=(control_data.mode>>1)&0x01;
    uint8_t yaw_mode=control_data.mode&0x01;
    gimbal_send_data.setdata.mode=control_data.mode;

    if(pitch_mode==0)//position
    gimbal_send_data.setdata.pitch=control_data.pitch_position;
    if(pitch_mode==1)//position
    gimbal_send_data.setdata.pitch=control_data.pitch_speed;

    if(yaw_mode==0)//position
    gimbal_send_data.setdata.yaw=control_data.yaw_position;
    if(yaw_mode==1)//position
    gimbal_send_data.setdata.yaw=control_data.yaw_speed;
    
   // ROS_INFO("pitch_mode%d,yaw_mode:%d",pitch_mode,yaw_mode);
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
    //send_message.data_ptr = data_ptr;
    Send(send_message);
}


void CmdProcess()
{
CmdMessage *senddata=new CmdMessage();    
//ROS_INFO("%x",receive_mode);

    senddata->head.id=CMD_SEND_ID;
    senddata->head.len=sizeof(SetGimbalData);
    CmdSend(senddata->head.id,senddata->head.len,gimbal_send_data.set_gimbal_data);
     
}


void handle_spin()
{
    CmdMessage *recv_container=new CmdMessage();
    if(Take(recv_container))
    {
        memcpy(recv_container->data,gimbal_receive_data.get_gimbal_data ,recv_container->head.len);
        ROS_INFO("v_p:%f,v_y:%f   p_p:%f,p_y:%f",gimbal_receive_data.getdata.v_pitch,gimbal_receive_data.getdata.v_yaw,gimbal_receive_data.getdata.p_pitch,gimbal_receive_data.getdata.p_yaw );

    }
    CmdProcess(); //cmd process
    
    
}



