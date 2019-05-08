#include "protocol.h"
using namespace std;

bool runFlag;
int recvbytes=0;
CmdMessage *recv_container_ptr_;
uint8_t cmdReceiveData[256]={};
uint8_t cmdSendData[256]={};

std::vector<CmdMessage> buffer_pool_;

bool init_protocol(){
    if(!init_uart()) return false;
    recv_container_ptr_=new CmdMessage();

    thread t1(receive_pool);
    t1.detach();
       //t1.join();
    return true;
}

void receive_pool(){

   ros::Rate server_loop_rate(50);
    while (ros::ok()) {
        runFlag=true;
        ros::Rate receive_loop_rate(1000);
        while(runFlag) {
            CmdMessage *container_ptr=Receive();
            if(container_ptr)
            {
                buffer_pool_.push_back(*container_ptr);
            }
            //usleep(100);
            receive_loop_rate.sleep();
        }
        server_loop_rate.sleep();
        //usleep(10000);
    }

}

CmdMessage *Receive(){
    uint8_t head_aa[1] = {};
    uint8_t head_temp[sizeof(CmdHead)] = {};
    CmdMessage *recv_container = recv_container_ptr_;

    memset(head_temp, 0, sizeof(head_temp));
    memset(head_aa, 0, sizeof(head_aa));
    recvbytes=read_uart(head_aa, sizeof(head_aa));
    if (recvbytes == 0 || recvbytes == -1) {
            ROS_WARN("Receive aa error !!!");
            runFlag = false;
            return nullptr;     
        }


    if(head_aa[0]==0xaa)
    {
        recvbytes=read_uart(head_temp, sizeof(head_temp)-1);//receive head from uart

        if (recvbytes == 0 || recvbytes == -1) {
            ROS_WARN("Receive  id+len error !!!");
            runFlag = false;
            return nullptr;     
        }

        recv_container->head.sof = head_aa[0];//head_temp[0];
        recv_container->head.id = head_temp[0];
        recv_container->head.len = head_temp[1];
     
        if (recv_container->head.sof == HEAD_SOF){// head check
            recvbytes=read_uart(cmdReceiveData, recv_container->head.len + recv_container->tail.len);//receive data+tail from uart
            if (recvbytes == 0 || recvbytes == -1) {
                ROS_WARN("Receive data error !!!");
                runFlag = false;
                return nullptr;
            }
            memcpy(recv_container->data,cmdReceiveData ,recv_container->head.len);

            recv_container->tail.crc_h = cmdReceiveData[recv_container->head.len + recv_container->tail.len-2];// frame tail H
            recv_container->tail.crc_l = cmdReceiveData[recv_container->head.len + recv_container->tail.len-1];// frame tail L
          
            uint8_t CRCReceiveData[sizeof(CmdHead)+recv_container->head.len]={};
            CRCReceiveData[0]= head_aa[0];
            CRCReceiveData[1]= head_temp[0];
            CRCReceiveData[2]= head_temp[1];
           
            memcpy(CRCReceiveData+sizeof(CmdHead),recv_container->data, recv_container->head.len);
            
            uint16_t crc=CRC16Calc(CRCReceiveData,sizeof(CRCReceiveData));
            uint16_t crc_rec=recv_container->tail.crc_h<<8 | recv_container->tail.crc_l;
            
            //ROS_INFO("crc=|%x|,tail=|%x|",crc,crc_rec);
            

            if (crc==crc_rec){// tail CRC check
                printf("receice data:\n");
                for(int i=0;i<sizeof(CRCReceiveData);i++) printf("%x ",CRCReceiveData[i]);
                printf("\n");
                
                memcpy(recv_container->data, cmdReceiveData, recv_container->head.len);

                //ROS_INFO("Receive head+data+tail !!!");
                //ROS_INFO("size=%d",sizeof(CRCReceiveData));
              
                return recv_container;
            }
            else{
                ROS_WARN("Receive frame tail error !!!");
                ROS_WARN("cmdReceiveData:");
                for(int i=0;i<recv_container->head.len;i++)ROS_WARN("|%x|",cmdReceiveData[i]);
                return nullptr;
            }
           
        }
        else{
            ROS_WARN("Receive head is not AA !!!");
            return nullptr;
        }   
    }
    else{
    return nullptr;
    }
    

}

bool Take(CmdMessage *recv_container)
{
    if(!buffer_pool_.size())return false;
    else{
        //ROS_INFO("recv_container SIZE %d",(int)buffer_pool_.size());
        *recv_container=buffer_pool_.front();
        buffer_pool_.erase(buffer_pool_.begin());
        return true;
    }
}

bool Send(CmdMessage &send_container)
{
    memset(cmdSendData, 0, sizeof(cmdSendData));

    send_container.head.sof=HEAD_SOF;
    cmdSendData[0]=send_container.head.sof;
    cmdSendData[1]=send_container.head.id;
    cmdSendData[2]=send_container.head.len;
    memcpy(cmdSendData + sizeof(CmdHead),send_container.data,send_container.head.len);

    
    uint16_t send_crc=CRC16Calc(cmdSendData,sizeof(CmdHead)+send_container.head.len);
   // ROS_INFO("CRCSEND |%x|,CRCSEND size |%d| ",send_crc,sizeof(CmdHead)+send_container.head.len);

    cmdSendData[sizeof(CmdHead)+send_container.head.len]=(send_crc&0xff);//crc_h
    cmdSendData[sizeof(CmdHead)+send_container.head.len+1]=(send_crc>>8);//crc_l
    
    write_uart(cmdSendData,send_container.head.len + sizeof(CmdHead) + sizeof(send_container.tail.len)+1);    
    //ROS_INFO("Send data success!!");
    
    return true;
}

uint16_t CRC16Update(uint16_t crc, uint8_t ch) {
  uint16_t tmp;
  uint16_t msg;

  msg = 0x00ff & static_cast<uint16_t>(ch);
  tmp = crc ^ msg;
  crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];

  return crc;
}


 uint16_t CRC16Calc(const uint8_t *data_ptr, size_t length) {
  size_t i;
  uint16_t crc = CRC_INIT;

  for (i = 0; i < length; i++) {
    crc = CRC16Update(crc, data_ptr[i]);
  }

  return crc;
}



