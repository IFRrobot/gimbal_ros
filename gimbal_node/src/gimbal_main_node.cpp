#include "main.h"



int main(int argc, char **argv) {
    ros::init(argc, argv, "gimbal");
    ros::NodeHandle nh;

define_uart("/dev/pts/21",115200);
HandleInit(nh);
init_protocol();
 ros::Rate main_loop_rate(50);
    while(ros::ok()){
        handle_spin(); 
        ros::spinOnce();
        main_loop_rate.sleep();
    }
}
