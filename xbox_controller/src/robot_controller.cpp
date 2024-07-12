#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <math.h>

float A	= 0.0;
float B	= 0.0;
float X	= 0.0;
float Y	= 0.0;
float LB	= 0.0;
float RB	= 0.0;
float back	= 0.0;
float start	= 0.0;
float power	= 0.0;
float but_stick_L = 0.0;
float but_stick_R = 0.0;
float hor_axis_left = 0.0;
float ver_axis_left = 0.0;
float hor_axis_right = 0.0;
float ver_axis_richt = 0.0;
float LT = 0.0;
float RT = 0.0;
float cross_hor	= 0.0;
float cross_ver	= 0.0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    // Acquisisci i valori degli assi dal messaggio di joy

    A	= joy_msg->buttons[0];
    B	= joy_msg->buttons[1];
    X	= joy_msg->buttons[2];
    Y	= joy_msg->buttons[3];
    LB	= joy_msg->buttons[4];
    RB	= joy_msg->buttons[5];
    back	= joy_msg->buttons[6];
    start	= joy_msg->buttons[7];
    power	= joy_msg->buttons[8];
    but_stick_L = joy_msg->buttons[9];
    but_stick_R = joy_msg->buttons[10];
    hor_axis_left = joy_msg->axes[0];
    ver_axis_left = joy_msg->axes[1];
    hor_axis_right = joy_msg->axes[2];
    ver_axis_richt = joy_msg->axes[3];
    LT = joy_msg->axes[4];
    RT = joy_msg->axes[5];
    cross_hor	= joy_msg->axes[6];
    cross_ver	= joy_msg->axes[7];


}



int main(int argc, char** argv) {
    ros::init(argc, argv, "xbox_controller_node");
    ros::NodeHandle nh;


    // Sottoscrivi al topic /joy per leggere gli input del controller
    ros::Subscriber joy_sub = nh.subscribe("/joy", 10, joyCallback);

    ros::Rate loopRate(10);
    while (ros::ok()) {
        
        std::cout<<"A: "<<A<<std::endl;
        std::cout<<"B: "<<B<<std::endl;
        std::cout<<"X: "<<X<<std::endl;
        std::cout<<"Y: "<<Y<<std::endl;
        std::cout<<"LB: "<<LB<<std::endl;
        std::cout<<"RB: "<<RB<<std::endl;
        std::cout<<"back: "<<back<<std::endl;
        std::cout<<"start: "<<start<<std::endl;
        std::cout<<"power: "<<power<<std::endl;
        std::cout<<"but_stick_L: "<<but_stick_L<<std::endl;
        std::cout<<"but_stick_R: "<<but_stick_R<<std::endl;
        std::cout<<"hor_axis_left: "<<hor_axis_left<<std::endl;
        std::cout<<"ver_axis_left: "<<ver_axis_left<<std::endl;
        std::cout<<"hor_axis_right: "<<hor_axis_right<<std::endl;
        std::cout<<"ver_axis_richt: "<<ver_axis_richt<<std::endl;
        std::cout<<"LT: "<<LT<<std::endl;
        std::cout<<"RT: "<<RT<<std::endl;
        std::cout<<"cross_hor: "<<cross_hor<<std::endl;
        std::cout<<"cross_ver: "<<cross_ver<<std::endl;

        ros::spinOnce();
        loopRate.sleep();

    } 

    return 0;
}
