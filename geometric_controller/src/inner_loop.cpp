#include "geometric_controller/utils_fun.h"
#include <geometry_msgs/PoseArray.h>

float g = 9.81;

geometry_msgs::PoseArray poseArray;
geometry_msgs::Pose pose;
geometry_msgs::Twist twist;
Matrix3d Rb;
Matrix<double, 3, 3> Kr;
Matrix<double, 3, 3> Kw;
Vector3d zb_des;
Vector3d tau_b;
float psi;
float u_T;
Matrix3d Rb_des;
Vector3d omega_bb_des;
Vector3d err_r;
Vector3d err_w;
Vector3d ang_vel;

void odomCallback(const nav_msgs::Odometry& msg);
void imuCallback(const sensor_msgs::Imu& msg);
void psiCallback (const std_msgs::Float64& msg);
void zb_desCallback (const geometry_msgs::Vector3& msg);
void uTCallback (const std_msgs::Float64& msg);
Eigen::Matrix3d compute_dot_Rb_des(Matrix3d prev, double Ts);
Eigen::Vector3d compute_dot_omega_bb_des(Vector3d prev, double Ts);


void inner_loop (UAVParameters &uav_params, Matrix3d& dot_Rb_des, Vector3d& dot_omega_bb_des){

    Vector3d xb_des(cos(psi), sin(psi), 0);

    Vector3d yb_des = (skew(zb_des) * xb_des)/(skew(zb_des) * xb_des).norm();
    Rb_des.col(0) = skew(yb_des)*zb_des;
    Rb_des.col(1) = yb_des;
    Rb_des.col(2) = zb_des;

    // roll
    tau_b[0] = atan2(Rb_des(2,1), Rb_des(2,2));
    // pitch
    tau_b[1] = asin(-Rb_des(2,0));
    // yaw
    tau_b[2] = atan2(Rb_des(1,0), Rb_des(0,0));

    Vector3d omega_bb = ang_vel;
    ROS_INFO_STREAM("INNER omega_bb: " << omega_bb.transpose());

    dot_Rb_des = skew(ang_vel)*Rb_des;
    omega_bb_des = wedge(Rb_des.transpose() * dot_Rb_des);

    Vector3d err_R = 0.5 * wedge((Rb_des.transpose() * Rb) - (Rb.transpose() * Rb_des));
    Vector3d err_W = omega_bb - (Rb.transpose() * Rb_des * omega_bb_des);

    err_r = err_R;
    err_w = err_W;

    // tau_b = - Kr*err_R - Kw*err_W + skew(omega_bb)*uav_params.Ib*omega_bb - uav_params.Ib*(skew(omega_bb)*Rb.transpose()*Rb_des*omega_bb_des - Rb.transpose()*Rb_des*dot_omega_bb_des);


}


int main(int argc, char** argv) {
    ros::init(argc, argv, "inner_loop");
    ros::NodeHandle nh;

// variables 

    double Ts = 0.001;
    ros::Rate loopRate(1.0 / Ts);

    Kr << 2, 0, 0,
          0, 2.3, 0,
          0, 0, 0.15;
    Kw << 0.4, 0, 0,
          0, 0.52, 0,
          0, 0, 0.18;

// position_gain: {x: 6, y: 6, z: 6}
// velocity_gain: {x: 4.7, y: 4.7, z: 4.7}
// attitude_gain: {x: 2, y: 2.3, z: 0.15}
// angular_rate_gain: {x: 0.4, y: 0.52, z: 0.18}

    Matrix3d dot_Rb_des;
    Vector3d dot_omega_bb_des;
    Matrix3d prev_Rb_des;
    Vector3d prev_omega_bb_des;
    
    psi = 0;
    u_T = 0;
    zb_des << 0, 0, 1;

    Rb_des << 1,0,0,0,1,0,0,0,1;
    omega_bb_des << 0,0,0;

    dot_Rb_des << 1,0,0,0,1,0,0,0,1;
    dot_omega_bb_des << 0,0,0;

    UAVParameters uav_params;
    if (!loadUAVParameters(uav_params)) {
        ROS_ERROR("Failed to load UAV parameters.");
        return -1;
    }

    Eigen::Matrix4d AllocationMat = computeAllocationMatrix();

    // allocation matrix rotors simulator
    AllocationMat <<-5.44024e-07,  5.44031e-07,  5.44024e-07, -5.44031e-07,
                    -5.44033e-07,  5.44026e-07, -5.44033e-07,  5.44026e-07,
                    -1.36777e-07, -1.36777e-07,  1.36777e-07,  1.36777e-07,
                     8.54858e-06,  8.54858e-06,  8.54858e-06,  8.54858e-06;


    Eigen::Matrix4d I;
    I.setZero();
    I.block<3, 3>(0, 0) = uav_params.Ib;
    I(3, 3) = 1;

    Eigen::Vector4d control_commands (0,0,0,0); 
    Eigen::Vector4d motor_forces (0,0,0,0);    
    Eigen::Vector4d motor_speeds (0,0,0,0);
    ros::Publisher motor_pub = nh.advertise<mav_msgs::Actuators> ("/ardrone/command/motor_speed", 10);

    ros::Subscriber imu_sub = nh.subscribe("/ardrone/ground_truth/imu", 10, imuCallback);
    ros::Subscriber odom_sub = nh.subscribe("/ardrone/ground_truth/odometry", 10, odomCallback);

    ros::Publisher ctrl_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust> ("/ardrone/command/roll_pitch_yawrate_thrust", 10);
    ros::Publisher ctrl1_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust> ("/ardrone/command/roll_pitch_yawrate_thrust1", 10);

    ros::Subscriber psi_sub      = nh.subscribe ("/ardrone/geom_control/psi_des", 10, psiCallback);
    ros::Subscriber zb_des_sub   = nh.subscribe ("/ardrone/geom_control/zb_des", 10, zb_desCallback);
    ros::Subscriber u_T_sub      = nh.subscribe ("/ardrone/geom_control/u_T", 10, uTCallback);

    ros::Publisher err_R_pub   = nh.advertise<geometry_msgs::Vector3> ("/ardrone/geom_control/err_R", 10);
    ros::Publisher err_W_pub   = nh.advertise<geometry_msgs::Vector3> ("/ardrone/geom_control/err_W", 10);

    ros::Publisher omega_bb_pub           = nh.advertise<geometry_msgs::Vector3> ("/ardrone/geom_control/omega_bb", 10);
    ros::Publisher Rb_des_pub             = nh.advertise<geometry_msgs::Quaternion> ("/ardrone/geom_control/Rb_des", 10);
    ros::Publisher omega_bb_des_pub       = nh.advertise<geometry_msgs::Vector3> ("/ardrone/geom_control/omega_bb_des", 10);
    ros::Publisher dot_Rb_des_pub         = nh.advertise<geometry_msgs::Quaternion> ("/ardrone/geom_control/dot_Rb_des", 10);
    ros::Publisher dot_omega_bb_des_pub   = nh.advertise<geometry_msgs::Vector3> ("/ardrone/geom_control/dot_omega_bb_des", 10);

    // ros::Publisher path_pub	= nh.advertise<geometry_msgs::PoseArray>("/ardrone/geom_control/path", 1);

    // DiscreteTransferFunction derivatorVec(Ts);
    // DiscreteTransferFunction derivatorMat(Ts);

    DiscreteTransferFunction2 derivatorVec(Ts,omega_bb_des,Vector3d::Zero());
    DiscreteTransferFunction2 derivatorMat(Ts,Rb_des,Matrix3d::Zero());

        ROS_INFO_STREAM("INIZIO INNER");

    while(ros::ok()){

        //dot_Rb_des = derivatorMat.update(Rb_des);
        dot_omega_bb_des = derivatorVec.update(omega_bb_des);


        // INNER LOOP
        inner_loop(uav_params, dot_Rb_des, dot_omega_bb_des);

        // PATH PUBLISHER
        // path_pub.publish(poseArray);

        // ERROR R W PUBLISH
        geometry_msgs::Vector3 err_R_msg = EigenToVector3(err_r);
        err_R_pub.publish(err_R_msg);
         
        geometry_msgs::Vector3 err_W_msg = EigenToVector3(err_w);
        err_W_pub.publish(err_W_msg);


        // debug publish
        geometry_msgs::Vector3 temp;
        geometry_msgs::Quaternion tempMat;

        //temp = EigenToVector3(TwistAngularToEigen(twist));
        
        omega_bb_pub.publish(EigenToVector3(ang_vel));

        temp = EigenToVector3(omega_bb_des);
        omega_bb_des_pub.publish(temp);

        temp = EigenToVector3(dot_omega_bb_des);
        dot_omega_bb_des_pub.publish(temp);

        Eigen::Quaterniond quate(Rb_des);
        tempMat.x = quate.x(); 
        tempMat.y = quate.y(); 
        tempMat.z = quate.z(); 
        tempMat.w = quate.w(); 
        Rb_des_pub.publish(tempMat);

        Eigen::Quaterniond quate_dot(dot_Rb_des);
        tempMat.x = quate_dot.x(); 
        tempMat.y = quate_dot.y(); 
        tempMat.z = quate_dot.z(); 
        tempMat.w = quate_dot.w();   
        dot_Rb_des_pub.publish(tempMat);


        // // SATURATION COMMAND

        // if (tau_b[0] >= max_roll){
        //     tau_b[0] = max_roll;
        // }
        // else if (tau_b[0] < -max_roll){
        //     tau_b[0] = -max_roll;
        // }

        // if (tau_b[1] >= max_pitch){
        //     tau_b[1] = max_pitch;
        // }
        // else if (tau_b[1] < -max_pitch){
        //     tau_b[1] = -max_pitch;
        // }

        // if (tau_b[2] >= max_yaw_rate){
        //     tau_b[2] = max_yaw_rate;
        // }
        // else if (tau_b[2] < -max_yaw_rate){
        //     tau_b[2] = -max_yaw_rate;
        // }

        // if (u_T >= 35){
        //     u_T = 35;
        // }
        // else if (u_T < -35){
        //     u_T = -35;
        // }

        ROS_INFO_STREAM("INNER tau_b: " << tau_b.transpose());

        // RPY THRUST CONTROL PUBLISH
        mav_msgs::RollPitchYawrateThrust control_msg;

        ros::Time update_time = ros::Time::now();  
        control_msg.header.stamp = update_time;
        control_msg.header.frame_id = "rotors_geometric_frame";
        control_msg.roll = tau_b[0];
        control_msg.pitch = tau_b[1];
        control_msg.yaw_rate = tau_b[2];
        control_msg.thrust.x = 0;
        control_msg.thrust.y = 0;
        control_msg.thrust.z = u_T;

        // publish for bag file
        ctrl1_pub.publish(control_msg);
        // publish to control drone via RPYT tau_b u_T
        ctrl_pub.publish(control_msg);


        // DIRECT MOTOR CONTROL PUBLISH
        mav_msgs::Actuators motor_msg;
        
        control_commands << tau_b[0],tau_b[1],tau_b[2],u_T;
        //control_commands << 0, 0, 0, u_T;

        motor_forces = AllocationMat.inverse() * control_commands;

        motor_speeds << (motor_forces[0]),
                        (motor_forces[1]),
                        (motor_forces[2]),
                        (motor_forces[3]);

        motor_speeds = motor_speeds.cwiseMax(Eigen::VectorXd::Zero(4));
        motor_speeds = motor_speeds.cwiseSqrt();

        ROS_INFO_STREAM("INNER forces: " << motor_forces.transpose());
        ROS_INFO_STREAM("INNER motor: " << motor_speeds.transpose());

        motor_msg.header.stamp = ros::Time::now();
        motor_msg.angular_velocities.resize(4);

        for (int i=0; i<4 ;i++){
            motor_msg.angular_velocities[i] = motor_speeds[i];
        }

        //motor_pub.publish(motor_msg);


        ros::spinOnce();
        loopRate.sleep();

    }


    return 0;
}



void psiCallback (const std_msgs::Float64& msg){
    psi = msg.data;
}

void zb_desCallback (const geometry_msgs::Vector3& msg){
    zb_des.x() = msg.x;
    zb_des.y() = msg.y;
    zb_des.z() = msg.z;
}

void uTCallback (const std_msgs::Float64& msg){
    u_T = msg.data;
}




Eigen::Matrix3d compute_dot_Rb_des(Matrix3d prev, double Ts){
    return (Rb_des - prev)/Ts;
}

Eigen::Vector3d compute_dot_omega_bb_des(Vector3d prev, double Ts){
    return (omega_bb_des - prev)/Ts;
}


void imuCallback(const sensor_msgs::Imu& msg) {

    ang_vel = AngVelToEigen(msg.angular_velocity);
    Rb = (quaternionToRotation(msg.orientation));
    Rb = enuToNed(Rb);

// path computation
    // geometry_msgs::Pose poseStmp;
    // poseStmp.position.x = pose.position.x;
    // poseStmp.position.y = pose.position.y;
    // poseStmp.position.z = pose.position.z;
	// poseStmp.orientation = msg.orientation;

	// poseArray.header.stamp = ros::Time::now();
    // poseArray.header.frame_id = "world";

    // poseArray.poses.push_back(poseStmp);

}

void odomCallback(const nav_msgs::Odometry& msg) {

    pose = msg.pose.pose;
    //twist = msg.twist.twist;
    //Rb = quaternionToRotation(pose.orientation);

}