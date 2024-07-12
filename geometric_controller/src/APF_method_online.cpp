#include "geometric_controller/utils_fun.h"
#include <algorithm>

#define gamma 3
#define d_0 2.5
#define k_att 0.5

// #define k_rep 0.7   // v2.0
#define k_rep 0.07  // v1.0

geometry_msgs::Pose pose;
geometry_msgs::Twist twist;
Matrix3d Rb;
Vector3d ang_vel;

void odomCallback(const nav_msgs::Odometry& msg);
void imuCallback(const sensor_msgs::Imu& msg);

void computeF_rep(Vector3d& F_rep, Vector3d obstacle);


int main(int argc, char** argv) {

    ros::init(argc, argv, "APF_method_online");
    ros::NodeHandle nh;

    double Ts = 0.01;
    ros::Rate loopRate(1.0 / Ts);
    
    ros::Publisher afp_pub = nh.advertise<geometry_msgs::Vector3>("/ardrone/geom_controller/apf", 10);
    ros::Publisher F_rep_pub = nh.advertise<geometry_msgs::Vector3>("/ardrone/geom_controller/F_rep", 10);
    ros::Publisher dist_pub = nh.advertise<geometry_msgs::Vector3>("/ardrone/geom_controller/dist", 10);

    ros::Subscriber imu_sub = nh.subscribe("/ardrone/ground_truth/imu", 10, imuCallback);
    ros::Subscriber odom_sub = nh.subscribe("/ardrone/ground_truth/odometry", 10, odomCallback);


    Vector3d goal(0,0,0);
    
    if (!loadGoal(goal)) {
        ROS_ERROR("Failed to load goal parameters.");
        return -1;
    }

    Vector3d F_a(0,0,0);
    Vector3d F_rep(0,0,0);
    Vector3d F_tot(0,0,0);
    Vector3d err_pos(0,0,0);

    LaserScanToPointCloud lstopc(nh);
 

    while(ros::ok()){

        // compute atrattive force
        err_pos = goal + PoseToEigen(pose); // positivo perche PoseToEigen restituisce ie parametri in NED

        if (err_pos.norm() < 1){
            F_a = k_att * err_pos;
        }
        else {
            F_a = k_att * err_pos/err_pos.norm();
        }
        

        // compute repulsive force v1.0
        sensor_msgs::PointCloud cloud = lstopc.getPointCloud();

        ROS_INFO_STREAM("numero punti trovati: " << cloud.points.size());

        for (int i = 0; i < cloud.points.size(); i++){

            if (pose.position.z>0.25){
                Eigen::Vector3d point(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z); // + 0.0455 + pose.position.z);
                ROS_INFO_STREAM("point: " << point.transpose());
                computeF_rep(F_rep, point);
            }
        }

        ROS_INFO_STREAM("F_rep: " << F_rep.transpose());
        F_tot = F_rep - F_a;

        afp_pub.publish(EigenToVector3(F_tot));
        F_rep_pub.publish(EigenToVector3(F_rep));
        //dist_pub.publish(EigenToVector3(dist_3d));

        F_rep << 0, 0, 0;

        ros::spinOnce();
        loopRate.sleep();

    }

    return 0;
}


void imuCallback(const sensor_msgs::Imu& msg) {

    ang_vel = AngVelToEigen(msg.angular_velocity);
    Rb = (quaternionToRotation(msg.orientation));
    Rb = enuToNed(Rb);

}

void odomCallback(const nav_msgs::Odometry& msg) {

    pose = msg.pose.pose;
    twist = msg.twist.twist;
    //Rb = quaternionToRotation(pose.orientation);

}

void computeF_rep(Vector3d& F_rep, Vector3d obstacle){
    // compute repulsive force
    Vector3d dist_3d = (obstacle);
    double dist_d    = (obstacle).norm();
    Vector3d grad_eta_q(0,0,0);

    ROS_INFO_STREAM("dist_3d: " << dist_3d.transpose());
    ROS_INFO_STREAM("dist_d: " << dist_d);

    if (dist_d <= d_0){
        grad_eta_q << dist_3d[0]/dist_d, dist_3d[1]/dist_d, dist_3d[2]/dist_d;
        F_rep = F_rep + (k_rep/pow(dist_d,2)) * pow((1/dist_d - 1/d_0),(gamma-1)) * grad_eta_q;
    }
}


