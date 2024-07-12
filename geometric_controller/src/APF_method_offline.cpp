#include "geometric_controller/utils_fun.h"
#include <algorithm>

#define num_points_height 50
#define num_points_circumference 100
#define gamma 3
#define d_0 2.5
#define k_att 0.5

#define k_rep 0.7   // v2.0
// #define k_rep 0.05  // v1.0

geometry_msgs::Pose pose;
geometry_msgs::Twist twist;
Matrix3d Rb;
Vector3d ang_vel;

void odomCallback(const nav_msgs::Odometry& msg);
void imuCallback(const sensor_msgs::Imu& msg);

struct obstacle{
    double center_x;
    double center_y;
    double center_z;
    double radius;
    double height;
};

std::vector<Vector3d> generateCylinderSurfacePoints(obstacle obs);
void computeF_rep(Vector3d& F_rep, Vector3d obstacle);


int main(int argc, char** argv) {

    ros::init(argc, argv, "APF_method_offline");
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

    Vector3d obstacle1(5,5,5);
    
    obstacle obs1, obs2, obs3;
    obs1.center_x = 3.0;
    obs1.center_y = 3.0;
    obs1.center_z = 0.0;
    obs1.radius = 0.70;
    obs1.height = 3.0;

    obs2.center_x = 5.0;
    obs2.center_y = 5.50;
    obs2.center_z = 0.0;
    obs2.radius = 0.50;
    obs2.height = 15.0;

    obs3.center_x = 8.0;
    obs3.center_y = 8.0;
    obs3.center_z = 0.0;
    obs3.radius = 0.20;
    obs3.height = 15.0;


    std::vector<Vector3d> obsPts1 = generateCylinderSurfacePoints(obs1);
    std::vector<Vector3d> obsPts2 = generateCylinderSurfacePoints(obs2);
    std::vector<Vector3d> obsPts3 = generateCylinderSurfacePoints(obs3);
 

    std::vector<Vector3d> obsPtsList = obsPts1;
    obsPtsList.insert(obsPtsList.end(), obsPts2.begin(), obsPts2.end());
    obsPtsList.insert(obsPtsList.end(), obsPts3.begin(), obsPts3.end());

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
        // for (size_t i = 0; i < obsPtsList.size(); i++){
        //     Vector3d& point = obsPtsList[i];
        //     computeF_rep(F_rep, point);
        // }


        // compute repulsive force v2.0
        std::vector<double> dist1;
        std::vector<double> dist2;
        std::vector<double> dist3;

        dist1.resize(obsPts1.size(),0);
        dist2.resize(obsPts2.size(),0);
        dist3.resize(obsPts3.size(),0);

        for (size_t i = 0; i < obsPts1.size(); i++){
            dist1[i] = (obsPts1[i] + PoseToEigen(pose)).norm();
            dist2[i] = (obsPts2[i] + PoseToEigen(pose)).norm();
            dist3[i] = (obsPts3[i] + PoseToEigen(pose)).norm();
        }

        auto min_dist1 = std::min_element(dist1.begin(),dist1.end());
        auto min_dist2 = std::min_element(dist2.begin(),dist2.end());
        auto min_dist3 = std::min_element(dist3.begin(),dist3.end());

        int index1  = std::distance(dist1.begin(),min_dist1);
        int index2  = std::distance(dist2.begin(),min_dist2);
        int index3  = std::distance(dist3.begin(),min_dist3);

        ROS_INFO_STREAM("mindist1: " << obsPts1[index1].transpose());
        ROS_INFO_STREAM("mindist2: " << obsPts2[index2].transpose());
        ROS_INFO_STREAM("mindist3: " << obsPts3[index3].transpose());
        
        computeF_rep(F_rep, obsPts1[index1]);
        computeF_rep(F_rep, obsPts2[index2]);
        computeF_rep(F_rep, obsPts3[index3]);

        ROS_INFO_STREAM("F: " << F_rep.transpose());
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
    Vector3d dist_3d = (obstacle + PoseToEigen(pose));
    double dist_d = (obstacle + PoseToEigen(pose)).norm();
    Vector3d grad_eta_q(0,0,0);

    if (dist_d <= d_0){
        grad_eta_q << dist_3d[0]/dist_d, dist_3d[1]/dist_d, dist_3d[2]/dist_d;
        F_rep = F_rep + (k_rep/pow(dist_d,2)) * pow((1/dist_d - 1/d_0),(gamma-1)) * grad_eta_q;
    }
}

std::vector<double> linspace(double start, double end, int num) {
    std::vector<double> linspace(num);
    double step = (end - start) / (num - 1);

    for (int i = 0; i < num; ++i) {
        linspace[i] = start + i * step;
    }

    return linspace;
}

std::vector<Vector3d> generateCylinderSurfacePoints(obstacle obs) {
    
    // Genera angoli da 0 a 2*pi per la circonferenza
    std::vector<double> theta = linspace(0, 2 * M_PI, num_points_circumference);
 
    // Genera altezze da center_z a center_z + height per il cilindro
    std::vector<double> z_values = linspace(obs.center_z, obs.center_z + obs.height, num_points_height);
 
    // Inizializza il vettore per i punti 3D
    std::vector<Vector3d> points;
    points.reserve(num_points_circumference * num_points_height);
 
    // Calcola le coordinate dei punti esterni lungo la superficie del cilindro
    for (const double& z : z_values) {
        for (int i = 0; i < num_points_circumference; ++i) {
            double x = obs.center_x + obs.radius * cos(theta[i]);
            double y = obs.center_y + obs.radius * sin(theta[i]);
            points.emplace_back(x, y, z);
        }
    }
 
    return points;
}
