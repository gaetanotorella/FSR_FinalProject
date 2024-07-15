#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/Actuators.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

#include <vector>
using namespace Eigen;

#define max_roll 10.0 * M_PI / 180.0  // [rad]
#define max_pitch 10.0 * M_PI / 180.0  // [rad]
#define max_yaw_rate 45.0 * M_PI / 180.0  // [rad/s]

// STRUCT

struct UAVParameters {
    double mass;
    Matrix3d Ib;
};

struct Trajectory {
    std::vector<Vector4d> q, dot_q, ddot_q;
};

struct TrajectoryAPF{
    Vector4d q, dot_q, ddot_q;
};

struct InitialFinalConditions {
    Vector4d q0, dot_q0, ddot_q0, dddot_q0;
    Vector4d qf, dot_qf, ddot_qf, dddot_qf;
};

// EIGEN FUNCTIONS

geometry_msgs::Vector3 EigenToVector3 (const Eigen::Vector3d& vec);

Eigen::Vector3d PoseToEigen(const geometry_msgs::Pose& pose);

Eigen::Vector3d Vec3MsgToEigen(const geometry_msgs::Vector3& ang_vel);

Eigen::Vector3d TwistLinearToEigen(const geometry_msgs::Twist& twist);

Eigen::Vector3d TwistAngularToEigen(const geometry_msgs::Twist& twist);

Eigen::Matrix3d quaternionToRotation(const geometry_msgs::Quaternion& quaternion);

Eigen::Matrix3d skew(const Eigen::Vector3d& t);

Eigen::Vector3d wedge(const Eigen::Matrix3d& A);

Eigen::Matrix3d enuToNed(const Eigen::Matrix3d& rotationMatrixENU);

// GET UAV PARAMETERS

bool loadUAVParameters(UAVParameters &uav_params);

Eigen::Matrix4d computeAllocationMatrix();

// TRAJECTORY GENERATOR

Trajectory generateTrajectory(const InitialFinalConditions& conditions, double t_iniz, double ttot, double tdead, double Ts);

// AFP LOOP FUNCTION

bool loadGoal(Vector3d& goal);

TrajectoryAPF APFplanner (Vector3d F, TrajectoryAPF& trajPrev, double Ts);



//  INNER LOOP FUNCTIONS

class DiscreteTransferFunction {
public:
    DiscreteTransferFunction(double Ts)
        : Ts(Ts), num{1, -1}, den{Ts, 0} {
        // Initialize previous inputs and outputs for Vector3d
        prevInputsVec = std::vector<Vector3d>(2, Vector3d::Zero());
        prevOutputsVec = std::vector<Vector3d>(2, Vector3d::Zero());

        // Initialize previous inputs and outputs for Matrix3d
        prevInputsMat = std::vector<Matrix3d>(2, Matrix3d::Zero());
        prevOutputsMat = std::vector<Matrix3d>(2, Matrix3d::Zero());
    }

    Vector3d update(const Vector3d& input) {
        // Shift the previous inputs and outputs
        prevInputsVec[1] = prevInputsVec[0];
        prevInputsVec[0] = input;

        Vector3d output = (num[0] * prevInputsVec[0] + num[1] * prevInputsVec[1] 
                           - den[1] * prevOutputsVec[0]) / den[0];

        // Shift the previous outputs
        prevOutputsVec[1] = prevOutputsVec[0];
        prevOutputsVec[0] = output;

        return output;
    }

    Matrix3d update(const Matrix3d& input) {
        // Shift the previous inputs and outputs
        prevInputsMat[1] = prevInputsMat[0];
        prevInputsMat[0] = input;

        Matrix3d output = (num[0] * prevInputsMat[0] + num[1] * prevInputsMat[1] 
                           - den[1] * prevOutputsMat[0]) / den[0];

        // Shift the previous outputs
        prevOutputsMat[1] = prevOutputsMat[0];
        prevOutputsMat[0] = output;

        return output;
    }

private:
    double Ts;
    std::vector<double> num;
    std::vector<double> den;

    // Vectors for storing previous inputs and outputs for Vector3d
    std::vector<Vector3d> prevInputsVec;
    std::vector<Vector3d> prevOutputsVec;

    // Vectors for storing previous inputs and outputs for Matrix3d
    std::vector<Matrix3d> prevInputsMat;
    std::vector<Matrix3d> prevOutputsMat;
};


// NUOVA DERIVATA GAETANO 27 GIUGNO

class DiscreteTransferFunction2 {
public:
    DiscreteTransferFunction2(double Ts, const Vector3d& initInputVec, const Vector3d& initOutputVec)
        : Ts(Ts), num{1, -1}, den{Ts, 0}, prevInputsVec(2), prevOutputsVec(2) {
        prevInputsVec[0] = initInputVec;
        prevOutputsVec[0] = initOutputVec;
        prevInputsVec[1] = Vector3d::Zero();
        prevOutputsVec[1] = Vector3d::Zero();
    }

    DiscreteTransferFunction2(double Ts, const Matrix3d& initInputMat, const Matrix3d& initOutputMat)
        : Ts(Ts), num{1, -1}, den{Ts, 0}, prevInputsMat(2), prevOutputsMat(2) {
        prevInputsMat[0] = initInputMat;
        prevOutputsMat[0] = initOutputMat;
        prevInputsMat[1] = Matrix3d::Zero();
        prevOutputsMat[1] = Matrix3d::Zero();
    }

    Vector3d update(const Vector3d& input) {
        // Shift the previous inputs and outputs
        prevInputsVec[1] = prevInputsVec[0];
        prevInputsVec[0] = input;

        Vector3d output = (num[0] * prevInputsVec[0] + num[1] * prevInputsVec[1] 
                           - den[1] * prevOutputsVec[0]) / den[0];

        // Shift the previous outputs
        prevOutputsVec[1] = prevOutputsVec[0];
        prevOutputsVec[0] = output;

        return output;
    }

    Matrix3d update(const Matrix3d& input) {
        // Shift the previous inputs and outputs
        prevInputsMat[1] = prevInputsMat[0];
        prevInputsMat[0] = input;

        Matrix3d output = (num[0] * prevInputsMat[0] + num[1] * prevInputsMat[1] 
                           - den[1] * prevOutputsMat[0]) / den[0];

        // Shift the previous outputs
        prevOutputsMat[1] = prevOutputsMat[0];
        prevOutputsMat[0] = output;

        return output;
    }

private:
    double Ts;
    std::vector<double> num;
    std::vector<double> den;

    // Vectors for storing previous inputs and outputs for Vector3d
    std::vector<Vector3d> prevInputsVec;
    std::vector<Vector3d> prevOutputsVec;

    // Vectors for storing previous inputs and outputs for Matrix3d
    std::vector<Matrix3d> prevInputsMat;
    std::vector<Matrix3d> prevOutputsMat;
};


class LaserScanToPointCloud{

    public:
        ros::NodeHandle n_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
        tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
        ros::Publisher scan_pub_;
        sensor_msgs::PointCloud cloud_;

        LaserScanToPointCloud(ros::NodeHandle n) : 
        n_(n),
        laser_sub_(n_, "/laser/scan", 10),
        laser_notifier_(laser_sub_,listener_, "ardrone/base_link", 10)
        {
        laser_notifier_.registerCallback(
            boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("ardrone/my_cloud",1);
        }

        void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in){
            try{
                projector_.transformLaserScanToPointCloud(
                    "ardrone/base_link",*scan_in, cloud_, listener_);
            }

            catch (tf::TransformException& e){
                //std::cout << e.what();
                return;
            }
            // PointCloud processing
            scan_pub_.publish(cloud_);
            //ROS_INFO_STREAM("PUBBLICATO");
        }

        sensor_msgs::PointCloud getPointCloud() const{
            return cloud_;
        }

};