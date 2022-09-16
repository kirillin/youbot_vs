/**
 *
 *  author: kirillin
 *  created: 20.09.19.
**/

#ifndef SRC_YOUBOTVISUALSERVOING_H
#define SRC_YOUBOTVISUALSERVOING_H

#include <iostream>
#include <fstream>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <youbot_arm_kinematics/kinematics.h>

#include <boost/units/io.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <visp3/core/vpImage.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/blob/vpDot2.h>

#include <visp3/vision/vpPose.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpPixelMeterConversion.h>

// External camera parameters calibration
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/io/vpImageIo.h>

class YoubotVisualServoing {

    bool DEBUG = false;

    double START_TIME = 0;
    int RATE = 30;

    int Np = 4;
    double opt_square_width = 0.06;
    double L = opt_square_width / 2.;
    double distance_same_blob = 10.; // 2 blobs are declared same if their distance is less than this value

    ros::NodeHandle nh;
    ros::Publisher arm_velocity_pub;
    ros::Subscriber arm_js_sub;

    Kinematics ks;

    Matrix<double, 5, 1> q_state;
    Matrix<double, 5, 1> dq_state;
    Matrix<double, 5, 1> tau_state;

    Vector6d v_des;

    std::fstream file_joint_state;

  public:
    YoubotVisualServoing();
    ~YoubotVisualServoing();

    void js_callback(const sensor_msgs::JointState &msg);
    void publish(VectroNd dq);

    void pinv(Matrix<double, 8, 6>& L, Matrix<double, 6, 8>& pinvL, double alpha0 = 0.001, double w0 = 0.0001);
    void compute_error(std::vector<vpImagePoint>& ip, std::vector<vpImagePoint>& ipd, Matrix<double, 8, 1>& error);

    void compute_pose(std::vector <vpPoint> &point, const std::vector <vpImagePoint> &ip,
                     const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo);

    void work();
    void work_calibration();

};

#endif //SRC_YOUBOTVISUALSERVOING_H
