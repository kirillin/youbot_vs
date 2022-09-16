/**
 *  Visual servoing for KUKA youBot arm.
 *
 *  author: kirillin
 *  created: 20.05.19.
**/

#include <fstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <fstream>
#include <visp3/core/vpImage.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/gui/vpDisplayX.h>

#include <visp3/vision/vpPose.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpPixelMeterConversion.h>

#include <visp3/imgproc/vpImgproc.h>
#include <visp3/core/vpImageFilter.h>

#include <visp3/blob/vpDot2.h>


#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>

#include <boost/units/io.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>

#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>

#include <youbot_arm_kinematics/kinematics.h>



class YoubotVS {

    ros::NodeHandle nh;
    std::ofstream out_file;
    double rate = 30;
    int Np = 4; ///number of points

    Kinematics ks;
    ros::Publisher arm_velocity_pub;
    ros::Subscriber arm_js_sub;

    Matrix<double, 1, 5> q_state;
    Vector6d v;

public:
    YoubotVS() {
        ROS_INFO("Youbot VS initialized.");

        VectorNd v1;
        v1 << 0.033, 0.155, 0.135, 0.0, 0.0;
        VectorNd v2;
        v2 << M_PI / 2.0, 0.0, 0.0, M_PI / 2.0, 0.0;
        VectorNd v3;
        v3 << 0.147, 0.0, 0.0, 0.0, 0.1136;
        VectorNd v4;
        v4 << M_PI * 169.0 / 180.0, M_PI * 65.0 / 180.0 + M_PI / 2, -M_PI * 146.0 / 180.0, M_PI * 102.5 / 180.0 + M_PI / 2,
                M_PI * 167.5 / 180.0;

        Kinematics ks_(v1, v2, v3, v4);
        ks = ks_;


        arm_velocity_pub = nh.advertise<brics_actuator::JointVelocities>("arm_1/arm_controller/velocity_command", 1);
        arm_js_sub = nh.subscribe("joint_states", 10, &YoubotVS::js_callback, this);


        v << 0,0,0, 0,0,0;

        out_file.open ("youbot_vs.txt");
    }

    ~YoubotVS() {
        arm_velocity_pub.shutdown();
        arm_js_sub.shutdown();
        out_file.close();
    }

    void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip,
                     const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo)
    {
        vpPose pose;
        double x = 0, y = 0;
        for (unsigned int i = 0; i < point.size(); i++) {
            vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
            point[i].set_x(x);
            point[i].set_y(y);
            pose.addPoint(point[i]);
        }

        if (init == true) {
            vpHomogeneousMatrix cMo_dem;
            vpHomogeneousMatrix cMo_lag;
            pose.computePose(vpPose::DEMENTHON, cMo_dem);
            pose.computePose(vpPose::LAGRANGE, cMo_lag);
            double residual_dem = pose.computeResidual(cMo_dem);
            double residual_lag = pose.computeResidual(cMo_lag);
            if (residual_dem < residual_lag)
                cMo = cMo_dem;
            else
                cMo = cMo_lag;
        }
        pose.computePose(vpPose::VIRTUAL_VS, cMo);
    }

    void js_callback(const sensor_msgs::JointState &msg) {

        brics_actuator::JointVelocities arm_velocities;
        vector<brics_actuator::JointValue> point;

        /* feedback measurements'q' */
        Matrix<double, 1, 5> q;
        for (int i = 0; i < 5; i++) {
            q(i) = msg.position[i];
        }
        q_state = q;

        Matrix<double, 6, 5> J;
        ks.get_jacobn(q, J);

        Matrix<double, 5, 6> pinvJ;
        ks.pinv(J, pinvJ);

        VectorNd dq = pinvJ * v;

        if (dq.sum() < 5) {
            std::stringstream joint_name;
            point.resize(5);
            for (int i = 0; i < 5 /*+2*/; i++) {
                if (i < 5) { /* link's joints */
                    joint_name.str("");
                    joint_name << "arm_joint_" << (i + 1);

                    point[i].joint_uri = joint_name.str();
                    point[i].value = dq(i);
                    point[i].unit = boost::units::to_string(boost::units::si::radian_per_second);
                } else {  /* gripper joints */
                    if (i == 5) {
                        point[i].joint_uri = "gripper_finger_joint_l";
                    } else {
                        point[i].joint_uri = "gripper_finger_joint_r";
                    }
                    point[i].value = 0;
                    point[i].unit = boost::units::to_string(boost::units::si::meter_per_second);
                }
            };
            arm_velocities.velocities = point;
            arm_velocity_pub.publish(arm_velocities);
        } else {
            clog << "STOP! Jacobian is close to singularity! Velocities sre very big!" << endl;
        }
    }

    void work() {
        try {
// *************************************************************************

            vpRealSense2 rs;
            rs2::config config;
            unsigned int width = 640, height = 480;
            config.disable_all_streams();
            config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
            rs.open(config);

            // Get camera extrinsics
            vpPoseVector ePc;
            ePc[0] = -0.03; ePc[1] = 0.0; ePc[2] = 0.0;
            ePc[3] = 0.0; ePc[4] = 0.0; ePc[5] = 0.0;

            vpHomogeneousMatrix eMc(ePc);
            std::cout << "eMc:\n" << eMc << "\n";

            // Get camera intrinsics
            vpCameraParameters cam = rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
            std::cout << "cam:\n" << cam << "\n";

            vpImage<unsigned char> I(height, width);

            vpDisplayX dc(I, 20, 40, "Color image");

            // Servo
            vpHomogeneousMatrix cdMc, cMo, cdMo, oMo;

            // Desired pose to reach
            cdMo[0][0] = 1; cdMo[0][1] =  0; cdMo[0][2] =  0;
            cdMo[1][0] = 0; cdMo[1][1] = -1; cdMo[1][2] =  0;
            cdMo[2][0] = 0; cdMo[2][1] =  0; cdMo[2][2] = -1;
            cdMo[0][3] = 0;
            cdMo[1][3] = 0;
            cdMo[2][3] = 0.1;

            cdMc = cdMo * cMo.inverse();

            std::vector<vpImagePoint> ip;   // 2D coordinates of the points in pixels
            std::vector<vpPoint> point;     // 3D coordinates of the points
            std::vector<vpDot2> dot(Np);    // 2D desired coordinates

            // Clicked points
            vpImagePoint germ[Np];
            vpDot2 blob[Np];

            // Setup desired coordinates
            double desired_square_width = 0.06;
            double L = desired_square_width / 2.;   // offsets from the center of square
            point.push_back(vpPoint(-L, -L, 0));
            point.push_back(vpPoint( L, -L, 0));
            point.push_back(vpPoint( L,  L, 0));
            point.push_back(vpPoint(-L,  L, 0));

            for(int i = 0; i < N; i++) {
                blob[i].setGraphics(true);
                blob[i].setGraphicsThickness(1);
            }

            vpFeaturePoint p[Np], pd[Np];

            vpServo task;
            task.setServo(vpServo::EYEINHAND_CAMERA);
            task.setInteractionMatrixType(vpServo::CURRENT);

            for (unsigned int i = 0; i < Np; i++) {
                point[i].track(cdMo);
                vpFeatureBuilder::create(pd[i], point[i]);

                dot[i].setGraphics(true);
                dot[i].initTracking(I);

                vpDisplay::flush(I);

                vpFeatureBuilder::create(p[i], cam, dot[i].getCog());
                task.addFeature(p[i], pd[i]);
            }


            vpHomogeneousMatrix wMc, wMo;

            Matrix<double, 1, N> q0;
            q0 << 3.2, 1.18, -1.06, -1.75, 0;    // edit to current position
            Transform<double, 3, Affine> H;
            ks.forward(q0, 0, 5, H);
//            for(int k = 0; k < 4; k++) {
//              for(int o = 0; 0 < 4; o++) {
//                  wMo[k][o] = H[k][o];
//              }
//            }


            std::cout << wMc << endl;

            cdMo = wMc * cMo;

//
//            vpAdaptiveGain lambda(1.5, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
//            task.setLambda(lambda);
            task.setLambda(0.5);

            bool final_quit = false;
            bool has_converged = false;
            bool send_velocities = false;
            bool servo_started = false;


            static double t_init_servo = vpTime::measureTimeMs();

//             init pose???
//            robot.set_eMc(eMc); // Set location of the camera wrt end-effector frame


//**************************************************************************
            ros::Rate R(rate);
            while(nh.ok() && !final_quit) {
                double t_start = vpTime::measureTimeMs();
                rs.acquire(I);

                for(int i = 0; i < Np; i++) {
                    std::stringstream ss;
                    ss << "p" << i;
                    vpDisplay::displayText(I, blob[i].getCog(), ss.str(), vpColor::white);
                    blob[i].track(I);
                    ip[i] = blob[i].getCog();
                }

                vpDisplay::display(I);
//                cMo = cMo_vec[0];

                std::stringstream ss;
                ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
                vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

                vpColVector v_c(6);



                if (!send_velocities) {
                    v_c = 0;
                }

                std::cout << v_c.transpose() << endl;

                for(int k = 0; k < 6; k++) {
                    v[k] = v_c[k];
                }

                ss.str("");
                ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
                vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
                vpDisplay::flush(I);

                vpMouseButton::vpMouseButtonType button;
                if (vpDisplay::getClick(I, button, false)) {
                    switch (button) {
                        case vpMouseButton::button1:
                            send_velocities = !send_velocities;
                            break;

                        case vpMouseButton::button3:
                            v_c = 0;
                            final_quit = true;
                            break;

                        default:
                            break;
                    }
                }

                ros::spinOnce();
                R.sleep();
            }

            task.kill();

            if (!final_quit) {
                while (!final_quit) {
                    rs.acquire(I);
                    vpDisplay::display(I);

                    vpDisplay::displayText(I, 20, 20, "Click to quit the program.", vpColor::red);
                    vpDisplay::displayText(I, 40, 20, "Visual servo converged.", vpColor::red);

                    if (vpDisplay::getClick(I, false)) {
                        final_quit = true;
                    }

                    vpDisplay::flush(I);
                }
            }

        } catch (const vpException &e) {
            std::stringstream ss;
            ss << "vpException: " << e;
            std::cout << ss.str() << std::endl;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "youbot_vs_node");

    YoubotVS ybvs;
    ybvs.work();

    return 0;
}
