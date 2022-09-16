/**
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

#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>

#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/io/vpImageIo.h>


#include <boost/units/io.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/velocity.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>

#include <youbot_arm_kinematics/kinematics.h>

class YoubotVS {

    ros::NodeHandle nh;
    std::ofstream out_file;
    int Np = 4;
    int RATE = 30;

    Kinematics ks;
    ros::Publisher arm_velocity_pub;
    ros::Subscriber arm_js_sub;

    Matrix<double, 1, 5> q_state;
    Vector6d v;


    double opt_square_width = 0.06;
    double L = opt_square_width / 2.;
    double distance_same_blob = 10.; // 2 blobs are declared same if their distance is less than this value



public:
    YoubotVS() {
        ROS_INFO("Youbot VS initialized.");

        VectorNd v1; v1 << 0.033, 0.155, 0.135, 0.0, 0.0;
        VectorNd v2; v2 << M_PI/2.0, 0.0, 0.0, M_PI/2.0, 0.0;
        VectorNd v3; v3 << 0.147, 0.0, 0.0, 0.0, 0.218;
        VectorNd v4; v4 << M_PI*169.0/180.0, M_PI*65.0/180.0+M_PI/2, -M_PI*146.0/180.0, M_PI*102.5/180.0+M_PI/2, M_PI*167.5/180.0;
        Kinematics ks_(v1, v2, v3, v4);
        ks = ks_;


        arm_velocity_pub = nh.advertise<brics_actuator::JointVelocities>("arm_1/arm_controller/velocity_command", 1);
        arm_js_sub = nh.subscribe("joint_states", 10, &YoubotVS::js_callback, this);


        v << 0,0,0, 0,0,0;
        q_state << 0,0,0,0,0;


        out_file.open ("youbot_vs.txt");
    }

    ~YoubotVS() {
        arm_velocity_pub.shutdown();
        arm_js_sub.shutdown();
        out_file.close();
    }

    void display_trajectory(const vpImage<unsigned char> &I, const std::vector<vpDot2> &dot)
    {
        static std::vector<vpImagePoint> traj[4];
        for (unsigned int i = 0; i < 4; i++) {
            traj[i].push_back(dot[i].getCog());
        }
        for (unsigned int i = 0; i < 4; i++) {
            for (unsigned int j = 1; j < traj[i].size(); j++) {
                vpDisplay::displayLine(I, traj[i][j - 1], traj[i][j], vpColor::green);
            }
        }
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
            vpImage<unsigned char> I;

            /* CAMERA SETUP */
            vpRealSense2 g;
            rs2::config config;
            config.disable_stream(RS2_STREAM_DEPTH);
            config.disable_stream(RS2_STREAM_INFRARED);
            config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
            g.open(config);

            g.acquire(I);
            vpCameraParameters cam(840, 840, I.getWidth() / 2, I.getHeight() / 2);
            cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
            std::cout << cam << std::endl;


            vpImagePoint germ[Np];
            vpDot2 blob[Np];

            for(int i = 0; i < Np; i++) {
                blob[i].setGraphics(true);
                blob[i].setGraphicsThickness(1);
            }

            std::vector<vpPoint> point;
            std::vector<vpImagePoint> ip;
            vpFeaturePoint p[4], pd[4];

            point.push_back(vpPoint(-L, -L, 0));
            point.push_back(vpPoint( L, -L, 0));
            point.push_back(vpPoint( L,  L, 0));
            point.push_back(vpPoint(-L,  L, 0));

            vpDisplayX d;
            d.init(I, 20, 40, "Original");

            vpHomogeneousMatrix cMo, cdMo, wMc, wMo;

            std::vector<vpImagePoint> ipd;
            double w, h;
            h = I.getWidth()/2;
            w = I.getHeight()/2;
            ipd.push_back(vpImagePoint(w-100, h-100));
            ipd.push_back(vpImagePoint(w-100, h+100));
            ipd.push_back(vpImagePoint(w+100, h+100));
            ipd.push_back(vpImagePoint(w+100, h-100));
            // compute desired pose
            computePose(point, ipd, cam, true, cdMo);


            vpServo task;
            task.setServo(vpServo::EYEINHAND_CAMERA);
            task.setInteractionMatrixType(vpServo::CURRENT);
            task.setLambda(0.1);

//            Vector6d s = ks.forward(q_state, 0, 5);
//            vpPoseVector wPe(s(0), s(1), s(2), s(3), s(4), s(5));
//            vpHomogeneousMatrix wMe(wPe);
//            vpPoseVector ePc;
//            ePc[0] = -0.0144741; ePc[1] = -0.0556421; ePc[2] = 0.0460595;
//            ePc[3] = -2.21359; ePc[4] = -0.227854; ePc[5] = 0.772322;
//            vpHomogeneousMatrix eMc(ePc);
//            task.set_cVe(eMc);

            bool init_cv = true;   // initialize tracking and pose computation
            bool learn = true;
            bool isTrackingLost = false;
            bool send_velocities = false;
            bool final_quit = false;

            ros::Rate R(RATE);
            int k = 0;
            double t;
            static double t_init_servo = vpTime::measureTimeMs();

            while(nh.ok() && !final_quit) {
                try {
                    double t_start = vpTime::measureTimeMs();
                    t = vpTime::measureTimeMs();
                    g.acquire(I);
                    vpDisplay::display(I);

                    std::stringstream ss;
                    ss << "Left click to " << (send_velocities ? "stop the robot" : "servo the robot") << ", right click to quit.";
                    vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::white);

                    vpColVector v_c(6);

                    if (!isTrackingLost) {
                        if (!learn) {
                            vpDisplay::displayText(I, vpImagePoint(80, 20), "Tracking is ok!", vpColor::green);
                            try {

                                std::vector<vpImagePoint> ip(Np);
                                for(int i = 0; i < Np; i++) {
                                    std::stringstream ss;
                                    ss << "p" << i;
                                    vpDisplay::displayText(I, blob[i].getCog(), ss.str(), vpColor::white);

                                    blob[i].track(I);
                                    ip[i] = blob[i].getCog();

                                    //std::cout << ip[i] << std::endl;

                                    vpFeatureBuilder::create(p[i], cam, blob[i].getCog());
                                    vpColVector cP;
                                    point[i].changeFrame(cMo, cP);
                                    p[i].set_Z(cP[2]);
                                }

                                computePose(point, ip, cam, init_cv, cMo);
                                if (init_cv) init_cv = false;


                                vpDisplay::displayFrame(I, cdMo, cam, opt_square_width, vpColor::none, 1);
                                vpDisplay::displayFrame(I, cMo, cam, opt_square_width, vpColor::none, 2);

                                /*
                                    +cMo

                                */
                                vpColVector v_c = task.computeControlLaw();
                                //display_trajectory(I, blob);

                                std::cout << v_c.transpose() << '\n';

                                vpDisplay::displayArrow(I, vpImagePoint(w,h), vpImagePoint(w+v_c[0]*1000, h+v_c[1]*1000), vpColor::white);

                                if (!send_velocities) {
                                    v_c = 0;
                                }

                                for(int k = 0; k < 6; k++) {
                                    v(k) = v_c[k];
//                                    if (k >= 2) {
//                                        v(k) = 0;
//                                    }
                                }

                                /* TF publish*/
                                vpRotationMatrix R;
                                vpRotationMatrix Rd;
                                for(int i = 0; i < 3; i++) {
                                    for(int j = 0; j < 3; j++) {
                                        R[i][j] = cMo[i][j];
                                        Rd[i][j] = cdMo[i][j];
                                    }
                                }

                                vpQuaternionVector qv(R);
                                vpQuaternionVector qvd(R);
                                static tf::TransformBroadcaster br;
                                tf::Transform transform;
                                //tf::Quaternion q(-0.7,0,0,0.7);
                                tf::Quaternion q(0,0,0,1);
                                transform.setOrigin( tf::Vector3(-0.085,0.015,0) );
                                transform.setRotation(q);
                                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "arm_link_5", "camera_link"));

                                transform.setOrigin( tf::Vector3(cMo[0][3], cMo[1][3], cMo[2][3]) );
                                transform.setRotation(tf::Quaternion(qv.x(), qv.y(), qv.z(), qv.w()));
                                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "marker"));

                                transform.setOrigin( tf::Vector3(cdMo[0][3], cdMo[1][3], cdMo[2][3]) );
                                transform.setRotation(tf::Quaternion(qvd.x(), qvd.y(), qvd.z(), qvd.w()));
                                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "arm_link_5", "camera_des_link"));

                            }
                            catch(...) {
                                std::cout << "Computer vision failure.\r";
//                                isTrackingLost = true;
                            }
                        } else {
                            if (vpDisplay::getClick(I, germ[k], false)) {
                                blob[k].initTracking(I, germ[k]);

                                vpFeatureBuilder::create(pd[k], point[k]);
                                vpFeatureBuilder::create(p[k], cam, blob[k].getCog());

                                task.addFeature(p[k], pd[k]);

                                k++;
                            }
                            if (k == Np) {
                                learn = false;
                            }
                        }
                    } else {
                        std::stringstream ss;
                        ss << "Tracking lost! Finding blobs... Qty: " << 4;
                        vpDisplay::displayText(I, vpImagePoint(60, 20), ss.str(), vpColor::red);
                    }

                    vpDisplay::flush(I);

                    ss.str("");
                    ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
                    vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::white);
                    vpDisplay::flush(I);

                    vpMouseButton::vpMouseButtonType button;
                    if (vpDisplay::getClick(I, button, false)) {
                        switch (button) {
                            case vpMouseButton::button1:
                                send_velocities = !send_velocities;
                                std::cout << "Send velocities mode changed." << std::endl;
                                break;

                            case vpMouseButton::button2:
                                init_cv = true;   // initialize tracking and pose computation
                                learn = true;
                                isTrackingLost = false;
                                send_velocities = false;
                                k = 0;
                                std::cout << "Reseted." << std::endl;
                                break;

                            case vpMouseButton::button3:
                                v_c = 0;
                                final_quit = true;
                                std::cout << "Quited." << std::endl;
                                break;

                            default:
                                break;
                        }
                    }

                } catch (...) {
                    isTrackingLost = true;
                    std::cout << "Tracking lost. Finding blobs..    .\r";
                }

                ros::spinOnce();
                R.sleep();
            }
            task.kill();
        } catch (const vpException &e) {
            std::stringstream ss;
            ss << "vpException: " << e;
            std::cout << ss.str() << std::endl;
        }
    }



    void workCalibration() {

        try {
            vpImage<unsigned char> I;

            vpRealSense2 g;
            rs2::config config;
            config.disable_stream(RS2_STREAM_DEPTH);
            config.disable_stream(RS2_STREAM_INFRARED);
            config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
            g.open(config);
            g.acquire(I);

            unsigned int width = I.getWidth();
            unsigned int height = I.getHeight();

            std::cout << "Image size: " << width << " x " << height << std::endl;
            // Save intrinsics
            vpCameraParameters cam;
            vpXmlParserCamera xml_camera;
            cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
            xml_camera.save(cam, "camera.xml", "Camera", width, height);

            vpDisplayX dc(I, 10, 10, "Color image");

            ros::Rate R(RATE);

            bool end = false;
            unsigned cpt = 0;
            while (nh.ok() && !end) {
                g.acquire(I);

                vpDisplay::display(I);

                vpDisplay::displayText(I, 15, 15, "Left click to acquire data", vpColor::red);
                vpDisplay::displayText(I, 30, 15, "Right click to quit", vpColor::red);
                vpMouseButton::vpMouseButtonType button;
                if (vpDisplay::getClick(I, button, false)) {
                    if (button == vpMouseButton::button1) {
                        cpt ++;

                        Vector6d s = ks.forward(q_state, 0, 5);
                        vpPoseVector fPe(s(0), s(1), s(2), s(3), s(4), s(5));
                        std::cout << q_state << std::endl;
                        std::cout << s.transpose() << std::endl;

                        std::stringstream ss_img, ss_pos;

                        ss_img << "image-" << cpt << ".png";
                        ss_pos << "pose_fPe_" << cpt << ".yaml";
                        std::cout << "Save: " << ss_img.str() << " and " << ss_pos.str() << std::endl;
                        vpImageIo::write(I, ss_img.str());
                        fPe.saveYAML(ss_pos.str(), fPe);
                    } else if (button == vpMouseButton::button3) {
                        end = true;
                    }
                }
                vpDisplay::flush(I);

                ros::spinOnce();
                R.sleep();
            }
        } catch (const vpException &e) {
            std::cerr << "RealSense error " << e.what() << std::endl;
        } catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "youbot_vs_node");

    YoubotVS ybvs;
    ybvs.work();
    //ybvs.workCalibration();

    return 0;
}
