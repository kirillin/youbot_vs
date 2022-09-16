/**
 *
 *  author: kirillin
 *  created: 20.05.19.
**/

#include <cmath>
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

    std::ofstream file_joint_state;
    std::ofstream file_vs;

    double l = 0;
    double vcx = 0, vcy = 0, vcz = 0;


public:
    YoubotVS(double _l, double _vcx, double _vcy, double _vcz): l(_l), vcx(_vcx), vcy(_vcy), vcz(_vcz) {
        ROS_INFO("Youbot VS initialized.");

        file_joint_state.open("/home/kirix/ros_vs_ws/plots/joint_state.txt");
        file_vs.open("/home/kirix/ros_vs_ws/plots/vs.txt");

        arm_velocity_pub = nh.advertise<brics_actuator::JointVelocities>("arm_1/arm_controller/velocity_command", 1);
        arm_js_sub = nh.subscribe("joint_states", 10, &YoubotVS::js_callback, this);

        VectorNd v1;
        v1 << 0.033, 0.155, 0.135, 0.0, 0.0;
        VectorNd v2;
        v2 << M_PI / 2.0, 0.0, 0.0, M_PI / 2.0, 0.0;
        VectorNd v3;
        v3 << 0.147, 0.0, 0.0, 0.0, 0.218;
        VectorNd v4;
        v4 << M_PI * 169.0 / 180.0, M_PI * 65.0 / 180.0 + M_PI / 2, -M_PI * 146.0 / 180.0, M_PI * 102.5 / 180.0 +
                                                                                           M_PI / 2, M_PI * 167.5 /
                                                                                                     180.0;
        Kinematics ks_(v1, v2, v3, v4);
        ks = ks_;

        q_state << 0, 0, 0, 0, 0;
        dq_state << 0, 0, 0, 0, 0;
        tau_state << 0, 0, 0, 0, 0;

        v_des << 0, 0, 0, 0, 0, 0;

        START_TIME = ros::Time::now().toSec();


    }

    ~YoubotVS() {
        file_joint_state.close();
        file_vs.close();

        arm_velocity_pub.shutdown();
        arm_js_sub.shutdown();
    }

    void computePose(std::vector <vpPoint> &point, const std::vector <vpImagePoint> &ip,
                     const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo) {
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

    /*
     * Joint state update and write to file.
     */
    void js_callback(const sensor_msgs::JointState &msg) {
        double t = ros::Time::now().toSec() - START_TIME;

        for (int i = 0; i < 5; i++) {
            q_state(i) = msg.position[i];
            dq_state(i) = msg.velocity[i];
            tau_state(i) = msg.effort[i];
        }

        // write to file current joint state
        file_joint_state << q_state.transpose() << ' '
                         << dq_state.transpose() << ' '
                         << tau_state.transpose() << ' '
                         << t << '\n';
    }

    void publish(Matrix<double, 5, 1> dq) {
        brics_actuator::JointVelocities arm_velocities;
        vector<brics_actuator::JointValue> point;

        std::stringstream joint_name;
        point.resize(5);

        for (int i = 0; i < 5 /*+2*/; i++) {
            joint_name.str("");
            joint_name << "arm_joint_" << (i + 1);
            point[i].joint_uri = joint_name.str();
            point[i].value = dq(i);
            point[i].unit = boost::units::to_string(boost::units::si::radian_per_second);
        }
        arm_velocities.velocities = point;
        arm_velocity_pub.publish(arm_velocities);
    }

    /*
     * Pseudo inverse image jacobian.
     */
    void pinv(Matrix<double, 8, 6>& L, Matrix<double, 6, 8>& pinvL, double alpha0 = 0.001, double w0 = 0.0001) {
        double w = 0, alpha = 0;

        double detL = (L * L.transpose()).determinant();
        if (detL < 1.0e-10) {
            w = 1.0e-5;
        } else {
            w = sqrt(detL);
        }

        if (w >= w0) {
            alpha = 0;
        } else {
            alpha = alpha0 * (1.0 - w / w0) * (1 - w / w0);
        }

        // 6x8 = 6x8 * (8x6 * 6x8)
        pinvL = L.transpose() * (L * L.transpose() - alpha * MatrixXd::Identity(8, 8)).inverse();
    }

    void compute_error(std::vector<vpImagePoint>& ip, std::vector<vpImagePoint>& ipd, Matrix<double, 8, 1>& error) {
        error = Matrix<double, 8, 1>::Zero();
        for (int i = 0; i < Np; i++) {
            // error in image space for each point
            double ex = ip[i].get_u() - ipd[i].get_u();
            double ey = ip[i].get_v() - ipd[i].get_v();
            error(0 + 2 * i) = ex;
            error(1 + 2 * i) = ey;
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

            std::clog << cam << std::endl;

            vpImagePoint germ[Np];
            vpDot2 blob[Np];

            for (int i = 0; i < Np; i++) {
                blob[i].setGraphics(true);
                blob[i].setGraphicsThickness(1);
            }

            vpDisplayX d;
            d.init(I, 20, 40, "Original");

            std::vector<vpImagePoint> ip;
            std::vector<vpImagePoint> ipd;

            double w, h; h = I.getWidth() / 2; w = I.getHeight() / 2;
            //ipd.push_back(vpImagePoint(cam.get_v0() - 100, cam.get_u0() - 100)); // left top
            //ipd.push_back(vpImagePoint(cam.get_v0() - 100, cam.get_u0() + 100)); // right top
            //ipd.push_back(vpImagePoint(cam.get_v0() + 100, cam.get_u0() + 100)); // right bottom
            //ipd.push_back(vpImagePoint(cam.get_v0() + 100, cam.get_u0() - 100)); // left bottom

            ipd.push_back(vpImagePoint(300.954, 326.775)); // left top
            ipd.push_back(vpImagePoint(299.619, 408.009)); // rifht top                        
            ipd.push_back(vpImagePoint(382.439, 410.426)); // right bottom
			ipd.push_back(vpImagePoint(382.761, 328.593)); // right top
            
            

            
            std::vector<vpPoint> point;
            point.push_back(vpPoint(-L, -L, 0));
            point.push_back(vpPoint( L, -L, 0));
            point.push_back(vpPoint( L,  L, 0));
            point.push_back(vpPoint(-L,  L, 0));

            vpHomogeneousMatrix cMo, cdMo;
            //computePose(point, ipd, cam, true, cdMo);

            bool init_cv = true;   // initialize tracking and pose computation
            bool learn = true;
            bool isTrackingLost = false;
            bool send_velocities = false;
            bool final_quit = false;

            int k = 0;
            double t;
            double loop_start_time;

            double Z = 1;
            double f = 0.00193; // fo            cal length of RSD435
            double fx = cam.get_px();
            double fy = cam.get_py();
            double rhox = f / fx;
            double rhoy = f / fy;
            Matrix<double, 2, 2> diagrho;
            diagrho << rhox, 0, 0, rhoy;

            double lambda = l;
            double lambda_0 = vcx;      // 4
            double lambda_inf = vcy;    // 1
            double lambda_0l = vcz;     // 600
            double mu = l;          // 4

            Matrix<double, 3, 3> Rzero = Matrix<double, 3, 3>::Zero();
            Matrix<double, 3, 3> eRc;
            // roatate to pi/2 around Z-axis
//            eRc << 0, 1, 0,
//                  -1, 0, 0,
//                   0, 0, 1;
//            eRc << 0, 0, 1,
//                  -1, 0, 0,
//                   0, -1, 0;
            eRc <<  0,  0, -1,
                    1,  0,  0,
                    0, -1,  0;


            MatrixXd eVc(6, 6);
            eVc << eRc, Rzero, Rzero, eRc;


            ros::Rate R(RATE);
            while (nh.ok() && !final_quit) {
                loop_start_time = ros::Time::now().toSec();
                t = ros::Time::now().toSec() - START_TIME;
                Matrix<double, 5, 1> dq;

                try {
                    g.acquire(I);
                    vpDisplay::display(I);

                    std::stringstream ss; ss << (send_velocities ? "Click to START" : "Click to STOP");
                    vpDisplay::displayText(I, 20, 20, ss.str(), vpColor(254,188,0));

                    if (!learn) {
                        vpDisplay::displayText(I, vpImagePoint(80, 20), "Tracking is ok!", vpColor::green);
                        try {

                            Matrix<double, 8, 1> error;
                            Matrix<double, 8, 6> L = Matrix<double, 8, 6>::Zero();
                            Matrix<double, 6, 8> pinvL;

                            Matrix<double, 6, 5> J;
                            Matrix<double, 5, 6> pinvJ;


                            Matrix<double, 6, 1> v_c;
                            Matrix<double, 6, 1> v_c0;
                            Matrix<double, 6, 1> v_e;



                            std::vector <vpImagePoint> ip(Np);

							std::cout << "***" << std::endl;
                            for (int i = 0; i < Np; i++) {
                                blob[i].track(I);
                                ip[i] = blob[i].getCog();

                                if (!init_cv) {
                                    vpColVector cP;
                                    point[i].changeFrame(cMo, cP);
                                    Z = cP[2]; //FIXME: can be estimated from square dims???
//                                    std::cout << "Z" << i << ": " << Z << std::endl;
                                }

                                double x = (ip[i].get_u() - cam.get_u0()) * rhox / f;
                                double y = (ip[i].get_v() - cam.get_v0()) * rhoy / f;

								std::cout << ip[i].get_u() << " " << ip[i].get_v() << std::endl;

                                Matrix<double, 2, 6> Lx;
                                Lx << 1 / Z,      0, -x / Z, -x * y,        (1 + x * x), -y,
                                        0,    1 / Z, -y / Z, -(1 + y * y),        x * y, x;
                                Lx = -1 * f * diagrho * Lx;

                                // Copy one point-matrix to full image-jacobian
                                for (int j = 0; j < 6; j++) {
                                    for (int k = 0; k < 2; k++) {
                                        L(k + 2 * i, j) = Lx(k, j);
                                    }
                                }

                                if (DEBUG) {
                                    std::stringstream ss; ss << i;
                                    vpDisplay::displayText(I, blob[i].getCog(), ss.str(), vpColor::white); // number of point
                                    vpDisplay::displayLine(I, ipd[i], ip[i], vpColor(254,188,0), 1); // line between current and desired points
                                }
                            }

                            // Image jacobian
                            pinv(L, pinvL);
                            compute_error(ip, ipd, error);

                            // Manipulator jacobian
                            ks.get_jacobn(q_state, J);
                            J = -1 * J;
                            ks.pinv(J, pinvJ);

                            computePose(point, ip, cam, init_cv, cMo);
                            if (init_cv) {
                                init_cv = false;
                                v_c0 = pinvL * error; // compute initial velocity
                            }

                            v_c = pinvL * error;


                            // SIMPLE GAIN
//                            v_c(0) = vcx;
//                            v_c(1) = vcy;
//                            v_c(2) = vcz;
//                            v_c(3) = 0;
//                            v_c(4) = 0;
//                            v_c(5) = 0;

//                            v_c = lambda * v_c;     // Camera frame velocity

                            // ADAPTIVE GAIN
                            lambda = (lambda_0 - lambda_inf) * exp( - lambda_0l * v_c.lpNorm<Infinity>() / (lambda_0 - lambda_inf)) + lambda_inf;
                            //lambda = (lambda_0 - lambda_inf) * exp( - lambda_0l * error.lpNorm<Infinity>() / (lambda_0 - lambda_inf)) + lambda_inf;
                            v_c = lambda * v_c;// - lambda * v_c0 * exp( - mu * t);

                            std::cout << lambda << "\t" << v_c << std::endl;

                            v_c(2) = 0; // Z-axis velocity to zero
                            v_e = eVc * v_c;    // N-frame velocity



//                            v_e(0) = vcx;
//                            v_e(1) = vcy;
//                            v_e(2) = vcz;
//                            v_e(3) = 0;
//                            v_e(4) = 0;
//                            v_e(5) = 0;


                            dq = pinvJ * v_e;
                            dq(4) = 0;

                            file_vs << error.transpose() << ' '
                                    << v_c.transpose() << ' '
                                    << v_e.transpose() << ' '
                                    << dq.transpose() << ' '
                                    << lambda << ' '
                                    << t << '\n';


                            if ((!send_velocities) || (dq.lpNorm<Infinity>() <= 0.015)) {
                                for(int j = 0; j < 5; j++) {
                                    dq(j) = 0;
                                }
                                v_c0 = pinvL * error; // compute initial velocity
                                START_TIME = ros::Time::now().toSec();
                            }
//                            dq(4) = 0;
                            publish(dq);

                            vpDisplay::displayArrow(I, vpImagePoint(cam.get_v0(), cam.get_u0()),
                                                    vpImagePoint(cam.get_v0(), cam.get_u0() + v_c[0] * 10000),
                                                    vpColor::red);
                            vpDisplay::displayArrow(I, vpImagePoint(cam.get_v0(), cam.get_u0()),
                                                    vpImagePoint(cam.get_v0() + v_c[1] * 10000, cam.get_u0()),
                                                    vpColor::green);

                            vpDisplay::displayArrow(I, vpImagePoint(cam.get_v0(), cam.get_u0()),
                                                    vpImagePoint(cam.get_v0() + v_c[1] * 10000,
                                                                 cam.get_u0() + v_c[0] * 10000), vpColor(254,188,0));

                            if (DEBUG) {
                                vpDisplay::displayFrame(I, cdMo, cam, opt_square_width, vpColor::none, 1);
                                vpDisplay::displayFrame(I, cMo, cam, opt_square_width, vpColor::none, 2);
                                // write error to file
                            }

                        } catch (...) {
                            std::cout << "Computer vision failure.\n";
//                                isTrackingLost = true;
                        }
                    } else {
                        if (vpDisplay::getClick(I, germ[k], false)) {
                            blob[k].initTracking(I, germ[k]);
                            k++;
                        }
                        if (k == Np) {
                            learn = false;
                            k = 0;
                        }
                    }

                    vpDisplay::flush(I);

                    ss.str("");
                    ss << "Loop time: " << ros::Time::now().toSec() - loop_start_time << " ms";
                    vpDisplay::displayText(I, 40, 20, ss.str(), vpColor(254, 188, 0));
                    vpDisplay::flush(I);

                    vpMouseButton::vpMouseButtonType button;
                    if (vpDisplay::getClick(I, button, false)) {
                        for(int j = 0; j < 5; j++) {
                            dq(j) = 0;
                        }
                        publish(dq);
                        switch (button) {
                            case vpMouseButton::button1:
                                send_velocities = !send_velocities;
                                std::cout << "Send velocities mode changed." << std::endl;
                                break;

                            case vpMouseButton::button2:
                                init_cv = true;   // initialize tracking and pose computation
                                learn = true;
                                send_velocities = false;
                                std::cout << "Reseted." << std::endl;
                                break;

                            case vpMouseButton::button3:
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
                    for(int j = 0; j < 5; j++) {
                        dq(j) = 0;
                    }
                    publish(dq);
                }

                ros::spinOnce();
                R.sleep();
            }
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
                        cpt++;

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

int main(int argc, char **argv) {
    double lambda = std::atof(argv[1]);

    double vcx = std::atof(argv[2]);
    double vcy = std::atof(argv[3]);
    double vcz = std::atof(argv[4]);

    ros::init(argc, argv, "youbot_vs_node");

    YoubotVS ybvs(lambda, vcx, vcy, vcz);
    ybvs.work();
    //ybvs.workCalibration();

    return 0;
}
