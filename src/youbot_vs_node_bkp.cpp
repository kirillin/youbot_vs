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


class YoubotVS {

    ros::NodeHandle nh;
    std::ofstream out_file;

public:
    YoubotVS() {
        ROS_INFO("Youbot VS initialized.");
        out_file.open ("youbot_vs.txt");
    }

    ~YoubotVS() {
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

    void work() {
        try {
            vpImage<unsigned char> I;
            vpCameraParameters cam(840, 840, I.getWidth() / 2, I.getHeight() / 2); // Default parameters
            vpRealSense2 g;

            rs2::config config;
            config.disable_all_streams();
            config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);

            g.open(config);
            g.acquire(I);

            cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);

            // The pose container
            vpHomogeneousMatrix cMo;

            std::vector<vpDot2> dot(4);
            std::vector<vpPoint> point;   // 3D coordinates of the points
            std::vector<vpImagePoint> ip; // 2D coordinates of the points in pixels

            double opt_square_width = 0.06;
            double L = opt_square_width / 2.;
            point.push_back(vpPoint(-L, -L, 0));
            point.push_back(vpPoint( L, -L, 0));
            point.push_back(vpPoint( L,  L, 0));
            point.push_back(vpPoint(-L,  L, 0));

            vpDisplayX d;
            d.init(I, 20, 40, "Original");


            int N = 4;
            vpImagePoint germ[N];
            vpDot2 blob[N];

//            std::list<vpDot2> dots_list;    // FIXME dublicate of dot
//            vpImagePoint clicked_point;
            bool isTrackingLost = false;
            bool init_cv = true;   // initialize tracking and pose computation
            double distance_same_blob = 10.; // 2 blobs are declared same if their distance is less than this value


            for(int i = 0; i < N; i++) {
                blob[i].setGraphics(true);
                blob[i].setGraphicsThickness(1);
            }

            int k = 0;
            double t;
            bool learn = true;
            ros::Rate R(30);
            while(nh.ok()) {
                try {
                    t = vpTime::measureTimeMs();
                    g.acquire(I);
                    vpDisplay::display(I);

                    if (!isTrackingLost) {
                        if (!learn) {
                            vpDisplay::displayText(I, vpImagePoint(10, 10), "Tracking is ok!", vpColor::white);
                            try {

                                for(int i = 0; i < N; i++) {
                                    std::stringstream ss;
                                    ss << "p" << i;
                                    vpDisplay::displayText(I, blob[i].getCog(), ss.str(), vpColor::white);
                                    blob[i].track(I);
                                }

                                std::vector<vpImagePoint> ip(N);
                                for (unsigned int i = 0; i < N; i++) {
                                    ip[i] = blob[i].getCog();
                                }
                                computePose(point, ip, cam, init_cv, cMo);
                                vpDisplay::displayFrame(I, cMo, cam, opt_square_width, vpColor::none, 3);
                                if (init_cv) init_cv = false;



                                vpRotationMatrix R;
                                for(int i = 0; i < 3; i++) {
                                    for(int j = 0; j < 3; j++) {
                                        R[i][j] = cMo[i][j];
                                    }
                                }
                                vpQuaternionVector qv(R);

                                static tf::TransformBroadcaster br;

                                tf::Transform transform;

                                tf::Quaternion q(0.7,0,0,0.7);
                                transform.setOrigin( tf::Vector3(0,0,0) );
                                transform.setRotation(q);
                                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_link"));

                                transform.setOrigin( tf::Vector3(cMo[0][3], cMo[1][3], cMo[2][3]) );
                                transform.setRotation(tf::Quaternion(qv.x(), qv.y(), qv.z(), qv.w()));
                                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "marker"));

                                out_file << cMo[0][3] << " "
                                        << cMo[1][3] << " "
                                        << cMo[2][3] << " "
                                        <<  t << " "
                                        <<  vpTime::measureTimeMs() - t << "\n";



                                /*{ // Display estimated pose in [m] and [deg]
                                    vpPoseVector pose(cMo);
                                    std::stringstream ss;
                                    ss << "Translation: " << std::setprecision(5) << pose[0] << " " << pose[1] << " " << pose[2] << " [m]";
                                    vpDisplay::displayText(I, 60, 20, ss.str(), vpColor::white);
                                    ss.str(""); // erase ss
                                    ss << "Rotation tu: " << std::setprecision(4) << vpMath::deg(pose[3]) << " " << vpMath::deg(pose[4]) << " " << vpMath::deg(pose[5]) << " [deg]";
                                    vpDisplay::displayText(I, 80, 20, ss.str(), vpColor::white);
                                }
*/

                            }
                            catch(...) {
                                std::cout << "Computer vision failure." << std::endl;
//                                isTrackingLost = true;
                            }
                        } else {
                            if (vpDisplay::getClick(I, germ[k], false)) {
                                blob[k].initTracking(I, germ[k]);
                                k++;
                            }
                            if (k == 4) {
                                learn = false;
                            }
                        }
                    } else {
                        std::stringstream ss;
                        ss << "Tracking lost! Finding blobs... Qty: " << dot.size();
                        vpDisplay::displayText(I, vpImagePoint(10, 10), ss.str(), vpColor::red);
                    }

                    vpDisplay::flush(I);

                    // click to quit
                    if (vpDisplay::getClick(I, false))
                        break;
                    // vpTime::wait(40);

                } catch (...) {
                    isTrackingLost = true;
                    std::cout << "Tracking lost. Finding blobs...\r";
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "youbot_vs_node");

    YoubotVS ybvs;
    ybvs.work();

    return 0;
}
