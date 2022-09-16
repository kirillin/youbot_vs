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
            /* CAMERA SETUP */
            // Get config from the RS camera
            rs2::config rs_config;
            rs_config.disable_all_streams();
            rs_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);

            // New camera object with loaded config
            vpRealSense2 rs;
            rs.open(rs_config);

            // Get a frame
            vpImage<unsigned char> Image;
            rs.acquire(Image);

            vpCameraParameters camera_params(640, 480, Image.getWidth() / 2, Image.getHeight() / 2);
            camera_params = rs.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);

            /* POINTS SETUP */
            int Np = 4;

            // The pose container
            vpHomogeneousMatrix cMo;

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

            /* VISP SETUP */
            vpDisplayX d;
            d.init(Image, 20, 40, "Original");

            for(int i = 0; i < N; i++) {
                blob[i].setGraphics(true);
                blob[i].setGraphicsThickness(1);
            }

            double k = 0;
            double t = 0;
            bool isPointsSelected = false;
            double rate = 30
            ros::Rate R(rate);

            // Temp
            vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);

            vpServo task;
            task.setServo(vpServo::EYEINHAND_CAMERA);
            task.setInteractionMatrixType(vpServo::CURRENT);
            task.setLambda(0.5);

            vpFeaturePoint p[Np], pd[Np];

            for (unsigned int i = 0; i < Np; i++) {
                point[i].track(cdMo);
                vpFeatureBuilder::create(pd[i], point[i]);

                dot[i].setGraphics(true);
                dot[i].initTracking(Image);
                vpDisplay::flush(Image);
                vpFeatureBuilder::create(p[i], camera_params, dot[i].getCog());
                task.addFeature(p[i], pd[i]);
            }

            vpHomogeneousMatrix wMc, wMo;
            wMc = eye(4,4); // !!!!!!!!
            cwMo = wMc * cMo;


            while(nh.ok()) {
                t = vpTime::measureTimeMs();
                g.acquire(Image);
                vpDisplay::display(Image);

                if (isPointsSelected) {
                    vpDisplay::displayText(I, vpImagePoint(10, 10), "Tracking is ok!", vpColor::white);

                    try {
                        for(int i = 0; i < Np; i++) {
                            std::stringstream ss;
                            ss << "p" << i;
                            vpDisplay::displayText(Image, blob[i].getCog(), ss.str(), vpColor::white);
                            blob[i].track(Image);
                            ip[i] = blob[i].getCog();

                        }

                        // Compute camera pose and display
                        computePose(point, ip, camera_params, true, cMo);
                        vpDisplay::displayFrame(I, cMo, cam, desired_square_width, vpColor::none, 3);

                        robot.getPosition(wMc);
                        cMo = wMc.inverse() * wMo;

                        rs.acquire(Image, cMo);
                        vpDisplay::display(Image);

                        for (unsigned int i = 0; i < Np; i++) {
                            dot[i].track(Image);
                            vpFeatureBuilder::create(p[i], camera_params, dot[i].getCog());
                            vpColVector cP;
                            point[i].changeFrame(cMo, cP);
                            p[i].set_Z(cP[2]);
                        }

                        vpColVector v = task.computeControlLaw();


                        vpServoDisplay::display(task, camera_params, Image, vpColor::green, vpColor::red);

                    } catch(...) {
                        std::cout << "Pints was lost!" << std::endl;
                        isPointsSelected = false;
                    }

                } else {
                    if (vpDisplay::getClick(Image, germ[k], false)) {
                        blob[k].initTracking(Image, germ[k]);
                        k++;
                    }
                    if (k == Np) {
                        isPointsSelected = true;
                    }
                }

                vpDisplay::flush(I);

                // click to quit
                if (vpDisplay::getClick(I, false))
                    break;

//                vpTime::wait(robot.getSamplingTime() * 1000);
//                t = t + robot.getSamplingTime();

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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "youbot_vs_node");

    YoubotVS ybvs;
    ybvs.work();

    return 0;
}
z