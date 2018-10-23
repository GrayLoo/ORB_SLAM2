#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_laptop path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    auto system_start = std::chrono::steady_clock::now();
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cout << "Failed to open webcam." << std::endl;
        return 1;
    }

    // Main loop
    while (true)
    {
        // Read image from file
        cv::Mat im;
        cap >> im;
        auto start = std::chrono::steady_clock::now();
        double tframe = std::chrono::duration_cast<std::chrono::duration<double>>(start - system_start).count();
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);
        auto end = std::chrono::steady_clock::now();
        auto tracking_time = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();
        std::cout << "\r--- Tracking took " << tracking_time << std::flush;
        usleep(1e4);
    }

    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}