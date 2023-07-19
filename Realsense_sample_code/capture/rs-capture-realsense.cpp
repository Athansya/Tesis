// License: Apache 2.0 See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

// Caputre Example demonstrates how to
// caputre data and color video streams and rendter them to the screen
int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Declare data colorizer for pretty visualization of data data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    pipe.start();
    
    // OpenCV settings
    const auto window_name = "Display Image";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    while (true)
    {
        // Key for controlling window
        char key = (char)cv::waitKey(10);
        
        // Image adquisition
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame().apply_filter(printer).apply_filter(color_map);
        rs2::frame image = data.get_color_frame();

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // OpenCV Image object
        cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        // Show image
        cv::imshow(window_name, image);

        // Closing condition
        if ((key == 'q') | (key == 'Q') | (key == 27)) break;
    }
    // Cleaning 
    cv::destroyAllWindows();

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n   " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
