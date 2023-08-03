#include <librealsense2/rs.hpp>
#include <iostream>

int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    // Declares RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    rs2::config cfg;
    // Enabling IMU
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    // Disabling information not in use
    cfg.disable_stream(RS2_STREAM_DEPTH);
    cfg.disable_stream(RS2_STREAM_COLOR);
    cfg.disable_stream(RS2_STREAM_INFRARED);

    pipe.start();  // Start device

    while (true)
    {
        rs2::frameset frameset = pipe.wait_for_frames();

        // Find and retrieve IMU data
        if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
        {
            rs2_vector accel_sample = accel_frame.get_motion_data();
            std::cout << "Accel: " << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.y << std::endl;
        }

        if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
        {
            rs2_vector gyro_sample = gyro_frame.get_motion_data();
            std::cout << "Gyro: " << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.y << std::endl;
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "Realsense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n   " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

