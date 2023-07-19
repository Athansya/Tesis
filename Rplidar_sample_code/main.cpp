#include <iostream>
#include <vector>
#include <rplidar.h> // include the RPLidar SDK

using namespace rp::standalone::rplidar; // RPLidar 

int main(int argc, char* argv[])
{
    RPlidarDriver* lidar = RPlidarDriver::CreateDriver();
    // Connect to the RPLidar S1
    const char * devicePort = "/dev/ttyUSB0"; // or "COM3" for Windows
    u_result result = lidar->connect(devicePort, 256000);

    if (IS_FAIL(result))
    {
        std::cerr << "Failed to connect to RPLidar S1." << std::endl;
        return 1;
    }

    if (lidar->isConnected())
        std::cout << "Connection established!\n" << std::endl;

    
    // Sensor health and specs
    rplidar_response_device_health_t healthInfo;
    result = lidar->getHealth(healthInfo);
    
    std::cout << "***** Health *****" << std::endl;
    std::cout << "Status: " << static_cast<int>(healthInfo.status) << std::endl;
    std::cout << "Error code: " << healthInfo.error_code << std::endl;

    rplidar_response_device_info_t deviceInfo;
    result = lidar->getDeviceInfo(deviceInfo);

    std::cout << "***** Sensor information *****" << std::endl;
    std::cout << "Model: " << deviceInfo.model << std::endl;
    std::cout << "Firmware Version: " << deviceInfo.firmware_version << std::endl;
    std::cout << "Hardware Version: " << deviceInfo.hardware_version  << "\n" << std::endl;

    // Get and show supported scan modes
    std::vector<RplidarScanMode> scanModes;
    lidar->getAllSupportedScanModes(scanModes);

    std::cout << "***** Supported scan modes: " << scanModes.size() << " *****" << std::endl;
    // List scan modes
    for (auto it = scanModes.begin(); it != scanModes.end(); ++it)
    {
            std::cout << "Mode name: " << it[0].scan_mode << std::endl;
            std::cout << "Sampling duration: " << it[0].us_per_sample << " microseconds per sample" << std::endl;
            std::cout << "Max distance: " << it[0].max_distance << std::endl;
            std::cout << "Ans Type: " << it[0].ans_type << std::endl;
            std::cout << std::endl;

    }
    // Typical Scan Mode
    // I think it indicates the chosen mode number from the previous list
    _u16 outMode;  
    lidar->getTypicalScanMode(outMode);
    std::cout << "***** Typical Scan Mode: " << scanModes[outMode-1].scan_mode << " *****" << std::endl;

    // Let's start the motor
    std::cout << "\nStarting the motor..." << std::endl;
    lidar->startMotor();  // Not needed for S1 model, may add as safety net

    // Start scan with dense mode
    bool force = true;
    bool useTypicalScan = true;
    std::cout << "Starting dense scan mode" << std::endl;

    // lidar->startScan(force, useTypicalScan);
    lidar->startScanExpress(force, 1);
    
    // Wait and grab a complete 0-360 degree scan data previously received
    rplidar_response_measurement_node_hq_t nodes[8192];  // Buffer
    size_t nodeCount = sizeof(nodes) / sizeof(rplidar_response_measurement_node_hq_t);
    std::cout << "Saving scanned data..." << std::endl;
    result = lidar->grabScanDataHq(nodes, nodeCount); // Saves data

    if (IS_FAIL(result))
    {
        std::cerr << "Failed to get scan data" << std::endl;
        lidar->disconnect();
        delete lidar;  // Equal to RPlidarDriver::DisposeDriver();
        return 1;
    }
    std::cout << "Scanned data saved!" << std::endl;

    // Get frequency
    float frequency;
    lidar->getFrequency(scanModes[1], nodeCount, frequency);
    std::cout << "Frequency: " << frequency << " kHz" << std::endl;  // kHz
    
    std::cout << "Ending default scan mode" << std::endl;

    std::cout << "Stopping motor..." << std::endl;
    lidar->stopMotor();

    std::cout << "\nProgram finished, closing connection!" <<std::endl;
    lidar->disconnect(); // Frees memory
    //RPlidarDriver::DisposeDriver(lidar); //Should be used to free the memory, but ends up in segmentation fault

    delete lidar;  // Equal to RPlidarDriver::DisposeDriver();

    return 0;
}
