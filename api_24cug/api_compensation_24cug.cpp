#include <iostream>
#include "uvccamera.h"
#include "see3cam_24cug.h"
#include "v4l2-api.h"

bool parser(int argc, char *argv[], std::string &device, bool &operation, unsigned int &compensation)
{
    if(argc >= 2)
    {
        device = std::string(argv[1]);

        if( (argc == 3) && (std::string(argv[2]) == "-g") )
        {
            operation = false;
            return true;
        }
        
        if( (argc == 4) && (std::string(argv[2]) == "-s") )
        {
            operation = true;
            compensation = std::stoi(argv[3]);
            return true;
        }
    }

    return false;
}

int main(int argc, char *argv[])
{
    unsigned int compensation = 6666;
    std::string device = "/dev/video0";
    std::string deviceName = "See3CAM_24CUG";
    
    bool operation = 0;

    if( !parser(argc, argv, device, operation, compensation) )
    {
        std::cerr << "Parameter error" << std::endl;
        std::cerr << "sudo ./api_compensation_24cug /dev/videoX -s compensation_value\n";
        std::cerr << "sudo ./api_compensation_24cug /dev/videoX -g\n";
        return -1;
    }

    // std::cout << "Device: " << device << " Op: " << operation << " Compensation: " << compensation << std::endl;

    uvccamera uvcam;

    std::vector<std::string> deviceNames, devicePaths;
    struct v4l2_capability m_querycap;
    std::string deviceNode;
    v4l2 v4l2_api;

    uvcam.enumerateDevices(&deviceNames, &devicePaths);

    uvcam.findEconDevice("video4linux");
    uvcam.findEconDevice("hidraw");

    if(v4l2_api.open(device, false))
    {
        if(v4l2_api.querycap(m_querycap))
        {
            deviceNode = (char*) m_querycap.bus_info;
        }
    }
    v4l2_api.close();


    uvcam.getDeviceNodeName(deviceNode);

    uvcam.exitExtensionUnit();
    bool hidInit = uvcam.initExtensionUnit(deviceName);

    See3CAM_24CUG extcam;

    if( operation )
    {
        unsigned int compExp0 = 0, compExp1 = 0;
        if( !extcam.getExposureCompensation(compExp0) )
        {
            std::cerr << "Get error" << std::endl;
            return -1;
        }

        if ( !extcam.setExposureCompensation(compensation) )
        {
            std::cerr << "Set error" << std::endl;
            return -1;
        }

        if( !extcam.getExposureCompensation(compExp1) )
        {
            std::cerr << "Get error" << std::endl;
            return -1;
        }

        std::cout << "Compensation from: " << compExp0 << " to: " << compExp1 << std::endl;
    }
    else
    {
        unsigned int compExp0 = 0;
        if( !extcam.getExposureCompensation(compExp0) )
        {
            std::cerr << "Get error" << std::endl;
            return -1;
        }
        std::cout << compExp0 << std::endl;
    }

    return 0;
}