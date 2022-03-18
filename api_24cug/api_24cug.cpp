#include <iostream>
#include "uvccamera.h"
#include "see3cam_24cug.h"
#include "v4l2-api.h"

int main(int argc, char *argv[])
{
    unsigned int compensation = 6666;

    if(argc == 2)
    {
        compensation = std::stoi(argv[1]);
    }
    else
    {
        std::cerr << "Faltando parametros\r\n sudo ./api_24cug 600\n";
        return -1;
    }

    uvccamera uvcam;

    std::vector<std::string> deviceNames, devicePaths;
    struct v4l2_capability m_querycap;
    std::string deviceNode;
    v4l2 v4l2_api;

    uvcam.enumerateDevices(&deviceNames, &devicePaths);

    for(auto &i : deviceNames) 
        std::cout << "deviceNames = " << i << std::endl;

    std::cout << std::endl;

    for(auto &i : devicePaths) 
        std::cout << "devicePaths = " << i << std::endl;

    std::cout << std::endl;

    uvcam.findEconDevice("video4linux");
    uvcam.findEconDevice("hidraw");

    if(v4l2_api.open("/dev/video0",false))
    {
        if(v4l2_api.querycap(m_querycap))
        {
            deviceNode = (char*) m_querycap.bus_info;
        }
    }
    v4l2_api.close();

    std::cout << "deviceNode read: " << deviceNode << std::endl;

    uvcam.getDeviceNodeName(deviceNode);

    std::cout << std::endl;

    std::string deviceName = "See3CAM_24CUG";

    uvcam.exitExtensionUnit();
    bool hidInit = uvcam.initExtensionUnit(deviceName);

    std::cout << "hidInit: " << hidInit << std::endl;

    See3CAM_24CUG extcam;

    extcam.setExposureCompensation(compensation);

    unsigned int compExp = 0;
    if (extcam.getExposureCompensation(compExp))
        std::cout << "getExposureCompensation: " << compExp << std::endl;
    else
        std::cout << "Falhou getExposureCompensation" << std::endl;

    return 0;
}