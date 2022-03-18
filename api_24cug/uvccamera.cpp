/*
 * uvccamera.cpp -- Filter e-con camera and common features of e-con camera
 * Copyright © 2015  e-con Systems India Pvt. Limited
 *
 * This file is part of Qtcam.
 *
 * Qtcam is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Qtcam is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Qtcam. If not, see <http://www.gnu.org/licenses/>.
 */

#include "uvccamera.h"
#include <iostream>
#include <algorithm>


// for string delimiter 
// https://stackoverflow.com/a/46931770
std::vector<std::string> split (std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

std::string toLower( std::string sourceString)
{
    std::string destinationString;
    destinationString.resize(sourceString.size());

    std::transform(sourceString.begin(),
            sourceString.end(),
            destinationString.begin(),
            ::tolower);
    
    return destinationString;
}

std::multimap<std::string, std::string> uvccamera::cameraMap;
std::multimap<std::string, std::string> uvccamera::serialNumberMap;
// Added by Dhurka - 13th Oct 2016
/**
 * @brief uvccamera::selectedDeviceEnum - This is used to compare camera enum value instead of camera name
 */
CommonEnums::ECameraNames uvccamera::selectedDeviceEnum;
std::string uvccamera::hidNode;
// Modified by Dhurka
std::string uvccamera::openNode;

int uvccamera::hid_fd;
libusb_device_handle *uvccamera::handle;
// Added by Dhurka - 14th Oct 2016
/**
 * @brief econCameraVid - to avoid hard coded value checking in findEconDevice()
 */
const std::string econCameraVid = "2560";
const std::string ascellaCameraVid = "04b4";

uvccamera::uvccamera()
{
    // Added by Dhurka - 13th Oct 2016
    /**
     * @brief initCameraEnumMap - Initialize the cameramap with key as vid,pid and value is corressponding
     * enum value
     */
    initCameraEnumMap();
    handle = NULL;
}

void uvccamera::initCameraEnumMap()
{
    // Added by Dhurka - 13th Oct 2016
    std::string econVid = "2560"; // This is common for econ camera. So initialize once
    cameraEnumMap.clear();
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid + ",c112", CommonEnums::SEE3CAM_11CUG));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid + ",c113", CommonEnums::SEE3CAM_12CUNIR));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid + ",c130", CommonEnums::SEE3CAM_CU30));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid + ",c133", CommonEnums::SEE3CAM_CU38));

    // Added by Sankari : 28 July 2017
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid + ",c120", CommonEnums::SEE3CAM_CU20));

    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid + ",c030", CommonEnums::SEE3CAM_30));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid + ",c140", CommonEnums::SEE3CAM_CU40));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid + ",c151", CommonEnums::SEE3CAM_CU50));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid + ",c152", CommonEnums::SEE3CAM_CU51));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid + ",c1d0", CommonEnums::SEE3CAM_CU130));

    // Added by Sankari : 22 Feb 2017
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid + ",c1d1", CommonEnums::SEE3CAM_CU135));

    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>("04b4,00c3", CommonEnums::CX3_UVC_CAM));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>("046d,082d", CommonEnums::HD_PRO_WEBCAM));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c111", CommonEnums::ECON_1MP_BAYER_RGB));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c110", CommonEnums::ECON_1MP_MONOCHROME));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",d151", CommonEnums::ECON_CX3_RDX_T9P031));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",d052", CommonEnums::ECON_CX3_RDX_V5680));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c080", CommonEnums::ECON_8MP_CAMERA));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c0d0", CommonEnums::SEE3CAM_130));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c0d3", CommonEnums::SEE3CAM_130A));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c081", CommonEnums::SEE3CAM_81));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>("04b4,0035", CommonEnums::CX3_SNI_CAM)); // Cypress Semiconductor Corp : CX3-SNI front and rear camera
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c132", CommonEnums::NILECAM30_USB));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c123", CommonEnums::ECAM22_USB)); // h264 camera
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c154", CommonEnums::SEE3CAM_CU55));

    // Added by Navya
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c155", CommonEnums::SEE3CAM_CU55_MH)); // cu55 monochrome
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c1d4", CommonEnums::FSCAM_CU135));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c124", CommonEnums::SEE3CAM_20CUG));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c126", CommonEnums::SEE3CAM_CU22));
    // Added by M Vishnu Murali
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c129", CommonEnums::ECAM22_USB)); // hyperyon Dual stream device
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c0d6", CommonEnums::SEE3CAM_130D));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",C128", CommonEnums::SEE3CAM_24CUG));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",D400", CommonEnums::NONE)); // See3CAM_160_EVALKIT
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",C180", CommonEnums::SEE3CAM_CU81));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",C058", CommonEnums::ECAM51A_USB));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",C05a", CommonEnums::ECAM51A_USB));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c05c", CommonEnums::ECAM51B_USB));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c181", CommonEnums::ECAM82_USB));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c0d8", CommonEnums::SEE3CAM_1332));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c184", CommonEnums::ECAM83_USB));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c12c", CommonEnums::SEE3CAM_CU27));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c1d7", CommonEnums::SEE3CAM_CU1330M));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c0d7", CommonEnums::SEE3CAM_135M));
    cameraEnumMap.insert(std::pair<std::string, CommonEnums::ECameraNames>(econVid +",c400", CommonEnums::SEE3CAM_160));
}

unsigned int uvccamera::getTickCount()
{
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0)
        return 0;
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

int uvccamera::enumerateDevices(std::vector<std::string> *deviceNames, std::vector<std::string> *devicePaths)
{
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    struct udev_device *dev, *pdev;

    std::string parameter = "video4linux";
    /* Create the udev object */
    udev = udev_new();
    if (!udev)
    {
        std::cerr << "Can't create udev\n";
        return -1;
    }

    /* Create a list of the devices in the 'video4linux/hidraw' subsystem. */
    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, parameter.data());
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);
    /* For each item enumerated, print out its information.
       udev_list_entry_foreach is a macro which expands to
       a loop. The loop will be executed for each member in
       devices, setting dev_list_entry to a list entry
       which contains the device's path in /sys. */
    udev_list_entry_foreach(dev_list_entry, devices)
    {
        const char *path;

        /* Get the filename of the /sys entry for the device
           and create a udev_device object (dev) representing it */
        path = udev_list_entry_get_name(dev_list_entry);
        dev = udev_device_new_from_syspath(udev, path);

        /* usb_device_get_devnode() returns the path to the device node
           itself in /dev. */
        // printf("Device Node Path: %s\n", udev_device_get_devnode(dev));

        /* The device pointed to by dev contains information about
           the hidraw device. In order to get information about the
           USB device, get the parent device with the
           subsystem/devtype pair of "usb"/"usb_device". This will
           be several levels up the tree, but the function will find
           it.*/
        pdev = udev_device_get_parent_with_subsystem_devtype(
            dev,
            "usb",
            "usb_device");

        if (!pdev)
        {
            std::cerr << "Unable to find parent usb device.";
            continue;
        }

        /* From here, we can call get_sysattr_value() for each file
           in the device's /sys entry. The strings passed into these
           functions (idProduct, idVendor, serial, etc.) correspond
           directly to the files in the directory which represents
           the USB device. Note that USB strings are Unicode, UCS2
           encoded, but the strings returned from
           udev_device_get_sysattr_value() are UTF-8 encoded. */

        // Modified by Dhurka - 14th Oct 2016
        /*
         * Modified string comparision with lower case to avoid future issues
         * Previous comparision is not using the lower case comparision
         */
        std::string productName = udev_device_get_sysattr_value(pdev, "product");
        deviceNames->push_back(productName);
        std::string myString = path;
        std::vector<std::string> pieces = split(myString, "/");
        std::string neededWord = pieces.back();
        devicePaths->push_back(neededWord);
        udev_device_unref(dev);
    }
    /* Free the enumerator object */
    udev_enumerate_unref(enumerate);
    udev_unref(udev);
    return 1;
}

// Modified by Nithyesh
/*
 * Removed arg QStringList from function as it was unused.
 * Previous fn signature was
 * int uvccamera::findEconDevice(QStringList *econCamera,std::string parameter)
 */

int uvccamera::findEconDevice(std::string parameter)
{
    cameraMap.clear();
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    struct udev_device *dev, *pdev;

    /* Create the udev object */
    udev = udev_new();
    if (!udev)
    {
        std::cerr << "Can't create udev\n";
        return -1;
    }

    /* Create a list of the devices in the 'video4linux/hidraw' subsystem. */
    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, parameter.data());
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);
    /* For each item enumerated, print out its information.
       udev_list_entry_foreach is a macro which expands to
       a loop. The loop will be executed for each member in
       devices, setting dev_list_entry to a list entry
       which contains the device's path in /sys. */
    udev_list_entry_foreach(dev_list_entry, devices)
    {
        const char *path;

        /* Get the filename of the /sys entry for the device
           and create a udev_device object (dev) representing it */
        path = udev_list_entry_get_name(dev_list_entry);
        dev = udev_device_new_from_syspath(udev, path);

        /* usb_device_get_devnode() returns the path to the device node
           itself in /dev. */
        // printf("Device Node Path: %s\n", udev_device_get_devnode(dev));

        /* The device pointed to by dev contains information about
           the hidraw device. In order to get information about the
           USB device, get the parent device with the
           subsystem/devtype pair of "usb"/"usb_device". This will
           be several levels up the tree, but the function will find
           it.*/
        pdev = udev_device_get_parent_with_subsystem_devtype(
            dev,
            "usb",
            "usb_device");
        if (!pdev)
        {
            std::cerr << "Unable to find parent usb device.";
            continue;
        }

        /* From here, we can call get_sysattr_value() for each file
           in the device's /sys entry. The strings passed into these
           functions (idProduct, idVendor, serial, etc.) correspond
           directly to the files in the directory which represents
           the USB device. Note that USB strings are Unicode, UCS2
           encoded, but the strings returned from
           udev_device_get_sysattr_value() are UTF-8 encoded. */

        // Modified by Dhurka - 14th Oct 2016
        /*
         * Modified string comparision with lower case to avoid future issues
         * Previous comparision is not using the lower case comparision
         */
        if ((toLower(std::string(udev_device_get_sysattr_value(pdev, "idVendor"))) == toLower(econCameraVid)) ||
            (toLower(std::string(udev_device_get_sysattr_value(pdev, "idVendor"))) == toLower(ascellaCameraVid)))
        {
            std::string hid_device = udev_device_get_devnode(dev);
            std::string productName = udev_device_get_sysattr_value(pdev, "product");
            std::string serialNumber = udev_device_get_sysattr_value(pdev, "serial");
            std::string vidValue = udev_device_get_sysattr_value(pdev, "idVendor");
            std::string pidValue = udev_device_get_sysattr_value(pdev, "idProduct");

            if (parameter != "video4linux")
            {
                uvccamera::cameraMap.insert(std::pair<std::string,std::string>(productName, hid_device));
                if (serialNumber.empty())
                    serialNumberMap.insert(std::pair<std::string,std::string>(hid_device, "Not assigned"));
                else
                    serialNumberMap.insert(std::pair<std::string,std::string>(hid_device, serialNumber));
            }
            else
            {
                // Added by Dhurka - 13th Oct 2016
                /*
                 * Added the camera name  and corresponding vid,pid in the pidVid map
                 */
                std::string vidPid = toLower(vidValue.append(",").append(pidValue));
                pidVidMap.insert(std::pair<std::string, std::string>(productName, vidPid));
                std::cout << "pidVidMap - productName: " << productName << std::endl;
                std::cout << "pidVidMap - vidPid: " << vidPid << std::endl;
            }
        }
        udev_device_unref(dev);
    }
    /* Free the enumerator object */
    udev_enumerate_unref(enumerate);
    udev_unref(udev);
    return 1;
}
// Added by Dhurka - 13th Oct 2016
/**
 * @brief uvccamera::currentlySelectedDevice - This is used to get the currently selected
 * camera enum value.
 */
void uvccamera::currentlySelectedDevice(std::string deviceName)
{
    bool deviceFound = false;
    std::string originalDeviceName;
    // Added by Sankari: To fix string name and hid initialization issue: Check the camera name selected is having substring
    // in pidvidmap product name
    std::map<std::string, std::string>::iterator pidvidmapIterator;
    for (pidvidmapIterator = pidVidMap.begin(); pidvidmapIterator != pidVidMap.end(); ++pidvidmapIterator)
    {
        if (deviceName.find(pidvidmapIterator->first) != std::string::npos)
        {
            deviceFound = true;
            originalDeviceName = pidvidmapIterator->first;
        }
    }

    if (deviceFound)
    {
        std::string selectedCameraVidPid = pidVidMap.find(originalDeviceName)->second;
        // Convert the vid,pid value in the cameraEnum map as lower case and compare with the selected Vid,pid
        std::map<std::string, CommonEnums::ECameraNames>::iterator mapIterator; 
        selectedDeviceEnum = CommonEnums::NONE;
        for (mapIterator = cameraEnumMap.begin(); mapIterator != cameraEnumMap.end(); ++mapIterator)
        {
            if (toLower(mapIterator->first) == selectedCameraVidPid)
            {
                selectedDeviceEnum = mapIterator->second;
                break;
            }
        }
    }
    else
    {
        selectedDeviceEnum = CommonEnums::NONE;
    }
}

int uvccamera::initExtensionUnitAscella()
{
    int ret;

    exitExtensionUnitAscella();

    // Added by Nithyesh
    ret = -1;
    kernelDriverDetached = 0;
    libusb_init(NULL);
    libusb_set_debug(NULL, 3);

    handle = libusb_open_device_with_vid_pid(NULL, ASCELLA_VID, ASCELLA_PID);

    if (!handle)
    {
        std::cerr << "\nunable to open the device\n";
    }
    else
    {
        if (libusb_kernel_driver_active(handle, 2))
        {
            ret = libusb_detach_kernel_driver(handle, 2);
            if (ret == 0)
            {
                kernelDriverDetached = 1;
                std::cout << "driver detachment successful\n";
            }
        }

        ret = libusb_claim_interface(handle, 2);
        if (ret == 0)
        {
            std::cout << "Interface Claimed successfully\n";
        }
        else
        {
            std::cerr << "error claiming interface\n";
        }
    }

    return ret;
}

bool uvccamera::closeAscellaDevice()
{
    int res;

    if (handle == NULL)
    {
        return false;
    }
    res = libusb_release_interface(handle, 2);

    if (0 != res)
    {
        std::cerr << "Error releasing interface\n";
        return false;
    }

    if (kernelDriverDetached)
    {
        libusb_attach_kernel_driver(handle, 2);
        std::cout << "Attaching libusb kernel driver\n";
    }

    if (handle)
    {
        libusb_close(handle);
        std::cout << "Closing libusb\n";
    }

    libusb_exit(NULL);
    handle = NULL;

    return true;
}

bool uvccamera::readFirmwareVersion(uint8_t *pMajorVersion, uint8_t *pMinorVersion1, uint16_t *pMinorVersion2, uint16_t *pMinorVersion3)
{

    if (uvccamera::hid_fd < 0)
    {
        return false;
    }

    bool timeout = true;
    int ret = 0;
    unsigned int start, end = 0;
    unsigned short int sdk_ver = 0, svn_ver = 0;

    *pMajorVersion = 0;
    *pMinorVersion1 = 0;
    *pMinorVersion2 = 0;
    *pMinorVersion3 = 0;
    // Initialize the buffer
    memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

    // Set the Report Number
    g_out_packet_buf[1] = READFIRMWAREVERSION; /* Report Number */

    /* Send a Report to the Device */
    ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
    if (ret < 0)
    {
        _text = "Device not available";
        return false;
    }
    /* Read the Firmware Version from the device */
    start = getTickCount();
    while (timeout)
    {
        /* Get a report from the device */
        ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
        if (ret < 0)
        {
        }
        else
        {
            if (g_in_packet_buf[0] == READFIRMWAREVERSION)
            {
                sdk_ver = (g_in_packet_buf[3] << 8) + g_in_packet_buf[4];
                svn_ver = (g_in_packet_buf[5] << 8) + g_in_packet_buf[6];

                *pMajorVersion = g_in_packet_buf[1];
                *pMinorVersion1 = g_in_packet_buf[2];
                *pMinorVersion2 = sdk_ver;
                *pMinorVersion3 = svn_ver;

                timeout = false;
            }
        }
        end = getTickCount();
        if (end - start > TIMEOUT)
        {
            timeout = false;
            return false;
        }
    }
    return true;
}

bool uvccamera::initExtensionUnit(std::string cameraName)
{
    if (cameraName.empty())
    {
        std::cerr << "cameraName not passed as parameter\n";
        return false;
    }
    std::string originalDeviceName;

    std::map<std::string, std::string>::iterator pidvidmapIterator;
    for (pidvidmapIterator = pidVidMap.begin(); pidvidmapIterator != pidVidMap.end(); ++pidvidmapIterator)
    {
        std::cout << "cameraName: " << cameraName << " - pidvidmapIterator->first: " << pidvidmapIterator->first << " - pidvidmapIterator->second: " << pidvidmapIterator->second   << std::endl;
        if (cameraName.find(pidvidmapIterator->first) != std::string::npos )
        {
            originalDeviceName = pidvidmapIterator->first;
        }
    }

    if (hid_fd >= 0)
    {
        close(hid_fd);
    }

    if (hidNode == "")
    {
        std::cout << "hidNode == \"\"";
        return false;
    }


    // Commented by Nithyesh
    // uint i;
    int ret, desc_size = 0, hid_imu = -1, fd;
    char buf[256];
    struct hidraw_devinfo info;
    struct hidraw_report_descriptor rpt_desc;

    /* Open the Device with non-blocking reads. In real life,
           don't use a hard coded path; use libudev instead. */
    std::map<std::string, std::string>::const_iterator ii = cameraMap.find(originalDeviceName);
    openNode = "";
    while (ii != cameraMap.end() && ii->first == originalDeviceName)
    {
        std::cout << "ii->second.data(): " << ii->second.data() << "\n";
        hid_fd = open(ii->second.data(), O_RDWR | O_NONBLOCK);
        memset(buf, 0x0, sizeof(buf));
        /* Get Physical Location */
        ret = ioctl(hid_fd, HIDIOCGRAWPHYS(256), buf);
        if (ret < 0)
        {
            std::cerr << "Unable to access extension unit controls. Please use the command \"sudo qtcam\" while launching application.\n";
            return false;
        }
        std::string tempBuf = buf;
        std::cout << "hidNode: " << hidNode;
        std::cout << " - tempBuf: " << tempBuf << "\n";
        if (tempBuf.find(hidNode) != std::string::npos)
        {
            openNode = ii->second;
            close(hid_fd);
            //          break;
        }
        close(hid_fd);
        ++ii;
    }

    std::cout << "-----------------\n";
    std::cout << "cameraName: " << cameraName << std::endl;
    std::cout << "originalDeviceName: " << originalDeviceName << std::endl;

    std::cout << "Node: " << openNode << std::endl;

    hid_fd = open(openNode.data(), O_RDWR | O_NONBLOCK);
    // Directly open from map value
    // fd = open(cameraMap.value(getCameraName()).toLatin1().data(), O_RDWR|O_NONBLOCK);

    if (hid_fd < 0)
    {
        perror("Unable to open device");
        return false;
    }

    memset(&rpt_desc, 0x0, sizeof(rpt_desc));
    memset(&info, 0x0, sizeof(info));
    memset(buf, 0x0, sizeof(buf));
    /* Get Report Descriptor Size */
    ret = ioctl(hid_fd, HIDIOCGRDESCSIZE, &desc_size);

    /* Get Report Descriptor Size */
    ret = ioctl(hid_fd, HIDIOCGRDESCSIZE, &desc_size);
    if (ret < 0)
    {
        perror("HIDIOCGRDESCSIZE");
        return false;
    }

    /* Get Report Descriptor */
    rpt_desc.size = desc_size;
    ret = ioctl(hid_fd, HIDIOCGRDESC, &rpt_desc);

    if (ret < 0)
    {
        perror("HIDIOCGRDESC");
        return false;
    }

    /* Get Raw Name */
    ret = ioctl(hid_fd, HIDIOCGRAWNAME(256), buf);
    if (ret < 0)
    {
        perror("HIDIOCGRAWNAME");
        return false;
    }

    /* Get Physical Location */
    ret = ioctl(hid_fd, HIDIOCGRAWPHYS(256), buf);
    if (ret < 0)
    {
        perror("HIDIOCGRAWPHYS");
        return false;
    }

    /* Get Raw Info */
    ret = ioctl(hid_fd, HIDIOCGRAWINFO, &info);
    if (ret < 0)
    {
        perror("HIDIOCGRAWINFO");
        return false;
    }
    // Modified by Dhurka - 14th Oct 2016
    /*
     * Added camera enum comparision
     * Before its like camera name comparision
     */
    // Modified by Sankari - 14th Oct 2018
    /*
     * Correcting OS code supported cameras
     */
    if (selectedDeviceEnum == CommonEnums::SEE3CAM_CU81 || selectedDeviceEnum == CommonEnums::SEE3CAM_11CUG || selectedDeviceEnum == CommonEnums::SEE3CAM_12CUNIR || selectedDeviceEnum == CommonEnums::ECON_1MP_BAYER_RGB || selectedDeviceEnum == CommonEnums::ECON_1MP_MONOCHROME || selectedDeviceEnum == CommonEnums::SEE3CAM_CU51 || selectedDeviceEnum == CommonEnums::SEE3CAM_24CUG)
    {
        ret = sendOSCode();
        if (ret == false)
        {
            std::cerr << "OS Identification failed\n";
        }
    }
    if (desc_size == DESCRIPTOR_SIZE_ENDPOINT)
    {
        hid_fd = fd;
    }
    else if (desc_size == DESCRIPTOR_SIZE_IMU_ENDPOINT)
    {
        hid_imu = fd;
    }
    return true;
}

void uvccamera::getDeviceNodeName(std::string hidDeviceNode)
{
    if (hidDeviceNode.empty())
    {
        std::cerr << "hid Device usbAddress Not found as parameter\n";
        return;
    }
    hidNode = hidDeviceNode;
}

void uvccamera::exitExtensionUnit()
{
    close(hid_fd);
}

bool uvccamera::exitExtensionUnitAscella()
{
    bool ret = false;
    if (handle != NULL)
    {
        ret = closeAscellaDevice();
    }
    return ret;
}

bool uvccamera::sendOSCode()
{

    if (hid_fd < 0)
    {
        return false;
    }
    int ret = 0;
    bool timeout = true;
    unsigned int start, end = 0;

    // Initialize the buffer
    memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

    // Set the Report Number for OS identification
    g_out_packet_buf[1] = OS_CODE;  /* Report Number OS identification */
    g_out_packet_buf[2] = LINUX_OS; /* Report Number for Linux OS */

    /* Send a Report to the Device */
    ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
    if (ret < 0)
    {
        std::cerr << "\nOS Identification Failed\n";
        return false;
    }
    start = getTickCount();
    while (timeout)
    {
        /* Get a report from the device */
        ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
        if (ret < 0)
        {
        }
        else
        {
            if (g_in_packet_buf[0] == OS_CODE &&
                g_in_packet_buf[1] == LINUX_OS)
            {
                if (g_in_packet_buf[2] == SET_SUCCESS)
                {
                    // set success
                }
                else if (g_in_packet_buf[2] == SET_FAIL)
                {
                    return false;
                }
                else
                {
                    return false;
                }
                timeout = false;
            }
        }
        end = getTickCount();
        if (end - start > TIMEOUT)
        {
            timeout = false;
            return false;
        }
    }
    return true;
}

void uvccamera::getFirmWareVersion()
{
    std::cout << "Firmware version: ";
    uint8_t MajorVersion = 0, MinorVersion1 = 0;
    uint16_t MinorVersion2 = 0, MinorVersion3 = 0;
    readFirmwareVersion(&MajorVersion, &MinorVersion1, &MinorVersion2, &MinorVersion3);
    _text.clear();
    _text.append("Version: ");
    _text.append(std::to_string(MajorVersion).append(".").append(std::to_string(MinorVersion1)).append(".").append(std::to_string(MinorVersion2)).append(".").append(std::to_string(MinorVersion3)));
    std::cout << _text << std::endl;
}

bool uvccamera::getUniqueId()
{
    std::cout << "Unique Id: ";

    if (uvccamera::hid_fd < 0)
    {
        return false;
    }

    bool timeout = true;
    int ret = 0;
    unsigned int start, end = 0;
    char buffer[50];
    std::string uniqueId = "";
    // Initialize the buffer
    memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

    // Set the Report Number
    g_out_packet_buf[1] = GETCAMERA_UNIQUEID; /* Report Number */

    /* Send a Report to the Device */
    ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
    if (ret < 0)
    {
        _text = "Device not available";
        return false;
    }
    /* Read the Unique id from the device */
    start = getTickCount();
    while (timeout)
    {
        /* Get a report from the device */
        ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
        if (ret < 0)
        {
        }
        else
        {
            if (g_in_packet_buf[0] == GETCAMERA_UNIQUEID)
            {
                sprintf(buffer, "%02x%02x%02x%02x", g_in_packet_buf[1], g_in_packet_buf[2], g_in_packet_buf[3], g_in_packet_buf[4]);
                uniqueId = std::string(buffer);
                timeout = false;
            }
        }
        end = getTickCount();
        if (end - start > TIMEOUT)
        {
            timeout = false;
            return false;
        }
    }
    _text.clear();
    _text.append(uniqueId);

    std::cout << _text << std::endl;
    return true;
}

std::string uvccamera::retrieveSerialNumber()
{
    auto it = serialNumberMap.find(openNode);
    if (it != serialNumberMap.end())
        return it->second;
    return "";
}

// bool See3CAM_Control::getFlashState(uint8_t *flashState) {

//     *flashState = 0;
//     if(uvccamera::hid_fd < 0)
//     {
//         return false;
//     }

//     bool timeout = true;
//     int ret =0;
//     unsigned int start, end = 0;
//     //Initialize the buffer
//     memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

//     //Set the Report Number
//     //Modified by Dhurka - 14th Oct 2016
//     /*
//      * Added camera enum comparision
//      * Before its like camera name comparision
//      */
//     if(selectedDeviceEnum == CommonEnums::ECON_8MP_CAMERA)
//     {
//         g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
//     }
//     else if(selectedDeviceEnum == CommonEnums::SEE3CAM_CU50)
//     {
//         g_out_packet_buf[1] = CAMERA_CONTROL_50; /* Report Number */
//     }
//     else if(selectedDeviceEnum == CommonEnums::SEE3CAM_12CUNIR)
//     {
//         g_out_packet_buf[1] = CAMERA_CONTROL_AR0130; /* Report Number */
//     }

//     //g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
//     g_out_packet_buf[2] = GET_FLASH_LEVEL; /* Report Number */

//     ret = write(uvccamera::hid_fd, g_out_packet_buf, BUFFER_LENGTH);

//     if (ret < 0) {
//         return false;
//     }
//     /* Read the Status code from the device */
//     start = getTickCount();
//     while(timeout)
//     {
//         /* Get a report from the device */
//         ret = read(uvccamera::hid_fd, g_in_packet_buf, BUFFER_LENGTH);
//         if (ret < 0) {
//         } else {
//             if((g_in_packet_buf[0] == g_out_packet_buf[1])&&
//                     (g_in_packet_buf[1]==GET_FLASH_LEVEL)) {
//                 *flashState = (g_in_packet_buf[2]);
//                 timeout=false;
//             }
//         }
//         end = getTickCount();
//         if(end - start > TIMEOUT)
//         {
//             timeout = false;
//             return false;
//         }
//     }
//     return true;
// }

// bool See3CAM_Control::setFlashState(flashTorchState flashState)
// {
//     if(uvccamera::hid_fd < 0)
//     {
//         return false;
//     }

//     bool timeout = true;
//     int ret =0;
//     unsigned int start, end = 0;

//     if(flashState == flashOff || flashState == flashOn)
//     {
//         //Initialize the buffer
//         memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
//         //Set the Report Number
//         //Modified by Dhurka - 14th Oct 2016
//         /*
//          * Added camera enum comparision
//          * Before its like camera name comparision
//          */
//         if(selectedDeviceEnum == CommonEnums::ECON_8MP_CAMERA)
//         {
//             g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
//         }
//         else if(selectedDeviceEnum == CommonEnums::SEE3CAM_CU50)
//         {
//             g_out_packet_buf[1] = CAMERA_CONTROL_50; /* Report Number */
//         }
//         else if(selectedDeviceEnum == CommonEnums::SEE3CAM_12CUNIR)
//         {
//             g_out_packet_buf[1] = CAMERA_CONTROL_AR0130; /* Report Number */
//         }
//         g_out_packet_buf[2] = SET_FLASH_LEVEL; 	/* Report Number */
//         g_out_packet_buf[3] = flashState;		/* Flash mode */

//         ret = write(uvccamera::hid_fd, g_out_packet_buf, BUFFER_LENGTH);

//         if (ret < 0) {
//             return false;
//         }
//         /* Read the Status code from the device */
//         start = getTickCount();
//         while(timeout)
//         {
//             /* Get a report from the device */
//             ret = read(uvccamera::hid_fd, g_in_packet_buf, BUFFER_LENGTH);
//             if (ret < 0) {
//             } else {
//                 if((g_in_packet_buf[0] == g_out_packet_buf[1])&&
//                         (g_in_packet_buf[1]==SET_FLASH_LEVEL) &&
//                         (g_in_packet_buf[2]==flashState )) {
//                     if(g_in_packet_buf[3] == SET_FAIL) {
//                         return false;
//                     } else if(g_in_packet_buf[3]==SET_SUCCESS) {
//                         timeout = false;
//                     }
//                 }
//             }
//             end = getTickCount();
//             if(end - start > TIMEOUT)
//             {
//                 timeout = false;
//                 return false;
//             }
//         }
//     }
//     else
//     {
//         return false;
//     }
//     return true;
// }

// bool See3CAM_Control::getTorchState(uint8_t *torchState)
// {

//     *torchState = 0;
//     if(uvccamera::hid_fd < 0)
//     {
//         return false;
//     }

//     bool timeout = true;
//     int ret =0;
//     unsigned int start, end = 0;
//     //Initialize the buffer
//     memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

//     //Set the Report Number
//     //Modified by Dhurka - 14th Oct 2016
//     /*
//      * Added camera enum comparision
//      * Before its like camera name comparision
//      */
//     if(selectedDeviceEnum == CommonEnums::ECON_8MP_CAMERA)
//     {
//         g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
//     }
//     else if(selectedDeviceEnum == CommonEnums::SEE3CAM_CU50)
//     {
//         g_out_packet_buf[1] = CAMERA_CONTROL_50; /* Report Number */
//     }
//     else if(selectedDeviceEnum == CommonEnums::SEE3CAM_CU51)
//     {
//         g_out_packet_buf[1] = CAMERA_CONTROL_51; /* Report Number */
//     }
//     g_out_packet_buf[2] = GET_TORCH_LEVEL; /* Report Number */
//     ret = write(uvccamera::hid_fd, g_out_packet_buf, BUFFER_LENGTH);
//     if (ret < 0) {
//         return false;
//     }
//     /* Read the Status code from the device */
//     start = getTickCount();
//     while(timeout)
//     {
//         /* Get a report from the device */
//         ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
//         if (ret < 0) {
//         } else {
//             if((g_in_packet_buf[0] == g_out_packet_buf[1])&&
//                     (g_in_packet_buf[1] == GET_TORCH_LEVEL)) {
//                 *torchState = (g_in_packet_buf[2]);
//                 timeout=false;
//             }
//         }
//         end = getTickCount();
//         if(end - start > TIMEOUT)
//         {
//             timeout = false;
//             return false;
//         }
//     }
//     return true;
// }

// bool See3CAM_Control::setTorchState(flashTorchState torchState)
// {
//     if(uvccamera::hid_fd < 0)
//     {
//         return false;
//     }

//     bool timeout = true;
//     int ret =0;
//     unsigned int start, end = 0;

//     if(torchState == torchOff || torchState == torchOn)
//     {
//         //Initialize the buffer
//         memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

//         //Set the Report Number
//         //Modified by Dhurka - 14th Oct 2016
//         /*
//          * Added camera enum comparision
//          * Before its like camera name comparision
//          */
//         if(selectedDeviceEnum == CommonEnums::ECON_8MP_CAMERA)
//         {
//             g_out_packet_buf[1] = CAMERA_CONTROL_80; /* Report Number */
//         }
//         else if(selectedDeviceEnum == CommonEnums::SEE3CAM_CU50)
//         {
//             g_out_packet_buf[1] = CAMERA_CONTROL_50; /* Report Number */
//         }
//         else if(selectedDeviceEnum == CommonEnums::SEE3CAM_CU51)
//         {
//             g_out_packet_buf[1] = CAMERA_CONTROL_51; /* Report Number */
//         }
//         g_out_packet_buf[2] = SET_TORCH_LEVEL; 	/* Report Number */

//         if(selectedDeviceEnum == CommonEnums::SEE3CAM_CU51 && torchState == torchOff)
//         {
//             g_out_packet_buf[3] = 2;		/* Torch mode */
//         }
//         else
//         {
//              g_out_packet_buf[3] = torchState;		/* Torch mode */
//         }

//         ret = write(uvccamera::hid_fd, g_out_packet_buf, BUFFER_LENGTH);

//         if (ret < 0) {
//             return false;
//         }
//         /* Read the Status code from the device */
//         start = getTickCount();
//         while(timeout)
//         {
//             /* Get a report from the device */
//             ret = read(uvccamera::hid_fd, g_in_packet_buf, BUFFER_LENGTH);
//             if (ret < 0) {
//             } else {
//                 if((g_in_packet_buf[0] == g_out_packet_buf[1])&&
//                         (g_in_packet_buf[1]==SET_TORCH_LEVEL) &&
//                         (g_in_packet_buf[2]==g_out_packet_buf[3] )) {
//                     if(g_in_packet_buf[3] == SET_FAIL) {
//                         return false;
//                     } else if(g_in_packet_buf[3]==SET_SUCCESS) {
//                         timeout = false;
//                     }
//                 }
//             }
//             end = getTickCount();
//             if(end - start > TIMEOUT)
//             {
//                 timeout = false;
//                 return false;
//             }
//         }
//     }
//     else
//     {
//         return false;
//     }
//     return true;
// }

// void See3CAM_Control::setFlashControlState(const int flashState)
// {
//     if(flashState == 1)
//         flashCheckBoxState = flashOn;
//     else
//         flashCheckBoxState = flashOff;
//     setFlashState(flashCheckBoxState);
// }

// void See3CAM_Control::setTorchControlState(const int torchState) {
//     if(torchState == 1)
//         torchCheckBoxState = torchOn;
//     else
//         torchCheckBoxState = torchOff;
//     setTorchState(torchCheckBoxState);
// }

// bool See3CAM_GPIOControl::getGpioLevel(camGpioPin gpioPinNumber)
// {
//     if(uvccamera::hid_fd < 0)
//     {
//         return false;
//     }
//     bool timeout = true;
//     int ret = 0;
//     unsigned int start, end = 0;

//     //Initialize the buffer
//     memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

//     //Set the Report Number
//     g_out_packet_buf[1] = GPIO_OPERATION; 	/* Report Number */
//     g_out_packet_buf[2] = GPIO_GET_LEVEL; 	/* Report Number */
//     g_out_packet_buf[3] = gpioPinNumber; 		/* GPIO Pin Number */

//     /* Send a Report to the Device */
//     ret = write(uvccamera::hid_fd, g_out_packet_buf, BUFFER_LENGTH);
//     if (ret < 0) {
//         return false;
//     }
//     /* Read the GPIO level and status of read from the device */
//     start = uvc.getTickCount();
//     while(timeout)
//     {
//         /* Get a report from the device */
//         ret = read(uvccamera::hid_fd, g_in_packet_buf, BUFFER_LENGTH);

//         if (ret < 0) {
//         } else {
//             if(g_in_packet_buf[0] == GPIO_OPERATION &&
//                     g_in_packet_buf[1] == GPIO_GET_LEVEL &&
//                     g_in_packet_buf[2] == gpioPinNumber) {
//                 emit gpioLevel(g_in_packet_buf[3]);
//                 if(g_in_packet_buf[4] == GPIO_LEVEL_FAIL) {
//                     return false;
//                 } else if(g_in_packet_buf[4]==GPIO_LEVEL_SUCCESS) {
//                     timeout = false;
//                 }
//             }
//         }
//         end = uvc.getTickCount();
//         if(end - start > TIMEOUT)
//         {
//             timeout = false;
//             return false;
//         }
//     }
//     return true;
// }

// bool See3CAM_GPIOControl::setGpioLevel(camGpioPin gpioPin,camGpioValue gpioValue)
// {
//     if(uvccamera::hid_fd < 0)
//     {
//         return false;
//     }

//     bool timeout = true;
//     int ret = 0;
//     unsigned int start, end = 0;

//     //Initialize the buffer
//     memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

//     //Set the Report Number
//     g_out_packet_buf[1] = GPIO_OPERATION; 	/* Report Number */
//     g_out_packet_buf[2] = GPIO_SET_LEVEL; 	/* Report Number */
//     g_out_packet_buf[3] = gpioPin; 		/* GPIO Pin Number */
//     g_out_packet_buf[4] = gpioValue; 	/* GPIO Value */

//     /* Send a Report to the Device */
//     ret = write(uvccamera::hid_fd, g_out_packet_buf, BUFFER_LENGTH);
//     if (ret < 0) {
//         return false;
//     }
//     /* Read the GPIO level and status of read from the device */
//     start = uvc.getTickCount();
//     while(timeout)
//     {
//         /* Get a report from the device */
//         ret = read(uvccamera::hid_fd, g_in_packet_buf, BUFFER_LENGTH);
//         if (ret < 0) {
//         } else {
//             if(g_in_packet_buf[0] == GPIO_OPERATION &&
//                     g_in_packet_buf[1] == GPIO_SET_LEVEL &&
//                     g_in_packet_buf[2] == gpioPin &&
//                     g_in_packet_buf[3] == gpioValue) {
//                 if(g_in_packet_buf[4] == GPIO_LEVEL_FAIL) {
//                     emit deviceStatus(tr("Failure"] = tr("Unable to change the GPIO level"));
//                     return false;
//                 } else if(g_in_packet_buf[4]==GPIO_LEVEL_SUCCESS) {
//                     timeout = false;
//                 }
//             }
//         }
//         end = uvc.getTickCount();
//         if(end - start > TIMEOUT)
//         {
//             timeout = false;
//             return false;
//         }
//     }
//     return true;
// }

// //Modified by Dhurka - Braces alignment - 14th Oct 2016
// bool See3CAM_ModeControls::enableMasterMode()
// {
//     int ret =0;

//     if(uvccamera::hid_fd < 0)
//     {
//         return false;
//     }
//     //Initialize the buffer
//     memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

//     //Set the Report Number
//     g_out_packet_buf[1] = ENABLEMASTERMODE; /* Report Number */
//     ret = write(uvccamera::hid_fd, g_out_packet_buf, BUFFER_LENGTH);
//     if (ret < 0) {
//         return false;
//     }
//     return true;
// }

// //Modified by Dhurka - Braces alignment - 14th Oct 2016
// bool See3CAM_ModeControls::enableTriggerMode()
// {
//     int ret =0;

//     if(uvccamera::hid_fd < 0)
//     {
//         return false;
//     }
//     //Initialize the buffer
//     memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

//     //Set the Report Number
//     g_out_packet_buf[1] = ENABLETRIGGERMODE; /* Report Number */

//     ret = write(uvccamera::hid_fd, g_out_packet_buf, BUFFER_LENGTH);
//     if (ret < 0) {
//         return false;
//     }
//     return true;
// }

/**
 * @brief sendHidCmd - Sending hid command and get reply back
 * @param outBuf - Buffer that fills to send into camera
 * @param inBuf  - Buffer to get reply back
 * @param len    - Buffer length
 * return true/false
 * */
bool uvccamera::sendHidCmd(unsigned char *outBuf, unsigned char *inBuf, int len)
{
    // Write data into camera
    int ret = write(hid_fd, outBuf, len);
    if (ret < 0)
    {
        return false;
    }
    struct timeval tv;
    fd_set rfds;

    FD_ZERO(&rfds);
    FD_SET(hid_fd, &rfds);

    /* Wait up to 5 seconds. */
    tv.tv_sec = 5;
    tv.tv_usec = 0;

    // Monitor read file descriptor for 5 secs

    if (0 > select(1, &rfds, NULL, NULL, &tv))
    {
        perror("select");
        return false;
    }

    // Read data from camera
    int retval = read(hid_fd, inBuf, len);
    if (retval < 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}
