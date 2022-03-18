#ifndef SEE3CAM_24CUG_H
#define SEE3CAM_24CUG_H

#include "uvccamera.h"
#include <string>

#define CAMERA_CONTROL_24CUG 0xA8

#define EXPOSURECOMP_MIN 50
#define EXPOSURECOMP_MAX 100000000

#define SMILE_THRESHOLD_MIN 40
#define SMILE_THRESHOLD_MAX 75
#define DEFAULT_SMILE_THRESHOLD 40

#define GET_SPECIALEFFECT_24CUG              0x03
#define SET_SPECIALEFFECT_24CUG              0x04

#define GET_DENOISE_24CUG					0x05
#define SET_DENOISE_24CUG					0x06

#define GET_EXP_ROI_MODE_24CUG               0x07
#define SET_EXP_ROI_MODE_24CUG 				0x08

#define GET_Q_FACTOR_24CUG                   0x09
#define SET_Q_FACTOR_24CUG                   0x0A

#define GET_FLIP_MODE_24CUG               0x0B
#define SET_FLIP_MODE_24CUG               0x0C


#define SET_FLIP_BOTHFLIP_ENABLE_24CUG    0x03
#define SET_FLIP_HORZFLIP_24CUG           0x01
#define SET_FLIP_VERTFLIP_24CUG           0x02
#define SET_FLIP_BOTHFLIP_DISABLE_24CUG   0x00

#define GET_FACE_DETECT_RECT_24CUG          0x0D
#define SET_FACE_DETECT_RECT_24CUG          0x0E

#define ENABLE_FACE_RECT                0x01
#define DISABLE_FACE_RECT               0x00
#define ENABLE_EMBED_DATA               0x01
#define DISABLE_EMBED_DATA              0x00
#define ENABLE_OVERLAY_RECT             0x01
#define DISABLE_OVERLAY_RECT            0x00

#define GET_SMILE_DETECTION                  0x0F
#define SET_SMILE_DETECTION                  0x10

#define ENABLE_SMILE_DETECT_24CUG             0x01
#define DISABLE_SMILE_DETECT_24CUG            0x00

#define GET_EXPOSURE_COMPENSATION_24CUG           0x11
#define SET_EXPOSURE_COMPENSATION_24CUG            0x12

#define GET_FRAME_RATE_24CUG                  0x13
#define SET_FRAME_RATE_24CUG                  0x14

#define GET_STROBE_CONTROL_24CUG             0x15
#define SET_STROBE_CONTROL_24CUG             0x16

#define GET_FLICKER_CONRTOL_24CUG            0x17
#define SET_FLICKER_CONTROL_24CUG            0x18

#define GET_BURST_LENGTH_24CUG               0x19
#define SET_BURST_LENGTH_24CUG               0x1A

#define GET_STREAM_MADE_24CUG                0x1B
#define SET_STREAM_MADE_24CUG                0x1C

#define SET_TO_DEFAULT_24CUG                 0xFF

//#define SET_OS_CODE                              0x70

class See3CAM_24CUG
{
private:
    unsigned char g_out_packet_buf[BUFFER_LENGTH];
    unsigned char g_in_packet_buf[BUFFER_LENGTH];
    uvccamera uvc;

    void initializeBuffers();

public:

    enum streamModes {
       MODE_MASTER =0x00,
       MODE_TRIGGER =0x01
    };

    enum specialEffects
    {
        EFFECT_NORMAL = 0x01,
        EFFECT_BLACK_WHITE = 0x04,
        EFFECT_GREYSCALE = 0x07,
        EFFECT_NEGATIVE = 0x08,
        EFFECT_SKETCH = 0x10
    };

    enum camROIAutoExpMode {
        AutoExpFace = 0x00,
        AutoExpFull = 0x01,
        AutoExpManual = 0x02,
        AutoExpDisabled = 0x03
    };

    enum flipMirrorControls{
        FLIP_OFF_MIRROR_OFF = 0x00,
        FLIP_ON_MIRROR_OFF  = 0x01,
        FLIP_OFF_MIRROR_ON  = 0x02,
        FLIP_ON_MIRROR_ON   = 0x03
    };

    enum camFaceRectMode {
        FaceRectEnable = 0x01,
        FaceRectDisable = 0x00
    };

    enum camFaceDetectEmbedDataMode {
        FaceDetectEmbedDataEnable = 0x01,
        FaceDetectEmbedDataDisable = 0x00
    };

    enum camFaceDetectOverlayRect {
        FaceDetectOverlayRectEnable = 0x01,
        FaceDetectOverlayRectDisable = 0x00
    };

    enum camSmileDetectMode {
        SmileDetectEnable = 0x01,
        SmileDetectDisable = 0x00
    };

    enum camSmileDetectEmbedDataMode {
        SmileDetectEmbedDataEnable = 0x01,
        SmileDetectEmbedDataDisable = 0x00
    };

    enum flashStateValues{
        FLASHMODE_OFF = 0x00,
         FLASHMODE_STROBE = 0x01
     };

    enum camFlickerMode {
         MODE_DISABLE = 0x00,
         MODE_50Hz =0x01,
         MODE_60Hz =0x02,
         MODE_AUTO =0x03
       };

    bool getEffectMode(unsigned char &out);
    bool setEffectMode(const specialEffects& specialEffect);

    bool getDenoiseValue(unsigned char &out);
    bool setDenoiseValue(int deNoiseVal);

    bool getAutoExpROIModeAndWindowSize(unsigned char &out1, unsigned char &out2);
    bool setROIAutoExposure(camROIAutoExpMode see3camAutoexpROIMode, uint vidResolnWidth, uint vidResolnHeight, uint xCord, uint yCord, std::string winSize);

    bool getBurstLength(unsigned char &out);
    bool setBurstLength(uint burstLength, uint &out);

    bool getQFactor(unsigned char &out);
    bool setQFactor(uint qFactor);

    bool getOrientation(unsigned char &out);
    bool setOrientation(bool horzModeSel, bool vertiModeSel);

    bool getFrameRateCtrlValue(unsigned char &out);
    bool setFrameRateCtrlValue(uint frameRate);

    bool getExposureCompensation(unsigned int &exposureCompValue);
    bool setExposureCompensation(unsigned int exposureCompValue);

    bool getFaceDetectMode(unsigned char &out1, unsigned char &out2, unsigned char &out3);
    bool setFaceDetectionRect(bool enableFaceDetectRect, bool embedData, bool overlayRect);

    bool getSmileDetectMode(unsigned char &out1, unsigned char &out2);
    bool setSmileDetection(bool enableSmileDetect, bool embedData);

    bool getFlickerDetection(unsigned char &out);
    bool setFlickerDetection(camFlickerMode flickerMode);

    bool getFlashState(unsigned char &out);
    bool setFlashState(flashStateValues flashMode);

    bool getStreamMode(uint &streamMode, bool &autoFunctionLock);
    bool setStreamMode(streamModes streamMode , bool autoFunctionLock);

    bool setToDefault();
};

#endif // SEE3CAM_24CUG_H
