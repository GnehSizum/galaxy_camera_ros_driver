#ifndef GXCAMERA_H
#define GXCAMERA_H

#include "libgxiapi/GxIAPI.h"
#include "libgxiapi/DxImageProc.h"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

using string = std::string;

#define GX_SUCCESS(s) (s == GX_STATUS_SUCCESS)

// Show error message
#define GX_VERIFY(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)    	\
    {                                 		\
        getErrorString(emStatus);			\
		return emStatus;					\
    }

// Get error message and show it
void getErrorString(GX_STATUS errorStatus);

// ROI Param struct
typedef struct RoiParam {
	int64_t	width;		// Image roi width
	int64_t	height;    	// Image roi height
	int64_t offsetX;   	// OffsetX of roi
	int64_t offsetY;   	// OffsetY of roi
	RoiParam() {
		width	= 640;
		height	= 480;
		offsetX = 0;
		offsetY = 0;
	}
} RoiParam;

// Exposure Param struct
typedef struct ExposureParam {
	double 	exposureTimeUs;			// Exposure Time
	bool   	autoExposure;			// Whether to enable auto exposure
	double 	autoExposureTimeMinUs;	// Minimum exposure time when using Auto Exporsure mode
	double 	autoExposureTimeMaxUs;	// Maximum exposure time when using Auto Exporsure mode
	ExposureParam() {
		exposureTimeUs			= 2000;
		autoExposure			= false;
		autoExposureTimeMinUs	= 1000;
		autoExposureTimeMaxUs	= 10000;
	}
} ExposureParam;


// Gain Param struct
typedef struct GainParam {
	double	gainDb;				// Gain
	bool   	autoGain;			// Whether to enable auto gain
	double 	autoGainMinDb;		// Minimum gain when using Auto Gain mode
	double 	autoGainMaxDb;		// Maximum gain when using Auto Gain mode
	GainParam() {
		gainDb			= 2;
		autoGain		= false;
		autoGainMinDb	= 0;
		autoGainMaxDb	= 10;
	}
} GainParam;

class GxCamera {
  public:
	GxCamera();
	~GxCamera();

	/**
	 * @brief  Initialize ROS
	 * @return bool 
	 */
	bool initROS();

	/**
	 * @brief  Initialize the Galaxy SDK
	 * @return GX_STATUS 
	 */
	GX_STATUS initLib();

	/**
	 * @brief  Close the Galaxy SDK
	 * @return GX_STATUS 
	 */
	GX_STATUS closeLib();

	/**
	 * @brief  Open the Corresponding Device according to the Camera SN number
	 * @param  cameraSN   The camera SN number
	 * @return GX_STATUS 
	 */
	GX_STATUS openDeviceBySN(string cameraSN, GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Open the Corresponding Device according to the Camera Index number
	 * @param  camIndex 
	 * @return GX_STATUS  
	 */
	GX_STATUS openDeviceByIndex(string camIndex,GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Turn off the Currently Open Camera Device
	 * @return GX_STATUS 
	 */
	GX_STATUS closeDevice(GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Start Acquiring
	 * @return GX_STATUS 
	 */
	GX_STATUS startAcquiring(GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Stop Acquiring
	 * @return GX_STATUS 
	 */
	GX_STATUS stopAcquiring(GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Set the White Balance On
	 * @param  whiteBalance Whether to enable white balance
	 * @return GX_STATUS 
	 */
	GX_STATUS setWhiteBalanceOn(bool whiteBalance, GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Set the Trigger Mode
	 * @param  is_trigger Whether to enable trigger mode
	 * @return GX_STATUS 
	 */
	GX_STATUS setTriggerGrab(GX_DEV_HANDLE& camHandle_, bool is_trigger);

	/**
	 * @brief Set the ROI Params
	 * @param width   ROI width
	 * @param height  ROI height
	 * @param offsetX OffsetX of roi
	 * @param offsetY OffsetY of roi
	 */
	void setRoiParam(	int64_t width,
						int64_t height,
						int64_t offsetX,
						int64_t offsetY);

	/**
	 * @brief Set the Exposure Params
	 * @param exposureTimeUs        Timed Exposure Value, units: us  
	 * @param autoExposure          Whether to enable auto exposure
	 * @param autoExposureTimeMinUs Minimum Exposure Time when using Auto Exposure Mode, units: us
	 * @param autoExposureTimeMaxUs Maximum Exposure Time when using Auto Exposure Mode, units: us
	 */
	void setExposureParam(	double exposureTimeUs,
							bool autoExposure,
							double autoExposureTimeMinUs,
							double autoExposureTimeMaxUs);

	/**
	 * @brief Set the Gain Params
	 * @param gainDb        Gain, units: db
	 * @param autoGain 		Whether to enable auto gain
	 * @param autoGainMinDb Minimum Gain when using Auto Gain Mode, units: db
	 * @param autoGainMaxDb Maximum Gain when using Auto Gain Mode, units: db
	 */
	void setGainParam(	double gainDb,
						bool autoGain,
						double autoGainMinDb,
						double autoGainMaxDb);

	/**
	 * @brief Timer Callback
	 */
	void onFrameCallback(const ros::TimerEvent&);
	
  private:
	GX_DEV_HANDLE   camHandle;
	GX_FRAME_DATA	gxFrame;
	string 			camIndex;
	GX_STATUS 		status = GX_STATUS_SUCCESS;

	// Camera params
	RoiParam		roiParam;
	ExposureParam	exposureParam;
	GainParam		gainParam;
	
	bool			isAcquiring;
	bool			isColorCam;

	// ROS
	ros::NodeHandle nh;
	ros::Timer timer;
	sensor_msgs::Image imageMsg;
	sensor_msgs::CameraInfo infoMsg;
	image_transport::CameraPublisher camPub;
	boost::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;

	// Params buffer
	string camName;
	string camSN;
	string camInfoURL;

  private:
	/**
	 * @brief  Load ROI Params
	 * @return GX_STATUS 
	 */
	GX_STATUS loadRoi(GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Load Exposure Params
	 * @return GX_STATUS 
	 */
	GX_STATUS loadExposure(GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Load Gain Params
	 * @return GX_STATUS 
	 */
	GX_STATUS loadGain(GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Set the Pixel Format to 8bit
	 * @return GX_STATUS 
	 */
	GX_STATUS GxSetPixelFormat8bit(GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Request Memory Space for gxFrame.pImgBuf
	 * @return GX_STATUS 
	 */
	GX_STATUS allocateImageBuf(GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Read the Params of the Turned-On Camera
	 * @return GX_STATUS 
	 */
	GX_STATUS readCameraParams(GX_DEV_HANDLE& camHandle_);

	/**
	 * @brief  Optimize ROI Params to Meet Camera ROI Requirements
	 * @return GX_STATUS 
	 */
	GX_STATUS GxOptimizeRoiParam(GX_DEV_HANDLE& camHandle_);

}; // class GxCamera

#endif // GXCAMERA_H