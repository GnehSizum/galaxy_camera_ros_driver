#include "GxCamera.h"

GxCamera::GxCamera() {
	gxFrame			= GX_FRAME_DATA();
	roiParam		= RoiParam();
	exposureParam	= ExposureParam();
	gainParam		= GainParam();
	isAcquiring		= false;
	isColorCam		= true;
	camIndex		= "";

	// Initialize ROS
	if(initROS()) {
		printf(">>> ROS init finished!\n");
	}

	// Initialize lib
	do {
		status = initLib();
		if(!GX_SUCCESS(status)) {
			printf(">>> Failed to init GxIAPI!\n");
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	} while(!GX_SUCCESS(status));

	// Try opening the camera
	status = openDeviceBySN(camSN, camHandle);		// By SN
	// status = openDeviceByIndex("0", camHandle);		// By Index
	if(!GX_SUCCESS(status)) {
		printf(">>> Can not open camera, SN is %s\n", camSN.c_str());
	} else {
		printf(">>> Open camera, SN is %s\n", camSN.c_str());
	}

	// Set camera params
	setRoiParam(infoMsg.width, infoMsg.height, 0, 0);
	setExposureParam(25000, true, 100, 30000);
	setGainParam(5, true, 0, 10);
	setWhiteBalanceOn(true, camHandle);
	setTriggerGrab(camHandle, false);

	// Start acquiring
	status = startAcquiring(camHandle);
	if(!GX_SUCCESS(status)) {
		printf(">>> Failed to start acquiring!\n");
	} else {
		printf(">>> Start acquiring!\n");
	}

	timer = nh.createTimer(ros::Duration(0.016), &GxCamera::onFrameCallback, this);
}

GxCamera::~GxCamera() {
	stopAcquiring(camHandle);
	closeDevice(camHandle);
	closeLib();
}

bool GxCamera::initROS() {
	nh.param<string>("camera/name", camName, "camera");
	nh.param<string>("camera/SN", camSN, "233");
	nh.param<string>("camera/camera_info_url", camInfoURL, "package://galaxy_camera/config/camera_info.yaml");

	image_transport::ImageTransport it(nh);
	camPub = it.advertiseCamera("/galaxy_camera/" + camName + "/image", 10);

	infoManager.reset(new camera_info_manager::CameraInfoManager(nh, camName, camInfoURL));
	if(infoManager->validateURL(camInfoURL)) {
		infoManager->loadCameraInfo(camInfoURL);
		infoMsg = infoManager->getCameraInfo();
	} else {
		ROS_WARN("Invalid camera info URL: %s", camInfoURL.c_str());
	}
	return true;
}

GX_STATUS GxCamera::initLib() {
	GX_STATUS status = GX_STATUS_SUCCESS;
	printf("\n");
	printf("---------------------------------------------\n");
	printf("- Daheng Galaxy Camera C++ drive startup... -\n");
	printf("---------------------------------------------\n");
	printf("\n");
	printf(">>> Initializing......");
	printf("\n");
	status = GXInitLib();
	GX_VERIFY(status);
	return status;
}

GX_STATUS GxCamera::closeLib() {
	GX_STATUS status = GX_STATUS_SUCCESS;
	printf("\n");
	printf("---------------------------------------------\n");
	printf("- Daheng Galaxy Camera C++ drive closing... -\n");
	printf("---------------------------------------------\n");
	printf("\n");
	status = GXCloseLib();
	printf(">>> GxCamera Drive Exit!");
	printf("\n");
	return status;
}

GX_STATUS GxCamera::openDeviceBySN(string camSN_, GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;

	if (camHandle_ != NULL) {
		status = closeDevice(camHandle_);
		GX_VERIFY(status);
	}

	GX_OPEN_PARAM stOpenParam;
	stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
	stOpenParam.openMode = GX_OPEN_SN;
	stOpenParam.pszContent = const_cast<char*>(camSN_.c_str());

	status = GXOpenDevice(&stOpenParam, &camHandle_);
	GX_VERIFY(status);
	status = readCameraParams(camHandle_);
	GX_VERIFY(status);
	camIndex = camSN_;

	return status;
}

GX_STATUS GxCamera::openDeviceByIndex(string camIndex_, GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;

	if (camHandle_ != NULL) {
		status = closeDevice(camHandle_);
		GX_VERIFY(status);
	}

	GX_OPEN_PARAM stOpenParam;
	stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
	stOpenParam.openMode = GX_OPEN_INDEX;
	stOpenParam.pszContent = const_cast<char*>(camIndex_.c_str());

	status = GXOpenDevice(&stOpenParam, &camHandle_);
	GX_VERIFY(status);
	status = readCameraParams(camHandle_);
	GX_VERIFY(status);
	camIndex = camIndex_;

	return status;
}

GX_STATUS GxCamera::closeDevice(GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;
	bool confSavable = false;

	status = GXIsImplemented(camHandle_, GX_ENUM_USER_SET_DEFAULT, &confSavable);
	GX_VERIFY(status);

	if (confSavable == true) {
		status = GXSetEnum(camHandle_, GX_ENUM_USER_SET_SELECTOR, GX_ENUM_USER_SET_SELECTOR_USERSET0);
		GX_VERIFY(status);
		status = GXSendCommand(camHandle_, GX_COMMAND_USER_SET_SAVE);
		GX_VERIFY(status);
		status = GXSetEnum(camHandle_, GX_ENUM_USER_SET_DEFAULT, GX_ENUM_USER_SET_DEFAULT_USERSET0);
		GX_VERIFY(status);
	}

	status = GXCloseDevice(camHandle_);
	GX_VERIFY(status);
	camHandle_ = NULL;

	return status;
}

GX_STATUS GxCamera::startAcquiring(GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;

	// Load roi params
	status = loadRoi(camHandle_);
	GX_VERIFY(status);

	// Load exposure params
	status = loadExposure(camHandle_);
	GX_VERIFY(status);

	// Load gain params
	status = loadGain(camHandle_);
	GX_VERIFY(status);

	// Alloc image buffer
	status = allocateImageBuf(camHandle_);
	GX_VERIFY(status);

	// Send start acquisition cmd
	status = GXSendCommand(camHandle_, GX_COMMAND_ACQUISITION_START);
	GX_VERIFY(status);

	isAcquiring = true;

	return status;
}

GX_STATUS GxCamera::stopAcquiring(GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;

	// Send stop acquisition cmd
	status = GXSendCommand(camHandle_, GX_COMMAND_ACQUISITION_STOP);
	GX_VERIFY(status);

	// Release image buffer
	free(gxFrame.pImgBuf);
	gxFrame.pImgBuf = NULL;

	isAcquiring = false;

	return status;
}

GX_STATUS GxCamera::setTriggerGrab(GX_DEV_HANDLE& camHandle_, bool if_trigger) {
	GX_STATUS status = GX_STATUS_SUCCESS;

	if(if_trigger) {
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_TRIGGER_MODE, 
							GX_TRIGGER_MODE_ON);	
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_TRIGGER_SWITCH, 
							GX_TRIGGER_SWITCH_ON);
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_TRIGGER_ACTIVATION, 
							GX_TRIGGER_ACTIVATION_RISINGEDGE);
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_TRIGGER_SOURCE, 
							GX_TRIGGER_SOURCE_LINE0);
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_TRIGGER_SELECTOR, 
							GX_ENUM_TRIGGER_SELECTOR_FRAME_START);
	} else {
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_TRIGGER_MODE, 
							GX_TRIGGER_MODE_OFF);
	}
	GX_VERIFY(status);
}

GX_STATUS GxCamera::setWhiteBalanceOn(bool whiteBalance,GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;

	if (whiteBalance) {
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_BALANCE_RATIO_SELECTOR, 
							GX_BALANCE_RATIO_SELECTOR_GREEN);

		// Setting the auto white balance region of interest (entire roi)
		status = GXSetInt(camHandle_, GX_INT_AWBROI_WIDTH, roiParam.width);
		status = GXSetInt(camHandle_, GX_INT_AWBROI_HEIGHT, roiParam.height);
		status = GXSetInt(camHandle_, GX_INT_AWBROI_OFFSETX, roiParam.offsetX);
		status = GXSetInt(camHandle_, GX_INT_AWBROI_OFFSETY, roiParam.offsetY);
		GX_VERIFY(status);

		// Defaults to an adaptive light source
		status = GXSetEnum(	camHandle_,
							GX_ENUM_AWB_LAMP_HOUSE, 
							GX_AWB_LAMP_HOUSE_ADAPTIVE);
		GX_VERIFY(status);

		// Default is continuous auto white balance
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_BALANCE_WHITE_AUTO, 
							GX_BALANCE_WHITE_AUTO_CONTINUOUS);
		GX_VERIFY(status);
	} else {
		// Turn off auto white balance
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_BALANCE_WHITE_AUTO, 
							GX_BALANCE_WHITE_AUTO_OFF);
		GX_VERIFY(status);
	}
	return status;
}

void GxCamera::setRoiParam(	int64_t width, int64_t height, 
							int64_t offsetX, int64_t offsetY) {
	roiParam.width 	 = width;
	roiParam.height  = height;
	roiParam.offsetX = offsetX;
	roiParam.offsetY = offsetY;
}

void GxCamera::setExposureParam(double exposureTimeUs, 
								bool   autoExposure, 
								double autoExposureTimeMinUs, 
								double autoExposureTimeMaxUs) {
	exposureParam.exposureTimeUs 		= exposureTimeUs;
	exposureParam.autoExposure 			= autoExposure;
	exposureParam.autoExposureTimeMinUs = autoExposureTimeMinUs;
	exposureParam.autoExposureTimeMaxUs = autoExposureTimeMaxUs;
}

void GxCamera::setGainParam(double gainDb, bool autoGain, 
							double autoGainMinDb, double autoGainMaxDb) {
	gainParam.gainDb 		= gainDb;
	gainParam.autoGain 		= autoGain;
	gainParam.autoGainMinDb = autoGainMinDb;
	gainParam.autoGainMaxDb = autoGainMaxDb;
}

void GxCamera::onFrameCallback(const ros::TimerEvent&) {
	// printf("========== Publishing! ==========\n");
	GX_STATUS status = GX_STATUS_SUCCESS;
	cv::Mat image;

	status = GXGetImage(camHandle, &gxFrame, 5000000);
	if(!GX_SUCCESS(status)) {
		getErrorString(status);
		printf("Failed to get image!\n");
	}

	DX_PIXEL_COLOR_FILTER bayer_type;
	switch (gxFrame.nPixelFormat) {
		case GX_PIXEL_FORMAT_BAYER_GR8: 
			bayer_type = BAYERGR; break;
		case GX_PIXEL_FORMAT_BAYER_RG8: 
			bayer_type = BAYERRG; break;
		case GX_PIXEL_FORMAT_BAYER_GB8: 
			bayer_type = BAYERGB; break;
		case GX_PIXEL_FORMAT_BAYER_BG8: 
			bayer_type = BAYERBG; break;
		default: 
			printf("Unsupported Bayer layout: %d!", gxFrame.nPixelFormat); return;
	}

	imageMsg.header.stamp = ros::Time::now();
	imageMsg.header.frame_id = "camera";
	imageMsg.encoding = "rgb8";
	imageMsg.width = gxFrame.nWidth;
	imageMsg.height = gxFrame.nHeight;
	imageMsg.step = gxFrame.nWidth * 3;
	imageMsg.data.resize(imageMsg.width * imageMsg.height * 3);
	status = DxRaw8toRGB24(	gxFrame.pImgBuf, imageMsg.data.data(),
                            gxFrame.nWidth, gxFrame.nHeight,
                        	RAW2RGB_NEIGHBOUR, bayer_type, false);

	infoMsg.header.stamp = ros::Time::now();
	camPub.publish(imageMsg, infoMsg);
}

GX_STATUS GxCamera::loadRoi(GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;

	status = GxOptimizeRoiParam(camHandle_);

	// Set an area with offset (X,Y) and size Width * Height
	status = GXSetInt(camHandle_, GX_INT_OFFSET_X, 0);
	GX_VERIFY(status);
	status = GXSetInt(camHandle_, GX_INT_OFFSET_Y, 0);
	GX_VERIFY(status);
	status = GXSetInt(camHandle_, GX_INT_WIDTH, roiParam.width);
	GX_VERIFY(status);
	status = GXSetInt(camHandle_, GX_INT_HEIGHT, roiParam.height);
	GX_VERIFY(status);
	status = GXSetInt(camHandle_, GX_INT_OFFSET_X, roiParam.offsetX);
	GX_VERIFY(status);
	status = GXSetInt(camHandle_, GX_INT_OFFSET_Y, roiParam.offsetY);
	GX_VERIFY(status);

	return GX_STATUS_SUCCESS;
}

GX_STATUS GxCamera::loadExposure(GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;

	if (exposureParam.autoExposure == true) {
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_EXPOSURE_AUTO, 
							GX_EXPOSURE_AUTO_CONTINUOUS);
		GX_VERIFY(status);
		status = GXSetFloat(camHandle_, 
							GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, 
							exposureParam.autoExposureTimeMinUs);
		GX_VERIFY(status);
		status = GXSetFloat(camHandle_, 
							GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, 
							exposureParam.autoExposureTimeMaxUs);
		GX_VERIFY(status);
	} else {
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_EXPOSURE_AUTO, 
							GX_EXPOSURE_AUTO_OFF);
		GX_VERIFY(status);
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_EXPOSURE_MODE, 
							GX_EXPOSURE_MODE_TIMED);
		GX_VERIFY(status);
		status = GXSetFloat(camHandle_, 
							GX_FLOAT_EXPOSURE_TIME, 
							exposureParam.exposureTimeUs);
		GX_VERIFY(status);
	}
	return status;
}

GX_STATUS GxCamera::loadGain(GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;

	if (gainParam.autoGain == true) {
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_GAIN_AUTO, 
							GX_GAIN_AUTO_CONTINUOUS);
		GX_VERIFY(status);
		status = GXSetFloat(camHandle_, 
							GX_FLOAT_AUTO_GAIN_MIN, 
							gainParam.autoGainMinDb);
		GX_VERIFY(status);
		status = GXSetFloat(camHandle_, 
							GX_FLOAT_AUTO_GAIN_MAX, 
							gainParam.autoGainMaxDb);
		GX_VERIFY(status);
	} else {
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_GAIN_AUTO, 
							GX_GAIN_AUTO_OFF);
		GX_VERIFY(status);
		status = GXSetEnum(	camHandle_, 
							GX_ENUM_GAIN_SELECTOR, 
							GX_GAIN_SELECTOR_ALL);
		GX_VERIFY(status);
		status = GXSetFloat(camHandle_, 
							GX_FLOAT_GAIN, 
							gainParam.gainDb);
		GX_VERIFY(status);
	}
	return status;
}

GX_STATUS GxCamera::GxSetPixelFormat8bit(GX_DEV_HANDLE& camHandle_) {
	uint32_t  i = 0;
	int64_t   nPixelSize = 0;
	uint32_t  nEnmuEntry = 0;
	size_t    nBufferSize = 0;
	GX_STATUS emStatus = GX_STATUS_SUCCESS;
	GX_ENUM_DESCRIPTION *pEnumDescription = NULL;
	GX_ENUM_DESCRIPTION *pEnumTemp = NULL;

	// Get pixel bit depth size
	emStatus = GXGetEnum(camHandle_, GX_ENUM_PIXEL_SIZE, &nPixelSize);
	if (emStatus != GX_STATUS_SUCCESS)	return emStatus;

	// Returns directly when the judgment is 8Bit
	if (nPixelSize == GX_PIXEL_SIZE_BPP8) {
		return GX_STATUS_SUCCESS;
	}

	// Get the number of enumeration entries for the pixel formats supported by the device
	emStatus = GXGetEnumEntryNums(camHandle_, GX_ENUM_PIXEL_FORMAT, &nEnmuEntry);
	if (emStatus != GX_STATUS_SUCCESS)	return emStatus;

	// Prepare resources for obtaining enumerated values for pixel formats supported by the device
	nBufferSize = nEnmuEntry * sizeof(GX_ENUM_DESCRIPTION);
	pEnumDescription = new GX_ENUM_DESCRIPTION[nEnmuEntry];

	// Get supported enumeration values
	emStatus = GXGetEnumDescription(camHandle_, GX_ENUM_PIXEL_FORMAT, pEnumDescription, &nBufferSize);
	if (emStatus != GX_STATUS_SUCCESS) {
		// Release resources
		if (pEnumDescription != NULL) {
			delete[]pEnumDescription;
			pEnumDescription = NULL;
		}
		return emStatus;
	}

	// Iterate through the pixel formats supported by the device and set the pixel format to 8Bit
	pEnumTemp = pEnumDescription;
	for (i = 0; i < nEnmuEntry; i++) {
		if ((pEnumTemp->nValue & GX_PIXEL_8BIT) == GX_PIXEL_8BIT) {
			emStatus = GXSetEnum(camHandle_, GX_ENUM_PIXEL_FORMAT, pEnumTemp->nValue);
			break;
		}
		pEnumTemp++;
	}

	// Release resources
	if (pEnumDescription != NULL) {
		delete[]pEnumDescription;
		pEnumDescription = NULL;
	}

	return emStatus;
}

GX_STATUS GxCamera::allocateImageBuf(GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;

	int64_t nPayLoadSize = 0;
	status = GXGetInt(camHandle_, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);
	if(status != GX_STATUS_SUCCESS && nPayLoadSize <= 0) {
		return status;
	}
	gxFrame.pImgBuf = malloc((size_t)nPayLoadSize);

	return status;
}

GX_STATUS GxCamera::readCameraParams(GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;

	// Camera color filter whether it is a color camera
	status = GXIsImplemented(camHandle_, GX_ENUM_PIXEL_COLOR_FILTER, &isColorCam);
	GX_VERIFY(status);

	// Exposure
	status = GXGetFloat(camHandle_, GX_FLOAT_EXPOSURE_TIME, &exposureParam.exposureTimeUs);
	GX_VERIFY(status);
	int64_t autoExposureValue = 0;
	status = GXGetEnum(camHandle_, GX_ENUM_EXPOSURE_AUTO, &autoExposureValue);
	exposureParam.autoExposure = autoExposureValue == GX_EXPOSURE_AUTO_OFF ? false : true;
	GX_VERIFY(status);
	status = GXGetFloat(camHandle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, &exposureParam.autoExposureTimeMinUs);
	GX_VERIFY(status);
	status = GXGetFloat(camHandle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, &exposureParam.autoExposureTimeMaxUs);
	GX_VERIFY(status);

	// Gain
	status = GXGetFloat(camHandle_, GX_FLOAT_GAIN, &gainParam.gainDb);
	GX_VERIFY(status);
	int64_t autoGainValue = 0;
	status = GXGetEnum(camHandle_, GX_ENUM_GAIN_AUTO, &autoGainValue);
	gainParam.autoGain = autoGainValue == GX_GAIN_AUTO_OFF ? false : true;
	GX_VERIFY(status);
	status = GXGetFloat(camHandle_, GX_FLOAT_AUTO_GAIN_MIN, &gainParam.autoGainMinDb);
	GX_VERIFY(status);
	status = GXGetFloat(camHandle_, GX_FLOAT_AUTO_GAIN_MAX, &gainParam.autoGainMaxDb);
	GX_VERIFY(status);

	// ROI
	status = GXGetInt(camHandle_, GX_INT_WIDTH, &roiParam.width);
	GX_VERIFY(status);
	status = GXGetInt(camHandle_, GX_INT_HEIGHT, &roiParam.height);
	GX_VERIFY(status);
	status = GXGetInt(camHandle_, GX_INT_OFFSET_X, &roiParam.offsetX);
	GX_VERIFY(status);
	status = GXGetInt(camHandle_, GX_INT_OFFSET_Y, &roiParam.offsetY);
	GX_VERIFY(status);

	return status;
}

GX_STATUS GxCamera::GxOptimizeRoiParam(GX_DEV_HANDLE& camHandle_) {
	GX_STATUS status = GX_STATUS_SUCCESS;
	GX_INT_RANGE xRange, yRange, widthRange, heightRange;
	int64_t maxWidth, maxHeight;
	status = GXGetIntRange(camHandle_, GX_INT_OFFSET_X, &xRange);
	GX_VERIFY(status);
	status = GXGetIntRange(camHandle_, GX_INT_OFFSET_Y, &yRange);
	GX_VERIFY(status);
	status = GXGetIntRange(camHandle_, GX_INT_WIDTH, &widthRange);
	GX_VERIFY(status);
	status = GXGetIntRange(camHandle_, GX_INT_HEIGHT, &heightRange);
	GX_VERIFY(status);
	status = GXGetInt(camHandle_, GX_INT_WIDTH_MAX, &maxWidth);
	GX_VERIFY(status);
	status = GXGetInt(camHandle_, GX_INT_HEIGHT_MAX, &maxHeight);
	GX_VERIFY(status);

	// Optimize width
	if(	roiParam.width > maxWidth) 
		roiParam.width = maxWidth;
	if(	roiParam.width < widthRange.nMin) 
		roiParam.width = widthRange.nMin;
	roiParam.width = (roiParam.width / widthRange.nInc) * widthRange.nInc;
	// Optimize height
	if(	roiParam.height > maxHeight) 
		roiParam.height = maxHeight;
	if(	roiParam.height < heightRange.nMin) 
		roiParam.height = heightRange.nMin;
	roiParam.height = (roiParam.height / heightRange.nInc) * heightRange.nInc;
	// Optimize x
	if(	roiParam.offsetX > maxWidth - roiParam.width) 
		roiParam.offsetX = maxWidth - roiParam.width;
	if(	roiParam.offsetX < 0) 
		roiParam.offsetX = 0;
	roiParam.offsetX = (roiParam.offsetX / xRange.nInc) * xRange.nInc;
	// Optimize y
	if(	roiParam.offsetY > maxHeight - roiParam.height) 
		roiParam.offsetY = maxHeight - roiParam.height;
	if(	roiParam.offsetY < 0) 
		roiParam.offsetY = 0;
	roiParam.offsetY = (roiParam.offsetY / yRange.nInc) * yRange.nInc;
}

void getErrorString(GX_STATUS errorStatus) {
	char *error_info = NULL;
	size_t size = 0;
	GX_STATUS emStatus = GX_STATUS_SUCCESS;
	// Get length of error description
	emStatus = GXGetLastError(&errorStatus, NULL, &size);
	if (emStatus != GX_STATUS_SUCCESS) {
		printf("<Error when calling GXGetLastError>\n");
		return;
	}
	// Alloc error resources
	error_info = new char[size];
	if (error_info == NULL) {
		printf("<Failed to allocate memory>\n");
		return;
	}
	// Get error description
	emStatus = GXGetLastError(&errorStatus, error_info, &size);
	if (emStatus != GX_STATUS_SUCCESS) {
		printf("<Error when calling GXGetLastError>\n");
	} else {
		printf("%s\n", (char*)error_info);
	}
	// Release error resources
	if (error_info != NULL) {
		delete[]error_info;
		error_info = NULL;
	}
}
