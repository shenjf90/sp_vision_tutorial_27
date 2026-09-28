#include "camera.hpp"

#include "hikrobot/include/MvCameraControl.h"

#include <stdexcept>

Camera::Camera()
{
  MV_CC_DEVICE_INFO_LIST device_list{};
  if (MV_CC_EnumDevices(MV_USB_DEVICE, &device_list) != MV_OK) {
    throw std::runtime_error("Failed to enumerate HikRobot USB cameras");
  }
  if (device_list.nDeviceNum == 0) {
    throw std::runtime_error("No HikRobot USB camera found");
  }

  if (MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]) != MV_OK) {
    handle_ = nullptr;
    throw std::runtime_error("Failed to create the HikRobot camera handle");
  }
  if (MV_CC_OpenDevice(handle_) != MV_OK) {
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    throw std::runtime_error("Failed to open the HikRobot camera");
  }
  opened_ = true;

  // Keep the parameters from the provided camera example.
  MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
  MV_CC_SetFloatValue(handle_, "ExposureTime", 10000.0F);
  MV_CC_SetFloatValue(handle_, "Gain", 20.0F);
  MV_CC_SetFrameRate(handle_, 60.0F);

  if (MV_CC_StartGrabbing(handle_) != MV_OK) {
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    opened_ = false;
    throw std::runtime_error("Failed to start HikRobot image acquisition");
  }
  grabbing_ = true;
}

Camera::~Camera()
{
  if (handle_ == nullptr) return;

  if (grabbing_) MV_CC_StopGrabbing(handle_);
  if (opened_) MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
}

cv::Mat Camera::read()
{
  MV_FRAME_OUT raw{};
  if (MV_CC_GetImageBuffer(handle_, &raw, 100) != MV_OK) return {};

  // The buffer belongs to the MVS SDK, so the converted image must be made
  // before MV_CC_FreeImageBuffer is called.
  cv::Mat image(raw.stFrameInfo.nHeight, raw.stFrameInfo.nWidth, CV_8UC3);
  MV_CC_PIXEL_CONVERT_PARAM_EX convert{};
  convert.nWidth = raw.stFrameInfo.nWidth;
  convert.nHeight = raw.stFrameInfo.nHeight;
  convert.pSrcData = raw.pBufAddr;
  convert.nSrcDataLen = raw.stFrameInfo.nFrameLen;
  convert.enSrcPixelType = raw.stFrameInfo.enPixelType;
  convert.pDstBuffer = image.data;
  convert.nDstBufferSize = static_cast<unsigned int>(image.total() * image.elemSize());
  convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

  const bool converted_ = MV_CC_ConvertPixelTypeEx(handle_, &convert) == MV_OK;
  MV_CC_FreeImageBuffer(handle_, &raw);
  return converted_ ? image : cv::Mat{};
}
