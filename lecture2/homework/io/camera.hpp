#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <opencv2/core.hpp>

/**
 * A minimal wrapper around the HikRobot MVS camera API.
 *
 * Creating an instance opens the first USB camera and starts acquisition.
 * read() returns a BGR image; an empty matrix means that the SDK timed out
 * while waiting for a frame.
 */
class Camera
{
public:
  Camera();
  ~Camera();
  cv::Mat read();

private:
  Camera(const Camera &) = delete;
  Camera & operator=(const Camera &) = delete;

  void * handle_ = nullptr;
  bool opened_ = false;
  bool grabbing_ = false;
};

#endif  // IO__CAMERA_HPP
