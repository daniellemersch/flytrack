//
//  flycam.cpp
//  
//
//  Created by Danielle Mersch on 10/31/14.
//
//
#include <cstring>
#include <string>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <map>

#include "FlyCapture2.h"
#include "stdafx.h"
//#include "Format7Page.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ncurses.h>
#include "utils.h"  // various utility functions
#include <pthread.h> // enable threads
#include "pthread_event.h" // enables threat events

const unsigned int IMG_WIDTH = 2048; // full camera resolution is 2048*2048 
const unsigned int IMG_HEIGHT = IMG_WIDTH;
const unsigned int SERIAL_NUMBER = 15123106; /// serial number of USB3_NIR camera (Grasshopper, Point Grey Research)

class FlyCam {

public:
  
  // constructor
  FlyCam();
  
  // destructor
  ~FlyCam();
  
  /**\brief Find camera on computer based on serial number and associate it with a globally unique identifier (Guid)
   * \param 
   * \param 
   * \return 
   */
  bool Init();
  
  /**\brief Print Point Grey Research error message
   * \param error message
   */
  void PrintError(FlyCapture2::Error error, std::string message = "");
 
  
  /**\brief Callback function called after each image acquisition
   * \param
   * \param
   * \return
   */
  static void AnalyzeImage(FlyCapture2::Image* img_data,const void* ptr_to_class);
  
  /**\brief Grabs a acquired image
   * \param
   * \param
   * \return
   */
  bool GrabImage(cv::Mat& img_data, double& acq_time, int& frame, double& previous_image_time, double&previous_acq_time);
  
  
  /**\brief
   * \param
   * \param
   * \return
   */
  bool DisplayCamInfo();

  /**\brief
   * \param
   * \param
   * \return
   */
  bool GetCameraSettings();
  
  /**\brief
   * \param
   * \param
   * \return
   */
  bool SetCameraSettings();

  /**\brief Start image capture from camera
   * \param
   * \param
   * \return
   */
  bool StartCamera();
  
  /**\brief
   * \param
   * \param
   * \return
   */
  bool StopCamera();

  /// returns the size of the input buffer
  int get_input_buffer_size();
  
private:
  
  struct raw_data {
    FlyCapture2::Image* image;
    double timestamp;
  };

  FlyCapture2::BusManager device;
  FlyCapture2::PGRGuid deviceGuid;
  
  FlyCapture2::AVIRecorder recorder;   // recorder to save images to videostream avifile (max 2GB)
  bool savevideo; // state variable, whether the images are saved to a video file
  
  FlyCapture2::Camera cam;
  FlyCapture2::CameraInfo camInfo;
  FlyCapture2::FC2Config camConfig;
  
  pthread_event* image_event; // signal each time new keypoints are available
  double timestamp;  // timestamp associated with image
  unsigned int framecount; // number of images acquired
  unsigned int frames_read;  ///< number of frames read with GrabImage
  pthread_event* stopping_event; // signal when tracking is stopped, so acquisition is stopped

  std::map<int, raw_data> raw_images;  // input buffer
  pthread_mutex_t raw_mutex;

  bool acq_started;   // whether actual frame acquisition has been started
  int purge_count;      // frame purging counter for startup
  
};
