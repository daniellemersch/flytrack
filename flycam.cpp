//
//  flycam.cpp
//  
//
//  Created by Danielle Mersch on 10/31/14.
//
//
#include "flycam.h"
#include <sys/types.h>
#include <sys/syscall.h> // call to system to access thread ID
#include <endian.h> // required for be32toh function that converts byte order to host system

//======================================================
//          METHODS
//======================================================


FlyCam::FlyCam( ){
  image_event = new pthread_event;
  framecount = 0;
  frames_read = 0;
  purge_count = 0;
  acq_started = false;
  pthread_mutex_init(&raw_mutex, NULL);
};

FlyCam::~FlyCam(){
   // Disconnect the camera
    FlyCapture2::Error error; // for all error messages from camera API
    error = cam.Disconnect();
    if (error != FlyCapture2::PGRERROR_OK){
        PrintError(error);
    }
    pthread_mutex_destroy(&raw_mutex);
};



//======================================================
bool FlyCam::Init(){
  // find camera on computer based on serial number and associate it with a globally unique identifier (Guid)
  FlyCapture2::Error error; // for all error messages from camera API
  error = device.GetCameraFromSerialNumber(SERIAL_NUMBER, &deviceGuid);
  if (error != FlyCapture2::PGRERROR_OK){
    PrintError(error, "GetCameraFromSerialNumber");
    return false;
  }
  // connect to camera
  error = cam.Connect(&deviceGuid);
  if (error != FlyCapture2::PGRERROR_OK){
    PrintError(error, "Camera connect");
    return false;
  }
  // get camera info
  DisplayCamInfo();
  
 // configure camera
  error = cam.GetConfiguration(&camConfig);
  if (error != FlyCapture2::PGRERROR_OK){
      PrintError(error, "GetConfiguration");
      return false;
  } 
  

  // Set the camera configuration
  error = cam.SetConfiguration(&camConfig);
  if (error != FlyCapture2::PGRERROR_OK) {
      PrintError(error, "SetConfiguration");
      return false;
  }
  
  return true;
}

//======================================================
void FlyCam::PrintError(FlyCapture2::Error error, std::string message){
  std::cerr << message << ": ";
  error.PrintErrorTrace();
}


//======================================================
// callback function: called at every image acquisition
void FlyCam::AnalyzeImage(FlyCapture2::Image* img_data,const void* ptr_to_class){

    FlyCapture2::Error error;
    FlyCam* ptr = (FlyCam*) ptr_to_class;

    // throw away the first 4 images to purge the camera buffer (can contain old images)
    if (!ptr->acq_started) {
      (ptr->purge_count)++;
      if (ptr->purge_count == 4) {
        ptr->acq_started = true;
      } else {
        return;
      }
    }

    //double t1 = time_real();
    //std::cout<<"called: "<<to_stringHP(t1,3)<<std::endl;
    // copy img_data to processedImage and rawImage
    raw_data tmp;
    tmp.timestamp = time_real();
    tmp.image = new FlyCapture2::Image;
    tmp.image->DeepCopy(img_data);
    pthread_mutex_lock(&ptr->raw_mutex);
    ptr->framecount++; 
    ptr->raw_images[ptr->framecount] = tmp;
    pthread_mutex_unlock(&ptr->raw_mutex);
    ptr->image_event->signal(); // signal image arrival
    //double t2 = time_real();
    //std::cout<<"time function: "<<t2-t1<<std::endl;   
}

//======================================================
// \brief Converts the camera timestamp that is embedded in the image to a double, holding the values in seconds
// \param data Pointer to the camera timestamp in the image data
// \return Camera timestamp with 1/8000 s resolution (cycle offset is ignored)
// \note This timestamp wraps every 128seconds
double data_to_timestamp(unsigned char* data){
  unsigned int second_count,cycle_count;

  //copies the first 7 bits of data[0] into second count
  second_count = data[0]>>1;

  // cycle count has one bit from data[0], 8 bits from data[1], and 4bits from data[2]
  // inputs are XXXXXXXA BBBBBBBB CCCCXXXX XXXXXXXX
  // output has to be ABBBBBBBBCCCC
  cycle_count = ((data[0] & 1) <<12) | (data[1] <<4) | (data[2]>>4);

  // computes raw timestamp in seconds - it will restart every 128seconds
  return (double) second_count + (double) cycle_count/8000.0;

}


//======================================================
// \brief extracts timestamp and framecount from embedded information of image (only possible if camera configued to included embedded info, use Flycap to check)
// \param data contains embedded information 
// \param image_time timestamp of acquisition (wraps every 128s) 
// \param acq_frame framenumber of acquisition
void extract_embedded_info(unsigned char* data, double& image_time, unsigned int& acq_frame){
  // timestamp is located in first 32bits of data: bits 0-6: second count (0-127), bits 7-19:cycle count (0-7999) = 1sec, bits 20-31: cycle offset (see GS3-U3 technical reference p82) 
  // framecount is located in second set of 32bits (unless other gain,shutter, brightness and/or white balance is also included in embedded information, then it is shifted)
  //char emb_tmstp = *data[1];
  //= atoi(data[2]);
  // framecounter
  /*unsigned int b1 (0), b2(0),b3 (0), b4(0);
  b1 = ((unsigned int)data[4]<<24); // shift by 3 bytes to left
  b2 = ((unsigned int)data[5]<<16); // shift by 2 bytes to left
  b3 = ((unsigned int)data[6]<<8); // shift 1 byte to left
  b4 = (unsigned int)data[7];
  acq_frame = (b1|b2|b3|b4); // sum bits*/
  acq_frame =  be32toh(*((uint32_t*)&data[4])); // replace previous bitwise operations, should be slightly faster 
  //std::cout<<"Framecounter "<<acq_frame<< "vs "<< acq_frame2 <<std::endl;
 
  //timestamp of image (raw)
  image_time = data_to_timestamp(data); 
}


//======================================================
// grabs images from FlyCapture, gives Tracker access to the images
bool FlyCam::GrabImage(cv::Mat& img_data, double& acq_time, int& frame, double& previous_image_time, double& previous_acq_time){
  
  FlyCapture2::Error error;
  FlyCapture2::Image raw8bit;
  //double t1 = time_monotonic();  

  frames_read++;
  frame = frames_read;

  bool missing;
  do {  
    pthread_mutex_lock(&raw_mutex);
    missing = (raw_images.find(frame) == raw_images.end());
    pthread_mutex_unlock(&raw_mutex);
    if (missing) {
      image_event->wait(); // wait from signal from callback
    }
  } while (missing);

  //double t2 = time_monotonic();
  //std::cout<<"time received: "<<t2<<std::endl;
  //std::cout<<"time wait callback: "<<t2-t1<<std::endl;

  pthread_mutex_lock(&raw_mutex);
  raw_data tmp = raw_images[frame];
  pthread_mutex_unlock(&raw_mutex);
  unsigned char* data = (*tmp.image).GetData(); // extract embedded information (timestamp = data[1]->data[3], framecount = data[4]->data[7])
  unsigned int acq_frame(0);
  double image_time(0.0);
  extract_embedded_info(data,image_time,acq_frame);
  double delta_t(0.0);
  if (previous_image_time < 0){ // special case for first image
    delta_t = 0;
  }else{
    delta_t = (image_time - previous_image_time); // time passed since last image, if negative, then has wrapped and need to add 128
  }
  if (delta_t < 0){
    delta_t +=128.0;
  }
  
 //  std::cout << "DBG pit = " << previous_image_time << " it " << image_time << " dt " << delta_t << " frame " << acq_frame << std::endl;
  acq_time = (previous_acq_time +delta_t);
  previous_image_time = image_time;
  previous_acq_time = acq_time;
  
  if (!tmp.image) {
    std::cerr << __FILE__ << ":" << __LINE__ << " tmp.image is NULL!" << std::endl;
    abort();
  }
  error = tmp.image->Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &raw8bit);
  delete tmp.image;
  if (error != FlyCapture2::PGRERROR_OK){
    PrintError(error, "convert to 8 bit");
    return false;
  }
  pthread_mutex_lock(&raw_mutex);
  raw_images.erase(raw_images.find(frame));
  pthread_mutex_unlock(&raw_mutex);
  cv::Mat img_grey(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, raw8bit[0], raw8bit.GetStride());
  cv::cvtColor(img_grey, img_data, CV_GRAY2RGB);
  return true;
}

//======================================================
int FlyCam::get_input_buffer_size(){
  pthread_mutex_lock(&raw_mutex);
  int size = raw_images.size();
  pthread_mutex_unlock(&raw_mutex);
  return size;
}

//======================================================
bool FlyCam::DisplayCamInfo(){
  
  FlyCapture2::Error error; // for all error messages from camera API
  error = cam.GetCameraInfo(&camInfo);
  if (error != FlyCapture2::PGRERROR_OK){
    PrintError(error, "GetCameraInfo");
    return false;
  }
   std::cout<<"*** CAMERA INFORMATION ***"<<std::endl;
   std::cout<<"Serial number - "<<camInfo.serialNumber<<std::endl;
   std::cout<<"Camera model - "<<camInfo.modelName<<std::endl;
   std::cout<<"Camera vendor - "<<camInfo.vendorName<<std::endl;
   std::cout<<"Sensor - "<<camInfo.sensorInfo<<std::endl;
   std::cout<<"Resolution - "<<camInfo.sensorResolution<<std::endl;
   std::cout<<"Firmware version - "<<camInfo.firmwareVersion<<std::endl;
   std::cout<<"Firmware build time - "<<camInfo.firmwareBuildTime<<std::endl;
   std::cout<<"**************************"<<std::endl;
   return true;
}

//======================================================
bool FlyCam::GetCameraSettings(){
  
  FlyCapture2::Error error; // for all error messages from camera API
  
  // get the current settings
  FlyCapture2::VideoMode currVideoMode;
  FlyCapture2::FrameRate currFrameRate;
  //Format7ImageSettings currFmt7Settings;
  
  FlyCapture2::Camera* pCamera = &cam;
  
  error = pCamera->GetVideoModeAndFrameRate(&currVideoMode, &currFrameRate);
  if (error != FlyCapture2::PGRERROR_OK){
    // Error
   PrintError(error, "Error getting current video mode and frame rate");
    return false;
  }
  
  //std::cout<<"*** CAMERA SETTINGS ***"<<std::endl;
  //std::cout<<"Frame rate - "<<currFrameRate<<std::endl;
  /*std::cout<<"Exposure - "<<camInfo.modelName<<std::endl;
  std::cout<<"Brightness - "<<camInfo.vendorName<<std::endl;
  std::cout<<"Gain - "<<camInfo.sensorInfo<<std::endl;
  std::cout<<"Resolution - "<<camInfo.sensorResolution<<std::endl;
  std::cout<<"Firmware version - "<<camInfo.firmwareVersion<<std::endl;
  std::cout<<"Firmware build time - "<<camInfo.firmwareBuildTime<<std::endl;
 */
  std::cout<<"**************************"<<std::endl;
  return true;
}

//======================================================
bool FlyCam::SetCameraSettings(){

  FlyCapture2::Error error; // for all error messages from camera API
  error = cam.StopCapture();
  if (error != FlyCapture2::PGRERROR_OK){
    PrintError(error, "StopCapture");
    return false;
  }
  FlyCapture2::Camera* pCamera = &cam;
  FlyCapture2::FrameRate newFrmRate((FlyCapture2::FrameRate)20);
  FlyCapture2::VideoMode newVdMode((FlyCapture2::VideoMode)20);
/*
  error = pCamera->SetVideoModeAndFrameRate(currVideoMode, newFrmRate);
  if ( error != FlyCapture2::PGRERROR_OK ){
    PrintError(error,  "Error setting video mode and frame rate.");
  }

  error = cam.StartCapture(AnalyzeImage, this);
  if (error != FlyCapture2::PGRERROR_OK){
    PrintError(error, "StartCapture");
    return false;
  }*/
  return true;
}

//======================================================
//capture images (runs in separate thread to allow interruption by keyboard)
bool FlyCam::StartCamera(){
  FlyCapture2::Error error; // for all error messages from camera API  
  error = cam.StartCapture(AnalyzeImage, this);
  if (error != FlyCapture2::PGRERROR_OK){
    PrintError(error, "StartCapture");
    return false;
  }
  return true;
}

//======================================================
bool FlyCam::StopCamera(){
  // stop capturing images
  FlyCapture2::Error error; // for all error messages from camera API
  error = cam.StopCapture();
  if (error != FlyCapture2::PGRERROR_OK){
    PrintError(error, "StopCapture");
    return false;
  }

  return true;
}
