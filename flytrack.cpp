//
//  flycam.cpp
//  selects multiple blobs per ROI
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
#include <vector>
#include <map>

//#include "FlyCapture2.h"
//#include "stdafx.h"

#include <opencv2/opencv.hpp>  // opencv
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <X11/Xlib.h> // for X11 thread handling

#include <unistd.h>  // usleep
#include <sys/time.h>  //

#include <sys/types.h>  // get IDs of threads
#include <sys/syscall.h>
#include <sys/prctl.h>  // set thread names
#include <signal.h> // for signal() / stop_handler

#include <ncurses.h>  // waitkey()
#include "rectedit.h" // ROI manipulations
#include "utils.h"  // various utility functions
#include <pthread.h> // enable threads
#include "pthread_event.h" // enables threat events
#include "exception.h" // enable exception handling
#include "flycam.h"
#include "data_format.h"  // format of data transmitted through sockets
#include "netutils.h" // contains includes for sockets and disable_nagle function
#include "MFC_data.h"
#include "cvbmpwrite.h"  // contains class to write txt on images (with bitmap font), needed for Virtual Ants automatic frame number detection


//using namespace std;

//const unsigned int BITRATE = 1024 * 1024 * 2; // multiple of 128 * 1024
const unsigned int FRAMERATE = 20;
const double RESOLUTION = 10.24; //nb of pixels per mm SHOULD BE ENTERED BY USER AS IT DEPENDSON CAMERA POSITION
const double IMG_SCALE = 0.5; // chose scaling so that image size is multiple of 4; choices: [0.1 0.25 0.35 0.5 0.6 0.75 0.85 1]
const int CV_CODEC = CV_FOURCC('X', 'V', 'I', 'D');   // N O T E ::: Codec and video extension depend on OS.
                                                      // for codec to work: try compiling opencv with "WITH_FFMPEG=ON
const unsigned int NB_THREAD_USE_CORR_IMG = 2; /// number of threads that access the undistorted image: blob detection thread and display_save thread
const unsigned int NB_KEYPOINT_USERS = 1;///< number of methods that access the keypoint data (organized by ROI)

const char* OPENCV_WINDOW_TITLE = "Flytracker";

//const int TIMED_WAIT = 3000; // duration of sleep between subsequent images from video file
const unsigned int BLOB_DETECT_THREADS = 4;   // count of parallel blob detection threads
const int FILELENGTH = 144000; ///< filelength of video files in frames, 2h at 20fps
const unsigned int WAIT_TIMEOUT = 1000000; // in microseconds

const uint16_t TCP_PORT = 8125; // port used for connection between Flytracker and valve controller
const uint32_t start_delay = 0; // zero immediate start, or delay in ms
const int TIMEOUT_DATA_QUERY = 300000; // maximum time (in us) to wait for event

const int NB_FLY_TUBES = 15; // number of tubes through which air flows
const int FLY_TUBE_DIAMETER = 8; // diameter of glass tubes in mm
const int FLY_TUBE_LENGTH = 180;  // length of glass tube available to fly in mm
const double PI = 3.141592653589793;
const double VELOCITY_CONST = (4.0 * RESOLUTION)/ (FLY_TUBE_DIAMETER * FLY_TUBE_DIAMETER * PI* FRAMERATE);
const double FLOW_CONVERT = 1000000/60;// correction factor to convert flowrate from L/minutes to mm3/sec

const double PEEK_D1 = 2.032; // inner diameter (mm) of thick PEEK tubing
const double PEEK_D2 = 1; //inner diameter (mm) of thin PEEK tubing
const double LENGTH_P1 = 1000; // length of common thick tubing in mm
const double LENGTH_P2 = 400; // length of common thin tubing in mm
const double DELIVERY_CONSTANT1 = PI * PEEK_D1 * PEEK_D1 * LENGTH_P1 / 4.0; // constant for calculating time need for pulse to travel through thick tubing
const double DELIVERY_CONSTANT2 = PI * PEEK_D2 * PEEK_D2 * LENGTH_P2 / 4.0; // constant for calculating time need for pulse to travel through thin tubing
const double DELIVERY_CONSTANT3 = PI * FLY_TUBE_DIAMETER * FLY_TUBE_DIAMETER * FLY_TUBE_LENGTH / 4.0;

const int PULSE_DRAW_HEIGHT = 20 * IMG_SCALE; /// height of bar representing odor pulse, always constant, depends on scaling
const int BORDER_DIST = 0; ///< distance from outer image boarder for pulse progression drawing, always constant, but scaled to 

const char* BITMAP_FONT = "/home/jefferis/git/source_code/common/vga.fon";  // path to font used in bmpwrite object

struct img_data {
  double timestamp;
  const cv::Mat* img; // raw image
};

struct img_use {
  double timestamp;
  int in_use; // gives nb of threads using it, each program decrements counter after use, when 0 it can be deleted
  const cv::Mat* img_original; // corrected image (original coming from undistort threads): only used to copy into img (done in get_img_corr)
  cv::Mat* img;   // copy of the corrected image, local to thread (copy is done to avoid locking image_mutex for long time by the threads using the image) -- thread has to delete this local copy
  pthread_mutex_t image_mutex;  // mutex for image access (image used at the same time by save thread and by blob detector)
};

struct fly_data_display{
  double timestamp;
  const std::vector <cv::KeyPoint>* data_display;
};

struct fly_data_ROI{
  double timestamp;
  const std::vector <std::vector <cv::KeyPoint> >* data_ROI;
  int in_use;
};


//======================================================
//Rotate an image: source image (src), angle in degrees, rotated image (dst)
void rotate(const cv::Mat& src, double angle, cv::Mat& dst){
  int len = std::max(src.cols, src.rows);
  cv::Point2f pt(len/2., len/2.);
  cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
  cv::warpAffine(src, dst, r, cv::Size(len, len));
}



//======================================================
/*cv::Mat Tracker::add_timestamp(const cv::Mat& small_img, int frame, double time){
    
  cv::Mat img_copy, small_rotated, small_final;
  rotate(small_img,-90,small_rotated);
  copyMakeBorder(small_rotated, img_copy, 0, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  std::string timestamp ="Frame " + to_string(frame) + " Time: " + UNIX_to_datetime(time);
  putText(img_copy, timestamp, cv::Point(100, 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0));
  rotate(img_copy,90, small_final);
  return small_final;
}*/


//======================================================
std::string timestamp_to_txt(int frame, double time){
   std::string txt ="Frame " + to_stringHP(frame,0,9,'0') + " Time: " + UNIX_to_datetime(time);
   return txt;
}


//======================================================
class Tracker{
  
  public:
  Tracker(std::string cam_dist_file, std::string ROIfile, std::string out_name, FlyCam* ptr_flycam, bool valve_controller);
    ~Tracker();
  
  // main thread that acquires image from either camera or video file, images are stored in queue
  // launches threads to process and save image
  void* acquire_image(volatile bool* state);

  // thread that takes image from queue, distorts it and puts in a vector for future processing
  static void* distortion_correction(void*);
  
  // thread that detects flies in each ROI
  // thread for blob detecting, feeds on incoming images, detects blobs, writes these to the output
  // and signals every time it has a list of kexpoints for the image
  static void* blob_detector(void*);
  
  // thread that downsamples images, saves it and displays it on the screen with the overlay from the tracking
  static void* save_display (void*);
  
  
  // thread that collects keypoints and writes them to file, interacts also with valve controller
  static void* keypoint_collector(void*);


  // thread that connects to valve controller, sends start signal and requests data and/or pulses
  static void* connect_to_valve_controller(void*);

  static volatile bool* running; // true if acquisition is running, false otherwise, used to control termination of threads

private:
  
  /// gets the next item to process, waiting for the data_reday event if nothing is available yet
  /// \param id The wanted ID -- if not in the map, waits for the data_ready event before retrying
  /// \param A data_entry structure with the data to process
  /// \warning As IDs are generated in monotonic increasing order, they must also be extracted this way, or the method will wait forever, unless running changed to false!
  /// \return True when image image successfully obtain, false if a waited timed out
  bool get_img_data(const int frame, img_data& data);
  
  
  //img_use get_img_corr(const int frame);
  bool get_img_corr(const int frame, img_use& data);

  bool get_keypoints_display(const int frame, fly_data_display& data);
  bool get_keypoints_ROI(const int frame, fly_data_ROI& data);
  
  /// decreases the reference count of a given item in the queue, and deletes the entry if count reached 0
  /// \param id The ID to process -- if not in the map, generates an error
  void release_img_corr(const int frame);
  
  /// releases keypoints
  void release_keypoints_display(const int frame);
  void release_keypoints_ROI(const int frame);
  
  // read xml data from camera calibration to extract Camera_Matrix, and Distortion_Coefficients
  void read_image_correction_data(cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
  
  // reads csv file with ID,x,y,height,width of each region of interest (ROI)
  void get_ROI(std::vector <cv::Rect>& ROI_table, int& ctr);

  // overlay the text on the image
  cv::Mat add_text(const cv::Mat& small_img, std::string txt, cv::Rect& rec, bool pulse_draw);
   
  
  // data
  std::string distortion_file; /// file with distortion coefficients and camera matrix (xml format)
  std::string ROI_file;  /// file qith coordinates of regions of interest
  std::string outputname; /// name for output files video and csv
  //std::string img_input; // name of input video file (or camera TO BE DEFINED)
  
  cv::VideoWriter* video; // writer for video output
  cv::VideoCapture image_source; // image input from video file
  FlyCam* camera;  // image input from camera
  
  std::map <int, img_data> img_map;  // map with raw images
  pthread_mutex_t acq_mutex; // mutex to control usage of img_queue
  pthread_event* acq_event; // signal each time new image is added to queue
  
  pthread_mutex_t img_mutex;
  pthread_event* corr_event; // signal each time new image is undistorted
  std::map <int, img_use> img_corr; // map with undistorted image data
  
  std::map <int, fly_data_display> blob_display; // contains frame, timestampe and vector of associated keypoints
  std::map <int, fly_data_ROI> blob_ROI; // contains frame, timestampe and vector of associated keypoints
  pthread_mutex_t keypoint_display_mutex;  // mutex to control acces to keypoint data
  pthread_mutex_t keypoint_ROI_mutex;  // mutex to control acces to keypoint data
  pthread_event* keypoint_ROI_event; // signal each time new keypoints are available
  pthread_event* keypoint_display_event; // signal each time new keypoints are available
  
  cv::Mat display_img; // display image
  pthread_mutex_t display_mutex; // mutex to control acces to display_img
  volatile bool update_image;    // true when there is a new image to display
  unsigned int displayed_frame;  // number of frame to display

  std::ofstream g;                ///< output stream for keypoints
  pthread_mutex_t output_mutex;   ///< mutex for output stream (CSV file with keypoints)
  
  // counters and mutex to enable multiple thread for each method
  volatile unsigned int frame_undistort;
  pthread_mutex_t fr_undistort_mutex;
  volatile unsigned int frame_blob;
  pthread_mutex_t fr_blob_mutex;
  volatile unsigned int frame_save;
  pthread_mutex_t fr_save_mutex;
  
  volatile unsigned int frame_collector;
  pthread_mutex_t fr_collector_mutex;
  
  static bool coupling_VC;
  /// only used when Flytracker is run in conjunction with valve_controller
  volatile unsigned int pulse_collector; 
  std::map <int, data_packet> pulse_map; // contains info on each pulse, index is nb of pulse, index is incremental
  pthread_mutex_t pulse_mutex;
  int pulse_given;
  
  double flow; 
  MFC_flows flow_data; // total flow running through system (in SLPM)
  pthread_mutex_t mutex_flow_overlay;

  CvBmpWrite bmpwriter; // class that write bitmap text on image 
};



//======================================================
//cv::Mat Tracker::display_img;
volatile bool* Tracker::running = NULL; 
bool Tracker::coupling_VC;

//======================================================

// handler to gracefully kill the program
void stop_handler(int signal)
{
  if (signal == SIGTERM) {
    std::cerr << "Terminate signal received ";
    if (Tracker::running) {
      *Tracker::running = false;
      std::cerr << ", program stop requested." << std::endl;
    } else {
      std::cerr << ", Tracker::running is NULL, aborting program." << std::endl;
      abort();
    }
  }
}

//======================================================


/// sets thread name for current thread (useful for debug)
/// \param str The name of the thread (MAX 16 characters!)
/// \param old Buffer (16 bytes) for the old name, if we want to keep it
/* ** replaced by calls to pthread_setname_np **
static void set_thread_name(const char* str, char* old = NULL)
{
  if (old != NULL) {
    prctl(PR_GET_NAME, old);
  }
  prctl(PR_SET_NAME, str);
}
*/

//======================================================
// constructor
Tracker::Tracker(std::string cam_dist_file, std::string ROIfile, std::string out_name, FlyCam* ptr_flycam, bool valve_controller){
  
  distortion_file = cam_dist_file;
  ROI_file = ROIfile;
  outputname = out_name;
  //img_input = input;
  camera = ptr_flycam;
  coupling_VC = valve_controller;	  

  acq_event = new pthread_event;
  corr_event = new pthread_event;
  keypoint_display_event = new pthread_event;
  keypoint_ROI_event = new pthread_event;

  // initialize mutexes for threads,
  pthread_mutex_init(&acq_mutex, NULL);
  pthread_mutex_init(&img_mutex, NULL);
  pthread_mutex_init(&keypoint_display_mutex, NULL);
  pthread_mutex_init(&keypoint_ROI_mutex, NULL);
  pthread_mutex_init(&display_mutex, NULL);
  
  // initialize counts and mutex for counts
  frame_undistort = 1;
  frame_blob = 1;
  frame_save = 1;
  frame_collector = 1;
  pthread_mutex_init(&fr_undistort_mutex, NULL);
  pthread_mutex_init(&fr_blob_mutex, NULL);
  pthread_mutex_init(&fr_save_mutex, NULL);
  pthread_mutex_init(&output_mutex, NULL);
  pthread_mutex_init(&fr_collector_mutex, NULL);
  pthread_mutex_init(&mutex_flow_overlay, NULL);

  // initialize pulse info
  pulse_given = 0;
  pthread_mutex_init(&pulse_mutex, NULL);

  // load font for cvbmpwriter
  bmpwriter.load_font(BITMAP_FONT);
}


//======================================================
// destructor
Tracker::~Tracker(){
  pthread_mutex_destroy(&acq_mutex);
  pthread_mutex_destroy(&img_mutex);
  pthread_mutex_destroy(&keypoint_ROI_mutex);
  pthread_mutex_destroy(&keypoint_display_mutex);
  pthread_mutex_destroy(&fr_undistort_mutex);
  pthread_mutex_destroy(&fr_blob_mutex);
  pthread_mutex_destroy(&fr_save_mutex);
  pthread_mutex_destroy(&display_mutex);
  pthread_mutex_destroy(&output_mutex);
  pthread_mutex_destroy(&fr_collector_mutex);
  pthread_mutex_destroy(&pulse_mutex);
  pthread_mutex_destroy(&mutex_flow_overlay);
}


//======================================================
cv::Mat Tracker::add_text(const cv::Mat& small_img, std::string txt, cv::Rect& rec, bool pulse_draw){
    
  cv::Mat img_copy, small_rotated;
  //rotate(small_img,-90,small_rotated);
  //copyMakeBorder(small_rotated, img_copy, 0, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  img_copy = small_img.clone();
  //putText(img_copy, txt, cv::Point(100, 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0));
  bmpwriter.write_text(img_copy, cv::Point(0, img_copy.rows), CvBmpWrite::DIR_TOP, txt.c_str());
  // draw a rectangle
  if (pulse_draw){ 
//    cout << "RECT @(" << rec.x << "," << rec.y <<") sz (" << rec.width << "," << rec.height << ")" << endl;
    rectangle(img_copy, rec, cv::Scalar(0,255,0), CV_FILLED);
  }
  //rotate(img_copy,90, small_final);
  return img_copy;
}


//======================================================
void Tracker::get_ROI(std::vector <cv::Rect>& ROI_table, int& ctr){
  std::ifstream f;
  f.open(ROI_file.c_str());
  if (!f.is_open()){
    *running = false;
    throw Exception(CANNOT_OPEN_FILE, ROI_file);
    //std::cerr<<"Unable to open file: "<<ROI_file<<std::endl;
    //return false;
  }
  while(!f.eof()){
    std::string s;
    getline(f, s);
    if(!s.empty()){
      if(s[0]!='#'){
        std::stringstream ss;
        ss.str(s);
        int ID;
        ss>>ID;
        ss.ignore(1,',');
        cv::Rect tmp;
        ss>>tmp.x;
        ss.ignore(1,',');
        ss>>tmp.y;
        ss.ignore(1,',');
        ss>>tmp.height;
        ss.ignore(1,',');
        ss>>tmp.width;
        ROI_table.push_back(tmp);
        ctr++;
      }
    }
  }
  f.close();
  if(ctr == 0){
    //std::cerr<<"No region of interest defined. Nothing to detect."<<std::endl;
    throw Exception(DATA_ERROR, "No region of interest defined. Nothing to detect.");
    //return false;
  }
}


//======================================================
void Tracker::read_image_correction_data(cv::Mat& cameraMatrix, cv::Mat& distCoeffs){
  cv::FileStorage fs(distortion_file, cv::FileStorage::READ);
  if (!fs.isOpened()){
    *running = false;
    throw Exception(CANNOT_OPEN_FILE,distortion_file);
  }
  fs["Camera_Matrix"] >> cameraMatrix;
  fs["Distortion_Coefficients"] >> distCoeffs;
  //std::cout<<"camera "<<cameraMatrix.size()<<std::endl;
  //throw Exception(CANNOT_READ_FILE, track_param->distortion_file);
  //std::cerr<<"Error: Failed to read file with camera matrix and distortion coefficients."<<std::endl;
}


//======================================================
bool Tracker::get_img_data(const int frame, img_data& data){

  bool found(false);
  img_data item;
  do {
    // locks the mutex to access the map
    pthread_mutex_lock(&acq_mutex);
    // gets an iterator to the wanted element
    std::map <int, img_data>::iterator iter = img_map.find(frame);
    // if the iterator is not the end of the map, we found what we want
    found = (iter != img_map.end());
    // if found, copies the structure from the map (before unlocking the mutex)
    if (found) {
      item = img_map[frame];
    }
    // finished with the map, unlocks the mutex
    pthread_mutex_unlock(&acq_mutex);
    // if we didn't find the element, wait for the "new data" signal before retrying
    if (!found) {
      if(!(acq_event->timed_wait(WAIT_TIMEOUT)) && !(*running)){
        return false;
      }
    }
  } while (!found);
  data = item;
  return true;
}


//======================================================
bool Tracker::get_img_corr(const int frame, img_use& data){
  bool found(false);
  img_use item;

  do {
    // locks the mutex to access the map
    pthread_mutex_lock(&img_mutex);
    // gets an iterator to the wanted element
    std::map <int, img_use>::iterator iter = img_corr.find(frame);
    // if the iterator is not the end of the map, we found what we want
    found = (iter != img_corr.end());
    // if found, copies the structure from the map (before unlocking the mutex)
    if (found) {
      item = img_corr[frame];
    }
    // finished with the map, unlocks the mutex
    pthread_mutex_unlock(&img_mutex);
    // if we didn't find the element, wait for the "new data" signal before retrying
    if (!found) {
      if (!(corr_event->timed_wait(WAIT_TIMEOUT)) && !(*running)){
        return false;
      }
    }
  } while (!found);

  if (pthread_mutex_lock(&item.image_mutex) != 0) {
    std::cerr << "Internal error, unable to lock mutex, self-destruct" << std::endl;
    abort();
  }
  item.img = new cv::Mat(*item.img_original);
  pthread_mutex_unlock(&item.image_mutex);
  data = item;
  return true;
}


//======================================================
bool Tracker::get_keypoints_display(const int frame, fly_data_display& data){
  bool found(false);
  fly_data_display item;
  do {
    // locks the mutex to access the map
    pthread_mutex_lock(&keypoint_display_mutex);
    // gets an iterator to the wanted element
    std::map <int, fly_data_display>::iterator iter = blob_display.find(frame);
    // if the iterator is not the end of the map, we found what we want
    found = (iter != blob_display.end());
    // if found, copies the structure from the map (before unlocking the mutex)
    if (found) {
      item = blob_display[frame];
    }
    // finished with the map, unlocks the mutex
    pthread_mutex_unlock(&keypoint_display_mutex);
    // if we didn't find the element, wait for the "new data" signal before retrying
    if (!found) {
      if (!(keypoint_display_event->timed_wait(WAIT_TIMEOUT)) && !(*running)){
        return false;
      }
    }
  } while (!found);
  data = item;
  return true;
}

//======================================================
bool Tracker::get_keypoints_ROI(const int frame, fly_data_ROI& data){
  bool found(false);
  fly_data_ROI item;
  do {
    // locks the mutex to access the map
    pthread_mutex_lock(&keypoint_ROI_mutex);
    // gets an iterator to the wanted element
    std::map <int, fly_data_ROI>::iterator iter = blob_ROI.find(frame);
    // if the iterator is not the end of the map, we found what we want
    found = (iter != blob_ROI.end());
    // if found, copies the structure from the map (before unlocking the mutex)
    if (found) {
      item = blob_ROI[frame];
    }
    // finished with the map, unlocks the mutex
    pthread_mutex_unlock(&keypoint_ROI_mutex);
    // if we didn't find the element, wait for the "new data" signal before retrying
    if (!found) {
      if (!(keypoint_ROI_event->timed_wait(WAIT_TIMEOUT)) && !(*running)){
        return false;
      }
    }
  } while (!found);
  data = item;
  return true;
}

//======================================================
void Tracker::release_img_corr(const int frame){
  // locks the mutex to access the map
  pthread_mutex_lock(&img_mutex);
  // gets an iterator to the wanted element
  std::map <int, img_use>::iterator iter = img_corr.find(frame);
  // checks if item actually exists
  if (iter == img_corr.end()) {
    pthread_mutex_unlock(&img_mutex);
    std::cerr << "Internal error: item not found in map (ID " << frame << ")" << std::endl;
    return;
  }
  // decrease reference count of image
  img_corr[frame].in_use--;
  // if no more references, deletes the data and removes the item from the map
  if (img_corr[frame].in_use == 0) {
    //std::cout << "* Ref. count for item " << frame << " is 0, deleting." << std::endl;
    delete img_corr[frame].img_original;
    pthread_mutex_destroy(&img_corr[frame].image_mutex);
    img_corr.erase(iter);
    //std::cout << "* Items in map = " << img_corr.size() << std::endl;
  }
  // done with the map, unlocks the mutex
  pthread_mutex_unlock(&img_mutex);
}


//======================================================
void Tracker::release_keypoints_display(const int frame){
  // locks the mutex to access the map
  pthread_mutex_lock(&keypoint_display_mutex);
  // gets an iterator to the wanted element
  std::map <int, fly_data_display>::iterator iter = blob_display.find(frame);
  // checks if item actually exists
  if (iter == blob_display.end()) {
    pthread_mutex_unlock(&keypoint_display_mutex);
    std::cerr << "Internal error: item not found in map (ID " << frame << ")" << std::endl;
    return;
  }
  delete blob_display[frame].data_display;
  blob_display.erase(iter);
  // done with the map, unlocks the mutex
  pthread_mutex_unlock(&keypoint_display_mutex);
}

//======================================================
void Tracker::release_keypoints_ROI(const int frame){
  // locks the mutex to access the map
  pthread_mutex_lock(&keypoint_ROI_mutex);
  // gets an iterator to the wanted element
  std::map <int, fly_data_ROI>::iterator iter = blob_ROI.find(frame);
  // checks if item actually exists
  if (iter == blob_ROI.end()) {
    pthread_mutex_unlock(&keypoint_ROI_mutex);
    std::cerr << "Internal error: item not found in map (ID " << frame << ")" << std::endl;
    return;
  }
  // decrease reference count of image
  blob_ROI[frame].in_use--;
  // if no more references, deletes the data and removes the item from the map
  if (blob_ROI[frame].in_use == 0) {
    //std::cout << "* Ref. count for item " << frame << " is 0, deleting." << std::endl;
    delete blob_ROI[frame].data_ROI;
    blob_ROI.erase(iter);
  }  
  // done with the map, unlocks the mutex
  pthread_mutex_unlock(&keypoint_ROI_mutex);
}



//======================================================
void* Tracker::acquire_image(volatile bool* state){
  
  running = state;
  // open image source: either camera or video file on disk
  //std::cout<<"Input: "<<img_input<<std::endl;
  //image_source.open(img_input);
  //if(!image_source.isOpened()){ // check if we succeeded
  //  std::string info = "Failed to open image source "+ img_input;
  //  running = false;
  //  throw Exception(NO_INPUT, info);
  //}
  
  *running = true;

  // installs signal handler callback for TERM signal (i.e. kill)
  signal(SIGTERM, stop_handler);

/*
  // get thread identifier
  pid_t tid1;
  tid1 = syscall(SYS_gettid);
  std::cout<<"acquire thread is: "<<tid1<<std::flush;
*/
  // creat threads for image processing
  pthread_t undistort_thread, save_thread, collector_thread;

  pthread_create(&undistort_thread, NULL, distortion_correction, this);
  pthread_setname_np(undistort_thread, "undistort 1");
  //pthread_detach(undistort_thread);

  pthread_t blob_thread[BLOB_DETECT_THREADS];
  for (unsigned int i(0); i < BLOB_DETECT_THREADS; i++) {
    pthread_create(&blob_thread[i], NULL, blob_detector, this);
    ostringstream os;
    os << "blob detect " << i+1;
    pthread_setname_np(blob_thread[i], os.str().c_str());
  }

  pthread_create(&save_thread, NULL, save_display, this);
  pthread_setname_np(save_thread, "save/display");
  
  pthread_create(&collector_thread, NULL, keypoint_collector, this);
  pthread_setname_np(collector_thread, "collector");
  
  //pthread_detach(save_thread);
  //std::cout<<"undistor thread is: "<<undistort_thread.gettid()<<std::endl;
  /*
  pthread_t undistort_thread2;
  pthread_create(&undistort_thread2, NULL, distortion_correction, this);
  pthread_setname_np(undistort_thread2, "undistort 2");
*/

   // create thread for socket connection with valve controller
  pthread_t valve_thread;
  if (coupling_VC){
    pthread_create(&valve_thread, NULL, connect_to_valve_controller, this);
    pthread_setname_np(valve_thread, "valve_connect");
  }
  
  int frame(0);
  double acq_time(0.0);
  cv::Mat raw_image;

  cv::namedWindow(OPENCV_WINDOW_TITLE, cv::WINDOW_AUTOSIZE);
  
  // reading images from input
  //while(image_source.read(raw_image) && running){
  double start_time = time_monotonic();
  unsigned int displayed_count(0);
  double previous_acq_time = time_real(); // start time for image acquisition
  double previous_image_time = -1.0; // no good value for initiatilisation, would need to acquire camera image to get data, so first 

  //while(camera->GrabImage(raw_image, acq_time, frame) && *running){
  while(camera->GrabImage(raw_image, acq_time, frame, previous_image_time, previous_acq_time) && *running){


    img_data image;
    cv::Mat* img = new cv::Mat(raw_image);
    image.img = img;
    image.timestamp = acq_time;
    
    // add new image to map and signal it
    pthread_mutex_lock(&acq_mutex);
    pthread_mutex_lock(&img_mutex);
    int undist_size = img_corr.size();
    int input_size = img_map.size();
    pthread_mutex_unlock(&img_mutex);
    //std::cout<<"frame "<<frame<<", displayed: " << displayed_frame << ", Δ = " << (signed)  (frame - displayed_frame) << ", in_buf = " << camera->get_input_buffer_size() << ", queued " << input_size << ", queued undistorted: " << undist_size << (char) 27 << "[K\r"<<std::flush;
    img_map[frame] = image;
    pthread_mutex_unlock(&acq_mutex);
    acq_event -> signal();
    
    //usleep(TIMED_WAIT); // 50ms sleep for testing
    
    // display image
    pthread_mutex_lock(&display_mutex);
    if (!display_img.empty() && update_image){
      imshow(OPENCV_WINDOW_TITLE, display_img);
      update_image = false;
      displayed_count++;
    }
    int k = cv::waitKey(1);
//    int k = -1;
    if (k != -1) {
      std::cout << "User pressed key: " << k << std::endl;
    }
    k = k & 0xFF;  // filters key to keep low byte (actual key without modifiers)

    if (k == 'Q' || k == 'q') {
      std::cout << "Terminating program..." << std::endl;
      *running = false ;
    }
    if (k == '!') {
      std::cerr << "Intentional program termination triggered." << std::endl;
      abort();
    }
    pthread_mutex_unlock(&display_mutex);
  }
  double stop_time = time_monotonic();
  *running = false;
  image_source.release();
  std::cout<<"Finished reading images. Read a total of "<<frame<<" images. Terminating program."<<std::endl;
  std::cout << "input frame count:  " << frame << std::endl;
  std::cout << "displayed frames:   " << displayed_count << std::endl;
  double delta_t = stop_time - start_time;
  std::cout << "time elapsed:       " << delta_t << " s" << std::endl;
  std::cout << "input frame rate:   " << frame / delta_t << " fps" << std::endl;
  std::cout << "display frame rate: " << displayed_count / delta_t << " fps" <<  std::endl;


  pthread_join(undistort_thread,NULL);
  //pthread_join(undistort_thread2,NULL);
  for (unsigned int i(0); i < BLOB_DETECT_THREADS; i++) {
    pthread_join(blob_thread[i], NULL);
  }
  pthread_join(save_thread,NULL);
  pthread_join(collector_thread,NULL);
  if (coupling_VC){
    pthread_join(valve_thread, NULL);
  }
  return NULL;
}


//======================================================
void* Tracker::distortion_correction(void* ptr_to_tracker){
  
  try{
    Tracker* track_param = (Tracker*) ptr_to_tracker;

/*
    // get thread identifier
    pid_t tid1;
    tid1 = syscall(SYS_gettid);
    std::cout<<"undistort thread is: "<<tid1<<std::endl;
*/

    // read distortion coefficients from file
    cv::Mat cameraMatrix, distCoeffs;
    track_param->read_image_correction_data(cameraMatrix, distCoeffs);
    
    bool init_undistort(false);
    cv::Mat map1, map2;

    // while acquisition is running, undistort acquired images
    while(*(track_param->running) ){
      
      pthread_mutex_lock(&(track_param->fr_undistort_mutex));
      const unsigned int frame = track_param->frame_undistort;
      img_data raw_image;
      if (!(track_param->get_img_data(frame, raw_image))){
        pthread_mutex_unlock(&(track_param->fr_undistort_mutex));
        return NULL;
      }
      track_param->frame_undistort++;
      pthread_mutex_unlock(&(track_param->fr_undistort_mutex));

      // undistort image using distortion coeficients from file
      cv::Size imageSize = raw_image.img->size();

      if (!init_undistort) {
        initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),imageSize, CV_16SC2, map1, map2);
        init_undistort = true;
      }
      
      cv::Mat corr_image;   // make sure we have a CLEAN corr_image each time!!! otherwise weird stuff happens (unaligned keypoints, or mixedup images)
      remap(*(raw_image.img), corr_image, map1, map2, cv::INTER_LINEAR);

      // delete raw_image and element from queue because not needed anymore
      pthread_mutex_lock(&(track_param->acq_mutex));
      delete raw_image.img;
      // gets an iterator to the wanted element
      std::map <int, img_data>::iterator iter = track_param->img_map.find(frame);
      // checks if item actually exists
      if (iter == track_param->img_map.end()) {
        pthread_mutex_unlock(&(track_param->acq_mutex));
        std::string info = "Internal error: item not found in map (ID " + to_string(frame) + ")";
        throw Exception(INTERNAL_ERROR, info);
      }

      track_param->img_map.erase(iter); // remove element with image data from map
      pthread_mutex_unlock(&(track_param->acq_mutex));

      // create corrected img_use data for further processing by other threads
      img_use undistorted;
      undistorted.timestamp = raw_image.timestamp;
      cv::Mat* img = new cv::Mat(corr_image);
      undistorted.img_original = img;
      undistorted.img = NULL;
      undistorted.in_use = NB_THREAD_USE_CORR_IMG;
      pthread_mutex_init(&undistorted.image_mutex, NULL);

      // add corrected imaged to map for processing, and signal event
      pthread_mutex_lock(&(track_param->img_mutex));
      track_param->img_corr[frame] = undistorted;
      pthread_mutex_unlock(&(track_param->img_mutex));

      track_param->corr_event->signal();
  
    }
    
    std::cout<<"no more images to undistort. "<<std::endl;
    return NULL;
  }catch(Exception e){
    *running = false;
    return NULL;
  }
}


//======================================================
void* Tracker::blob_detector(void* ptr_to_tracker){
  
  try {

/*
    // get thread identifier
    pid_t tid1;
    tid1 = syscall(SYS_gettid);
    std::cout<<"blob thread is: "<<tid1<<std::endl;
*/
    Tracker* track_param = (Tracker*) ptr_to_tracker;
    
    // read list of coordinates for rectangles from file
    std::vector <cv::Rect> ROI_table;
    int nb_ROI(0);
    track_param->get_ROI(ROI_table, nb_ROI);
    
    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;
    // Change thresholds: grey scale thresholds
    params.filterByColor = true;
    params.minThreshold = 0;
    params.maxThreshold = 170;
    // Filter by Area: nb pixel of blob area
    params.filterByArea = true;
    params.minArea = 50;
    params.maxArea = 350;
    // Filter by Circularity: how circular: circle = 1, square = 0.785, etc
    params.filterByCircularity = true;
    params.minCircularity = 0.3;
    // Filter by Convexity: how convex/concave the shape is
    params.filterByConvexity = false;
    params.minConvexity = 0.9;
    // Filter by Inertia: circle = 1, ellipse = between 0 and 1, line = 0
    // the lower the ratio, the more elongated the ellipse
    params.filterByInertia = false;
    params.minInertiaRatio = 0.8;
    
    // Set up the detector with parameters.
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // while acquisition is running, process input
    while(*(track_param->running)){
      
      pthread_mutex_lock(&(track_param->fr_blob_mutex));
      const unsigned int frame = track_param->frame_blob;
      img_use corr_image;
      if (!(track_param -> get_img_corr(frame, corr_image))){
        pthread_mutex_unlock(&(track_param->fr_blob_mutex));
        return NULL;
      }
      track_param->frame_blob++;
      pthread_mutex_unlock(&(track_param->fr_blob_mutex));
      
      std::vector <cv::KeyPoint>* converted_keypoints = new std::vector <cv::KeyPoint>;
      std::vector <std::vector <cv::KeyPoint> >* ROI_keypoints = new std::vector <std::vector <cv::KeyPoint> >;

      // find blobs
      for (int i(0); i< nb_ROI; i++){
        // extract region of interest
        cv::Mat img_segment = (*(corr_image.img))(ROI_table[i]);
        
        // Detect blobs in a image segment
        std::vector <cv::KeyPoint> keypoints;
        detector->detect(img_segment, keypoints);
        // unsigned int nb_blobs = keypoints.size();
        // select blob that corresponds to fly: only 1 blob per ROI
        // select biggest blob that is closest to previous blob, if no previous blob then take biggest blob
        
        // convert keypoints relative to original image and write to output
        //std::cout<<"In ROI "<<i<<": "<<nb_blobs<<" blobs found."<<std::endl;

        int blob=0;
        std::vector< cv::KeyPoint> current_ROI_keypoints;
        for (unsigned int k(0); k < keypoints.size(); k++){
          cv::KeyPoint tmp;
          tmp.pt.x = keypoints[k].pt.x + ROI_table[i].x;
          tmp.pt.y = keypoints[k].pt.y + ROI_table[i].y;
          tmp.size = keypoints[k].size;
          //std::cout<<"real: "<<i<<","<<blob <<","<< tmp.pt.x << "," << tmp.pt.y <<std::endl;
          
          // convert keypoint to scale of downsampled image and add to vector 
          cv::KeyPoint display;
          display.pt.x = (int)(tmp.pt.x * IMG_SCALE);
          display.pt.y = (int)(tmp.pt.y * IMG_SCALE);
          display.size = tmp.size;
          //std::cout<<"scaled: "<<i<<","<<blob <<","<< display.pt.x << "," << display.pt.y << std::endl;
          current_ROI_keypoints.push_back(tmp); // corrected: needs to be high resolution position 
          converted_keypoints->push_back(display);
          blob++;
          
        }
        ROI_keypoints->push_back(current_ROI_keypoints);
      }

      // add vector of keypoints of this image to queue of keypoints
      fly_data_display flies_keypoints;
      flies_keypoints.timestamp = corr_image.timestamp;
      flies_keypoints.data_display = converted_keypoints;
      pthread_mutex_lock(&(track_param->keypoint_display_mutex));
      track_param->blob_display[frame] = flies_keypoints;
      pthread_mutex_unlock(&(track_param->keypoint_display_mutex));
      track_param -> keypoint_display_event->signal();
      
      // add vector of keypoints of this image to queue of keypoints organized by ROI
      fly_data_ROI flies_keypoints_ROI;
      flies_keypoints_ROI.timestamp = corr_image.timestamp;
      flies_keypoints_ROI.data_ROI = ROI_keypoints;
      flies_keypoints_ROI.in_use = NB_KEYPOINT_USERS;
      pthread_mutex_lock(&(track_param->keypoint_ROI_mutex));
      track_param->blob_ROI[frame] = flies_keypoints_ROI;
      pthread_mutex_unlock(&(track_param->keypoint_ROI_mutex));
      track_param -> keypoint_ROI_event->signal();


      // free image from thread
      track_param->release_img_corr(frame);
      // deletes local copy of the image
      delete corr_image.img;
      
    }
    std::cout<<"finished blob finding"<<std::endl;
    
    return NULL;
  }catch(Exception e){
    *running = false;
    return NULL;
  }
}


//======================================================
void* Tracker::keypoint_collector(void* ptr_to_tracker){
  try{
    Tracker* track_param = (Tracker*) ptr_to_tracker;
    
    // open file for output
    std::string datetime = UNIX_to_datetime(time_real(), "%Y%m%d-%H%M"); // format is YYYYMMDD-HHMM
    std::string csvname = track_param->outputname + "-" + datetime + ".csv";
    
    (track_param->g).open(csvname.c_str());
    if(!((track_param->g).is_open())){
      //std::cerr<<"Error: Failed to open file for data output."<<std::endl;
      throw Exception( CANNOT_OPEN_FILE, csvname);
    }
    // header for output
    (track_param->g)<<"#timestamp,frame,ROI,nb_blobs,blobID,X,Y,size[blobID,X,Y,size[...]]"<<std::endl;
    

    std::vector <cv::Rect> ROI_table;
    int nb_ROI(0);
    track_param->get_ROI(ROI_table, nb_ROI);
    
    
    while(*(track_param->running) ){
      
      
      // get framenumber
      pthread_mutex_lock(&(track_param->fr_collector_mutex));
      fly_data_ROI fly_locations;
      const unsigned int frame = track_param->frame_collector;
      track_param->frame_collector++;
      if (!(track_param->get_keypoints_ROI(frame,fly_locations))){
        pthread_mutex_unlock(&(track_param->fr_collector_mutex));
        return NULL;
      }
      pthread_mutex_unlock(&(track_param->fr_collector_mutex));
      
      
      // write to output
      pthread_mutex_lock(&track_param->output_mutex);
      for (int i(0); i<nb_ROI; i++){
        unsigned int nb_blobs = (*fly_locations.data_ROI)[i].size();
        (track_param->g)<<to_stringHP(fly_locations.timestamp, 3)<<","<<frame<<","<<i<<","<<nb_blobs;
        for (unsigned int k(0); k < nb_blobs; k++){
          (track_param->g)<<","<<k <<","<< (*fly_locations.data_ROI)[i][k].pt.x << "," << (*fly_locations.data_ROI)[i][k].pt.y <<","<< (*fly_locations.data_ROI)[i][k].size;
        }
        (track_param->g)<<std::endl;
      }
      pthread_mutex_unlock(&track_param->output_mutex);
      track_param->release_keypoints_ROI(frame);
     
    }
    (track_param->g).close();  // closes keypoint outfile
    
  }catch(Exception e){
    *running = false;
    return NULL;
  }

  return NULL;
}


//======================================================
void check_boundary(double& value, double offset, int lower_boundary, int upper_boundary){
  if ((value + offset) < lower_boundary){
    value = lower_boundary - offset;
  }else if ((value + offset) > upper_boundary){
    value = upper_boundary - offset;
  }
}

//======================================================
void* Tracker::save_display(void* ptr_to_tracker){
  
  try{

/*
    // get thread identifier
    pid_t tid1;
    tid1 = syscall(SYS_gettid);
    std::cout<<"save thread is: "<<tid1<<std::endl;
*/

    Tracker* track_param = (Tracker*) ptr_to_tracker;

    // open video writer for output:
    cv::Size small_size;
    small_size.width = small_size.height = (int)(IMG_WIDTH * IMG_SCALE);
    std::cout<<"Scaled image size: "<<small_size.height<<", "<<small_size.width<<std::endl;
    track_param->video = new cv::VideoWriter;
    std::string datetime = UNIX_to_datetime(time_real(), "%Y%m%d-%H%M"); // format is YYYYMMDD-HHMM
    std::string video_name = track_param->outputname + "-" + datetime + ".avi";
    if (!(*(track_param->video)).open(video_name, CV_CODEC, FRAMERATE, small_size)) {
      delete(track_param->video);
      throw Exception(CANNOT_OPEN_FILE, "Unable to initialize the OpenCV video writer.");
      //std::cerr << "Unable to initialize the OpenCV video writer." << std::endl;
      //return NULL;
    }
    
//    int frame_count(0);
    int frames_written (0);
    int pulse_overlay (1);// pulse ID that is searched in map
    int current_pulse(0);
    data_packet pulse;
    bool next_pulse(true); // true if pulse data can be received
    bool pulse_found (false); //true if pulse data for overlay found
    double expire(0.0); ///timestamp at which the pulse info should no be displayed anymore
    double valid(0.0); /// timestamp at which the pulse info should be displayed
    double pulse_entry_time(0.0); /// timestamp at which the pulse has entirely entered tube
    bool pulse_draw (false); /// whether or not the pulse progression should be drawn on the image 
    
    // read list of coordinates for ROI rectangles from file, used as reference position for pulse display
    std::vector <cv::Rect> ROI_table;
    int nb_ROI(0);
    track_param->get_ROI(ROI_table, nb_ROI);
    
    double pulse_entry_pos = ROI_table[0].x;
    double pulse_y_pos = BORDER_DIST; // find tube on outer side -> odor pulse will be display at outer side of tube
    //double pulse_x_pos = ROI_table[0].x; // old version
    double pulse_exit_pos = ROI_table[0].x;
    cout<<"before scan, pos y :"<<pulse_y_pos<<endl;

    for (unsigned int i(1); i< ROI_table.size(); i++){
      if (ROI_table[i].x < pulse_entry_pos){ // find smallest y position = tube entry
        pulse_entry_pos = ROI_table[i].x;
      }
      // below code replaced by (IMG_WIDTH - BORDER_DIST) above
      /*if ((ROI_table[i].x + ROI_table[i].width) > pulse_x_pos){  // find tube on outer side -> odor pulse will be display at outer side of tube
        pulse_x_pos = (ROI_table[i].x + ROI_table[i].width);  
      }*/
      if (ROI_table[i].x + ROI_table[i].width > pulse_exit_pos){
        pulse_exit_pos = ROI_table[i].x + ROI_table[i].width; // find largest y position = tube exit
      }
    }

    // correct pixel positions for image scaling
    pulse_entry_pos *= IMG_SCALE;
    pulse_y_pos *= IMG_SCALE;
    pulse_exit_pos *= IMG_SCALE;
    
    double pulse_x = pulse_entry_pos; // position of pulse in image display at entry point of tube
    double pulse_w = 0; /// length of bar, represents odor pulse at entry
    cv::Rect pulse_rec;  /// used to draw representation of pulse on video
    pulse_rec.x = pulse_x;
    pulse_rec.width = pulse_w;
    pulse_rec.y = pulse_y_pos;  // constant value
    pulse_rec.height = PULSE_DRAW_HEIGHT; // constant value

    while(*(track_param->running)){
    
      if(frames_written == FILELENGTH){
        datetime = UNIX_to_datetime(time_real(), "%Y%m%d-%H%M"); // format is YYYYMMDD-HHMM
        video_name = track_param->outputname + "-" + datetime + ".avi";
        if (!(*(track_param->video)).open(video_name, CV_CODEC, FRAMERATE, small_size)) {
          delete(track_param->video);
          throw Exception(CANNOT_OPEN_FILE, "Unable to initialize the OpenCV video writer.");
          //std::cerr << "Unable to initialize the OpenCV video writer." << std::endl;
          //return NULL;
        }
        frames_written=0; // reset
      }
      
      pthread_mutex_lock(&(track_param->fr_save_mutex));
      // get undistorted image
      const unsigned int frame = track_param->frame_save;
      img_use corr_image;
      if (!(track_param -> get_img_corr(frame, corr_image))){
        pthread_mutex_unlock(&(track_param->fr_save_mutex));
        delete(track_param->video);
        return NULL;
      }
      track_param->frame_save++;
      pthread_mutex_unlock(&(track_param->fr_save_mutex));
//    frame_count = frame;
      
      // make downsampled copy of image
      cv::Mat small_img;
      cv::resize(*(corr_image.img), small_img, small_size);
      
      // deletes the local copy of the image
      delete corr_image.img;

      // free undistorted image from thread
      track_param->release_img_corr(frame);
      
      // get keypoints from blob detection
      fly_data_display fly_locations;
      if (!(track_param -> get_keypoints_display(frame, fly_locations))){
        return NULL;
      }
      
      std::string time_overlay = timestamp_to_txt(frame, fly_locations.timestamp);
      std::string txt_overlay, flow_overlay;

      // if the Fly tracker is coupled to the valve controller, then we overlay also the flow and odor information
      std::string odor_overlay ("");  // only used with valve controller
      if (coupling_VC){
        
        // get flow information
        double flow_tmp;
        pthread_mutex_lock(&(track_param->mutex_flow_overlay));
        flow_tmp = track_param->flow;
        pthread_mutex_unlock(&(track_param->mutex_flow_overlay));
        //cout<<"flow for overlay is: "<<flow_tmp<<endl;
        // add flow rate to image
        flow_overlay = " Flow: " + to_stringHP(flow_tmp,3);
        const double total_flow = NB_FLY_TUBES * flow_tmp;
        
        // check if new pulse data
        if (next_pulse){
          //cout<<"searching next pulse "<<pulse_overlay<<endl;
          valid = 0;
          expire = 0;
          pthread_mutex_lock(&(track_param->pulse_mutex));
          std::map <int, data_packet>::iterator iter = (track_param->pulse_map).find(pulse_overlay);
          if (iter != (track_param->pulse_map).end()){
            //cout<<"pulse found"<<endl;
            pulse = iter->second;
            pulse_found = true;
            current_pulse = pulse_overlay; //copy pulse id to be able to erase element from iterator
          }/*else{
            cout<<"Pulse not found."<<endl;
          }*/
          pthread_mutex_unlock(&(track_param->pulse_mutex));
          if (pulse_found){ // if we found the pulse to overlay
            next_pulse = false; // put to false to block search for next pulse during current pulse
            //cout<<"getting pulse timing"<<endl;
            double delivery_tube1 = DELIVERY_CONSTANT1 / (total_flow * FLOW_CONVERT); // in sec
            double delivery_tube2 = DELIVERY_CONSTANT2 / (flow_tmp * FLOW_CONVERT); // in sec
            //cout << "Flow in mm3/s = " << to_stringHP(total_flow * FLOW_CONVERT, 3) << endl;
            //cout << "Delivery constant 1 (mm3) = " << to_stringHP(DELIVERY_CONSTANT1, 3) << endl;
            //cout << "Delivery times: 1 = " << to_stringHP(delivery_tube1, 3) << " s, 2 = " << to_stringHP(delivery_tube2, 3) << " s" << endl;
            valid = pulse.timestamp + delivery_tube1 + delivery_tube2;
            pulse_entry_time = valid + pulse.duration/(double)1000;
            double delivery_time = DELIVERY_CONSTANT3 / (flow_tmp * FLOW_CONVERT); // in sec
            expire = valid + pulse.duration/(double)1000 + delivery_time; // devide by 1000 because pulse.duration is in ms and timestamp in seconds
          }
        }
        //cout<<"timestamp pulse "<<to_stringHP(fly_locations.timestamp,3)<<" valid: "<<to_stringHP(valid,3)<< " expire: "<<to_stringHP(expire,3)<<endl;
        if (fly_locations.timestamp >=valid){
          
          if (fly_locations.timestamp <=expire){
            // prepare pulse info for overlay TO BE IMPLEMENTED
            odor_overlay = " " + std::string(pulse.odor);
            pulse_draw = true;
            if (fly_locations.timestamp <= pulse_entry_time){
              pulse_w = pulse_w + (flow_tmp * FLOW_CONVERT * VELOCITY_CONST);
            }else{
              pulse_x = pulse_x + (flow_tmp * FLOW_CONVERT * VELOCITY_CONST);
            }
            //cout << "Before checks: y = " << (int) pulse_y << ", h = " << (int) pulse_w << endl;
            check_boundary(pulse_x, 0, pulse_entry_pos, pulse_exit_pos);
            check_boundary(pulse_w, pulse_x, pulse_entry_pos, pulse_exit_pos);
            pulse_rec.x = pulse_x;
            pulse_rec.width = pulse_w;
            //cout << "After checks: y = " << (int) pulse_y << ", h = " << (int) pulse_w << endl;
            //cout << "Entry pos = " << pulse_entry_pos << ", exit = " << pulse_exit_pos << endl;

            
          }else if (pulse_found){  // current pulse is expired, erase from map
            // release pulse from map
            pthread_mutex_lock(&(track_param->pulse_mutex));
            // gets an iterator to the wanted element
            std::map <int, data_packet>::iterator iter = (track_param->pulse_map).find(current_pulse);
            // checks if item actually exists
            if (iter == (track_param->pulse_map).end()) {
              pthread_mutex_unlock(&(track_param->pulse_mutex));
              std::cerr << "Internal error: item not found in pulse map (ID " << current_pulse << ")" << std::endl;
              return NULL;
            }
            (track_param->pulse_map).erase(iter);
            pthread_mutex_unlock(&(track_param->pulse_mutex));
            next_pulse = true;
            pulse_found = false;
            pulse_overlay++;
            // reset drawing of pulse
            pulse_draw = false;
            pulse_w = 0;
            pulse_x = pulse_entry_pos;
            pulse_rec.x = pulse_x;
            pulse_rec.width = pulse_w;
          }
        }


        txt_overlay = time_overlay + flow_overlay + odor_overlay;

      }else{
        txt_overlay = time_overlay;
      }

      // rotate image, add frame number and time stamp overlay to downsampled image, rotate back
      cv::Mat img_overlay = track_param->add_text(small_img, txt_overlay, pulse_rec, pulse_draw);

      // draw keypoints on downsampled image and display
      cv::Mat ds_img_keypoints;
      drawKeypoints(img_overlay, *(fly_locations.data_display), ds_img_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      track_param->release_keypoints_display(frame);

      // write downsampled image with overlay to file
      (*(track_param->video)).write(img_overlay);
      //(*(track_param->video)).write(ds_img_keypoints);
      frames_written++;

      // update display image 
      pthread_mutex_lock(&(track_param->display_mutex));
//    std::cerr << "updated display_img at frame " << frame << endl;
      track_param->display_img = ds_img_keypoints;
      track_param->update_image = true;
      track_param->displayed_frame = frame;
      pthread_mutex_unlock(&(track_param->display_mutex));

      /*
      // freeing keypoints from thread
      delete fly_locations.data_display;
      pthread_mutex_lock(&(track_param->keypoint_mutex));
      std::map <int, fly_data>::iterator iter = track_param->fly_data_display.find(frame);
      // checks if item actually exists
      if (iter == track_param->fly_data_display.end()) {
        pthread_mutex_unlock(&(track_param->keypoint_mutex));
        std::string info = "Item not found in map (ID " + to_string(frame) + ")";
        delete(track_param->video);
        throw Exception(INTERNAL_ERROR, info);
        //std::cerr << "Internal error: item not found in map (ID " << frame << ")" << std::endl;
        //return NULL;
      }
      track_param->fly_data_display.erase(iter);
      pthread_mutex_unlock(&(track_param->keypoint_mutex));
      */
      
    }
    delete(track_param->video);
    std::cout<<"Saved a total of "<<frames_written<<" images."<<std::endl;
    return NULL;
    
  } catch (Exception e){
    *running = false;
    return NULL;
  }
}

//======================================================
// calculate flow rate that each fly experiences from flow rates of all connected MFCs
double calculate_fly_flow(const MFC_flows& flow_data){
  double flow(0.0);
  int i(0);
  do{
    if(flow_data.validity[i]){
      flow += flow_data.values[i];
    }
    i++;
  }while(i < MAX_MFC);// && flow_data.names[i]!=0);
  flow = flow/(double)NB_FLY_TUBES;
  return flow;
}


//======================================================
//   double timestamp; // timestamp of event in ns
string data_to_string(const data_packet& data){
    string tmp = data.odor;
    return tmp; 
}


//======================================================
// socket connection to communicate with valve_controller
void* Tracker::connect_to_valve_controller(void* ptr_to_tracker){
  
 try{

   Tracker* track_param = (Tracker*) ptr_to_tracker;

  //bool connected = false;
  
  // creates socket
  int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // IPv4 TCP socket, TCP to ensure reliable transfer
  // specifies destination address of socket (IP address and port)
  sockaddr_in i_addr;
  i_addr.sin_family = AF_INET;
  i_addr.sin_port = htons(TCP_PORT); //htons converts bits to correct network byte order
  i_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); //htonl converts IP address in network byte order, INADDR_LOOPBACK = 127.0.0.1 (local host)
  
  // open socket connection (if result < 0, connection failed)
  int result = connect(s,(sockaddr*)&i_addr, sizeof(i_addr));
  if (result < 0){
    perror("Connect error: Connection to valve controller failed");
    return NULL;
  }

  // disable grouping of TCP packages to minimize delays
  disable_nagle(s);
  
  // send start singal to valve_controller
  if (send(s,&start_delay,sizeof(start_delay),0) != sizeof(start_delay)){
    perror("Send error: start signal not send to valve controller");
    close(s);
    return NULL;
  }
  //connected = true;
    
  while (*(track_param->running)){
    
    // TO BE IMPLEMENTED
    // to free up processor, do conditional wait, wait is interrupted if a signal is received from a fly_event_handler or due to timeout
    //bool event_received = fly_event.timed_wait(TIMEOUT_DATA_QUERY);
    usleep(TIMEOUT_DATA_QUERY);
    bool event_received = false; 

    if (event_received){
      uint8_t query = PULSE_QUERY;
      //cout<<"Sending pulse query"<<endl;
      if (send(s,&query,sizeof(query),0) != sizeof(query)){
        perror("Send error: failed to send pulse query");
        close(s);
        return NULL;
      }
      // wait for confirmation of trigger reception
      bool triggered (false);
      if (recv(s,&triggered,sizeof(triggered),MSG_WAITALL) != sizeof(triggered)){
        perror("Receive error: failed to receive trigger confirmation.");
        close(s);
        return NULL;
      }
      if (!triggered){
        std::cerr<<"Trigger reception was not confirmed"<<std::endl;
      }
      /*if (send(s,&timestamp_fly,sizeof(timestamp_fly),0) != sizeof(timestamp_fly)){
        perror("Send error: failed to send fly timestamp");
        close(s);
        return NULL;
      }*/
    
    }else{
      uint8_t query = DATA_QUERY; 
      //cout<<"Sending data query"<<endl;
      if (send(s,&query,sizeof(query),0) != sizeof(query)){
        perror("Send error: failed to send data query.");
        close(s);
        return NULL;
      }
      bool changed(false);
      if (recv(s,&changed,sizeof(changed),MSG_WAITALL) != sizeof(changed)){
        perror("Receive error: failed to receive info on state change of valve controller.");
        close(s);
        return NULL;
      }
      if (changed){
       // cout << "changed = true" << endl;
        data_packet data;
        int i = recv(s,&data,sizeof(data),MSG_WAITALL);
        if (i != sizeof(data)){
          perror("Receive error: failed to receive updated state from valve-controller.");
          cerr << i << " bytes received." << endl;
          close(s);
          return NULL;
        }
        if (data.event_type ==1){  // if it is a start pulse event, then add to 
          track_param->pulse_given++;
          pthread_mutex_lock(&(track_param->pulse_mutex));
          track_param->pulse_map[track_param->pulse_given] = data;
          pthread_mutex_unlock(&(track_param->pulse_mutex));
          //cout<<"pulse added: "<< track_param->pulse_given<<endl;
        }
        // TO BE IMPLEMENTED: transfer pulse info to video overlay and tracking output
        // need to match timestamp to frame to
        std::string msg_for_flytracker = data_to_string(data);
        //std::cout<<msg_for_flytracker<<std::endl<<std::flush;
        
      }/*else{
        cout<<"No pulse data available."<<endl;
      }*/
      
      // request update on flow information, receives last flow values of each MFC 
      uint8_t query2 = FLOW_QUERY;
      //cout<<"Sending flow query"<<endl;

      if (send(s,&query2,sizeof(query2),0) != sizeof(query2)){
        perror("Send error: failed to send flow query.");
        close(s);
        return NULL;
      }
      
      bool MFC_changed(false);
      if (recv(s,&MFC_changed,sizeof(MFC_changed),MSG_WAITALL) != sizeof(MFC_changed)){
        perror("Receive error: failed to receive info on state change of MFC flow infofrom valve controller.");
        close(s);
        return NULL;
      }
      
      // if flow information has changed since lasted request, collect data
      if (MFC_changed){
        MFC_flows flow_data;
        if (recv(s,&flow_data,sizeof(flow_data),MSG_WAITALL) != sizeof(flow_data)){
          perror("Receive error: failed to receive updated MFC flow info from valve-controller.");
          close(s);
          return NULL;
        }
        // update MFC info for video oderlay (block mutex)
        //calculate flow experienced by individual from flow_data
        double tmp = calculate_fly_flow(flow_data);
        pthread_mutex_lock(&(track_param->mutex_flow_overlay));
        track_param->flow = tmp;
        pthread_mutex_unlock(&(track_param->mutex_flow_overlay));
      }
    
    }
    
  }
  
  return NULL;
  
  } catch (Exception e){
    *running = false;
    return NULL;
  }
}

// read parameters from input configuration file 
void read_config (std::string& configfile, std::string& cam_dist_file, std::string& ROI_file, std::string& outputname, bool& VC_present){
	bool VC_known(false);  
	ifstream f; 
	f.open(configfile.c_str());
	if (!f.is_open()){
		throw Exception(CANNOT_OPEN_FILE, configfile);
	}
	while (!f.eof()){
		string s; 
		getline(f,s);
		if (!s.empty()){
			if (s[0] != '#'){  // # indicate comment lines
				vector <string> word_table;
        int nb_words = chop_line(s, word_table);
	/*cout<<"Number of words found: "<<nb_words<<endl;
	for (int i(0); i<nb_words; i++){
	  cout<<"word["<<i<<"]="<<word_table[i]<<endl;			
	}*/
				if (nb_words != 2){
					cerr<<"Error in line: "<<s<<endl;
					throw Exception(PARAMETER_ERROR, "The number of parameters is invalid.");				
				}

				if (word_table[0]=="ROI_DATA"){
					ROI_file = word_table[1];
				}else if (word_table[0]=="CAM_CALIB"){
					cam_dist_file = word_table[1];
				}else if (word_table[0]=="OUTPUT_PATH"){
					outputname = word_table[1];
				}else if (word_table[0]=="VALVE_CONTROLLER"){
					if ((word_table[1].size() != 1) || (word_table[1][0] < '0') || (word_table[1][0] > '1')){ 
						throw Exception(PARAMETER_ERROR, "Invalid parameter for keyword VALVE_CONTROLLER. Only 0 and 1 accepted.");
					}
					VC_present = ( word_table[1][0] == '1' );					
					VC_known = true; 				
				}
			}
		}
	}
	
	if (cam_dist_file == "" || ROI_file == "" || outputname == "" || !VC_known){
		throw Exception(PARAMETER_ERROR, "Missing input parameters.");
	}

}

//======================================================
//======================================================


int main(int argc, char* argv[]){
 
   // set realtime priority on the process
  if (set_realtime()) {
    std::cout << "Successfully set realtime scheduling." << std::endl;
  }

  try{
    
    XInitThreads(); // Xlib thread initialization: otherwise problems with X11 multithreading
   
    if (argc != 2) {
      //std::string info = (std::string)argv[0] + " ROI.csv camera_distortion_data.xml input.avi output.mov outdata.txt";
      //std::string info = (std::string)argv[0] + " ROI.csv camera_distortion_data.xml output_name VC_present[0,1]";
      std::string info = (std::string)argv[0] + " configfile.txt";            
	    throw Exception (USE, info); 
    }

		std::string configfile = (std::string) argv[1];
    // read camraMatrix and distorion coefficients from file.
    std::string cam_dist_file = "";
    std::string ROIfile = "";
    //std::string img_input = (std::string) argv[3];
    std::string outputname = "";
    bool VC_present = 0;  // 0 if no coupling to valve controller, 1 if coupling to vale controller

    // read config_file
    read_config(configfile, cam_dist_file, ROIfile, outputname, VC_present);
    
    if (VC_present){
      std::cout<<"Flytracker will be coupled to Valve controller program."<<std::endl;
    }else{
      std::cout<<"Flytracker will run as stand-alone program."<<std::endl;
    }
    // need to check whether permission to write to folder, if not -> fail
    // check whether output video file exists
    /*std::ifstream f;
    f.open(outputvideo.c_str());
    if (f.is_open()){
      f.close();
      throw Exception(OUTPUT_EXISTS, outputvideo);
    }*/

    // check whether output keypoints file exists
    /*
    f.open(outputdata.c_str());
    if (f.is_open()){
      f.close();
      throw Exception(OUTPUT_EXISTS, outputdata);
    }
    */

  volatile bool running = false; // true if acquisition is running, false otherwise, used to stop threads
  FlyCam flycam;
  FlyCam* ptr_flycam = &flycam;
  
    // initialize camera
  std::cout<<"Initializing camera ..."<<std::endl;
  if (!flycam.Init()){
    throw Exception(CAMERA_ERROR, "Failed to initialize camera.");
  }
    
  std::cout<<"Starting camera ..."<<std::endl;
  if (!flycam.StartCamera()){
    throw Exception(CAMERA_ERROR, "Failed to start camera.");
  }
  if (!flycam.GetCameraSettings()){
    throw Exception(CAMERA_ERROR, "Failed to get camera settings.");
  }

/*
  // thread to intercapt keyboard event from user
  pthread_t keyboard_thread;
  pthread_create(&keyboard_thread, NULL, flycam.GetKey, (void*) &running);
  pthread_detach(keyboard_thread);
*/

  std::cout<<"Starting tracker ..."<<std::endl;
  Tracker flytrack(cam_dist_file, ROIfile, outputname, ptr_flycam, VC_present);
  flytrack.acquire_image(&running);

  std::cout<<"Stopping camera ..."<<std::endl;
  if (!flycam.StopCamera()){
    throw Exception(CAMERA_ERROR, "Failed to stop camera.");
  }
    
    return 0;
  }catch(Exception e){
    return -1;
  }
}
