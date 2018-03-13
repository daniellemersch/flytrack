#include "rectedit.h"
#include <sstream>
#include <iostream>

using namespace std;
using namespace cv;

const char* WINDOW_TITLE = "Rectangle Editor";

RectangleEditor::RectangleEditor(){
  status = 0;
}

RectangleEditor::~RectangleEditor(){
}

void RectangleEditor::mouse_callback_static(int event, int x, int y, int flags, void* inst){
  if (inst != NULL) {
    static_cast<RectangleEditor*>(inst) -> mouse_callback(event, x, y, flags);
  }
}

void RectangleEditor::update_overlay(const string error_msg){
  Mat img_copy;
  copyMakeBorder(img, img_copy, 0, 18, 0, 0, BORDER_CONSTANT, Scalar(0, 0, 0));
  for (unsigned int i(0); i < rects.size(); i++) {
    rectangle(img_copy, rects[i], Scalar(0, 0, 255), 2.0);
    ostringstream num;
    num << i;
    putText(img_copy, num.str(), rects[i].tl() + Point(1, 13), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 0));
  }
  if (status == 1) {
    line(img_copy, last + Point(-3, -3), last + Point(3, 3), cvScalar(0, 255, 0));
    line(img_copy, last + Point(3, -3), last + Point(-3, 3), cvScalar(0, 255, 0));
    if (error_msg.length() == 0) {
      putText(img_copy, "Define second point, or right click to cancel", Point(1, img.size().height + 13), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 0));
    }
  } else {
    if (error_msg.length() == 0) {
      putText(img_copy, "ENTER to confirm, ESC to cancel, C to clear all", Point(1, img.size().height + 13), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 0));
    }
  }
  if (error_msg.length() > 0) {
    putText(img_copy, error_msg, Point(1, img.size().height + 13), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 64, 255));
  }
  imshow(WINDOW_TITLE, img_copy);
}



bool RectangleEditor::in_any_rect(const Point p){
  for (unsigned int i(0); i < rects.size(); i++) {
    if (p.inside(rects[i])) {
      return true;
    }
  }
  return false;
}


bool RectangleEditor::would_intersect(const Rect r){
  for (unsigned int i(0); i < rects.size(); i++) {
    // tests if there is an intersection surface
    if ((r & rects[i]).area() != 0) {
      return true;
    }
  }
  return false;
}



void RectangleEditor::mouse_callback(int event, int x, int y, int flags){
  if (event == EVENT_RBUTTONDOWN) {
    // right mouse click: cancel first point if defining rectangle (i.e. status == 1)
    if (status == 1) {
      status = 0;
      update_overlay();
    }
  } else if (event == EVENT_LBUTTONDOWN && y < img.size().height) {
    // left mouse click: define rectangle point
    if (status == 0) {
      // status == 0 -> first point, check that it is not inside a defined rectangle
      if (!in_any_rect(Point(x, y))) {
        last = Point(x, y);
        status = 1;
        update_overlay();
      } else {
        update_overlay("Invalid starting point: it is inside another rectangle.");
      }
    } else if (status == 1) {
      // status == 1 -> second point of a rectangle, test for overlapping with other ones
      Rect r(last, Point(x, y));
      if (!would_intersect(r)) {
        rects.push_back(r);
        status = 0;
        update_overlay();
      } else {
        update_overlay("Invalid end point: rectangle intersects with an existing one.");
      }
    }
  }
}

vector<Rect> RectangleEditor::get_rectangles(Mat& image, const double scale){
  namedWindow(WINDOW_TITLE, WINDOW_AUTOSIZE);
  setMouseCallback(WINDOW_TITLE, mouse_callback_static, this);   // important: third parameter MUST be this (pointer to object instance)
  
  rects.clear();
  this->scale = scale;

  bool exit(false);
  if (image.channels() == 1) {
    cvtColor(image, img, CV_GRAY2RGB);
  } else {
    image.copyTo(img);
  }
  
  if (fabs(scale - 1.0) > 0.01) {
    Mat img_copy;
    img.copyTo(img_copy);
    Size sz = img.size();
    sz.width *= scale;
    sz.height *= scale;
    resize(img_copy, img, sz);
  }

  while (!exit) {
    update_overlay();
    int c = waitKey(0);
    switch (c) {
      case 27:   // ESC key
        rects.clear();
        exit = true;
        break;
      case 'C':
      case 'c':  // clear all rectangles
        rects.clear();
        update_overlay();
        break;
      case '\n':
      case '\r':
        exit = true;
        break;
    }
  }
  destroyWindow(WINDOW_TITLE);

  // scale the rectangles if needed  
  if (fabs(scale - 1.0) > 0.01) {
    for (unsigned int i(0); i < rects.size(); i++) {
      rects[i].x = rects[i].x / scale;
      rects[i].y = rects[i].y / scale;
      rects[i].width = rects[i].width / scale;
      rects[i].height = rects[i].height / scale;
    }
  }
  return rects;
}
