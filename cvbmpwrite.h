/**
 \file    cvbmpwrite.h
 \brief   8x16 bitmap font writing class (OpenCV version, compatible with fonts used by the BmpWrite class of mainsvr)
 \date    August 2016
 \author  Alessandro Crespi <alessandro.crespi@epfl.ch>
*/

#ifndef _CVBMPWRITE_H
#define _CVBMPWRITE_H

#include <opencv2/core/core.hpp>
#include <stdint.h>

class CvBmpWrite {

public:

  CvBmpWrite();
  ~CvBmpWrite();

  /// text direction
  enum text_direction { DIR_RIGHT, DIR_LEFT, DIR_TOP, DIR_BOTTOM };
  
  /// loads a 256-characters bitmap font from a file containing a 8x16 bitmap for each of the 256 possible 8-bit characters
  /// (same format as a PC BIOS VGA font)
  bool load_font(const char* filename);
  
  /// sets the foreground and background values to write text
  void set_colors(const cv::Scalar fg, const cv::Scalar bg);
  
  /// \brief write text at the given coordinates in the image. The characters are rendered on a 9x16 grid as in PC VGA text mode,
  ///   with the 8th column of the character replicated to the 9th for line-drawing characters
  /// \return true if the text has been written, false otherwise (only possible reason: no font loaded)
  bool write_text(cv::Mat& image, cv::Point pos, const text_direction direction, const char* str);

private:

  /// converts the loaded font to 9x16 OpenCV Mat objects used to render the strings
  void convert_chars();

  /// rotates an image (OpenCV Mat) by a multiple of 90 degrees
  void rotate90(cv::Mat &img, const text_direction how);
  
  /// the loaded font data
  uint8_t* font_data;

  /// selected foreground color
  cv::Scalar fg;

  /// selected background color
  cv::Scalar bg;

  /// 9x16 Mat objects containing each character in the right colors
  cv::Mat font[256];

};

#endif
