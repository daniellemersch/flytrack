/**
 \file    cvbmpwrite.cpp
 \brief   VGA (8x16) bitmap font writing class for OpenCV
 \date    August 2016
 \author  Alessandro Crespi <alessandro.crespi@epfl.ch>
*/

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <FreeImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "cvbmpwrite.h"

using namespace std;
using namespace cv;

/// Character width in pixels. Currently only tested with 8 pixels wide (1 byte) characters
static const unsigned int CHAR_WIDTH = 8;

/// Character height in pixels
static const unsigned int CHAR_HEIGHT = 16;   

/// Actual display width of a character (remaining columns are empty or copied from last defined column,
/// depending on the character -- line drawing characters get the last columns copied, whereas all other
/// characters get an empty column). This is what a normal VGA graphics card does in text mode.
static const unsigned int CHAR_DISPLAY_WIDTH = 9;

/// Number of characters in font (generally 256 for a standard VGA font)
static const unsigned int CHAR_COUNT = 256;

/// Size in bytes of the font bitmap data
static const unsigned int FONT_SIZE = CHAR_COUNT * CHAR_HEIGHT;

CvBmpWrite::CvBmpWrite()
{
  fg = Scalar(255, 255, 255);  // default foreground color: white
  bg = Scalar(0, 0, 0);        // default background color: black
  font_data = new uint8_t[FONT_SIZE];
  memset(font_data, 0, FONT_SIZE);
}

CvBmpWrite::~CvBmpWrite()
{
  delete[] font_data;
}

// see http://stackoverflow.com/a/23990392
void CvBmpWrite::rotate90(cv::Mat &img, const text_direction how)
{
  switch (how) {
    case DIR_RIGHT:
      break;
    case DIR_BOTTOM:  // rotate clockwise
      transpose(img, img);  
      flip(img, img, 1);
      break;
    case DIR_TOP:  // rotate counterclockwise
      transpose(img, img);  
      flip(img, img, 0);
      break;
    case DIR_LEFT:  // rotate 180 degrees
      flip(img, img, -1);
      break;
  }
}

bool CvBmpWrite::load_font(const char* filename)
{
  ifstream f(filename, ios::binary);
  if (!f.is_open()) {
    perror(filename);
    return false;
  }
  // loads the font data in memory
  f.read((char*) font_data, FONT_SIZE);
  if (f.gcount() != FONT_SIZE) {
    cerr << filename << ": invalid font file." << endl;
    return false;
  }
  f.close();
  // converts the 1 bit-per-pixel bitmaps of the font to OpenCV Mat objects that can be easily reused
  convert_chars();
  return true;
}

bool CvBmpWrite::write_text(cv::Mat& image, cv::Point pos, const text_direction direction, const char* str)
{
  // checks if we have a font in the Mat objects (if not, width will be 0 pixels)
  if (font[0].cols != CHAR_DISPLAY_WIDTH) {
    cerr << "No font has been loaded, unable to write text." << endl;
    return false;
  }
  // allocates a 24-bit (color) image for the complete string to be written
  Mat output(CHAR_HEIGHT, CHAR_DISPLAY_WIDTH * strlen(str), CV_8UC3);
  // copies the bitmap of each character at the right position in the output bitmap
  for (unsigned int i(0); i < strlen(str); i++) {
    const unsigned int c = (uint8_t) str[i];
    // makes sure the character is valid (must be true if CHAR_COUNT is 256)
    if (c >= 0 && c <= CHAR_COUNT) {
      font[c].copyTo(output(Rect(i * CHAR_DISPLAY_WIDTH, 0, CHAR_DISPLAY_WIDTH, CHAR_HEIGHT)));
    }
  }
  // if output image is grayscale, convert output bitmap to grayscale
  if (image.type() == CV_8UC1) {
    cvtColor(output, output, CV_RGB2GRAY);
  }
  
  // rotates the output image so the text is rotated in the right direction
  rotate90(output, direction);

  // compensates the starting position so that it is relative to the top-left of the text
  switch (direction) {
    case DIR_RIGHT:
      break;
    case DIR_TOP:
      pos.y = pos.y - output.rows;
      break;
    case DIR_BOTTOM:
      pos.x = pos.x - output.cols;
      break;
    case DIR_LEFT:
      pos.y = pos.y - output.rows;
      pos.x = pos.x - output.cols;
      break;   
  }
  
  // copy output bitmap at the right place in the destination image
  output.copyTo(image(Rect(pos, output.size())));
  return true;
}

void CvBmpWrite::set_colors(const cv::Scalar fg, const cv::Scalar bg)
{
  this->fg = fg;
  this->bg = bg;
  // regenerates the font Mat objects with the newly defined colors
  convert_chars();
}

void CvBmpWrite::convert_chars()
{
  // pointer to the bitmap of the character we are generating
  BYTE* ptr = (BYTE*) font_data;
  for (unsigned int c(0); c < CHAR_COUNT; c++) {
    // creates a FreeImage 1-bit-per-pixel (2-color) bitmap from the character
    FIBITMAP* bmp = FreeImage_ConvertFromRawBits(ptr, CHAR_WIDTH, CHAR_HEIGHT, (CHAR_WIDTH + 7) / 8, 1, 0, 0, 0, FALSE);
    // changes the palette of the bitmap to match the colors we are using
    RGBQUAD* pal = FreeImage_GetPalette(bmp);
    pal[0].rgbRed = bg[0];
    pal[0].rgbGreen = bg[1];
    pal[0].rgbBlue = bg[2];
    pal[1].rgbRed = fg[0];
    pal[1].rgbGreen = fg[1];
    pal[1].rgbBlue = fg[2];
    // converts the 2-color image to 24 bits (full color bitmap)
    FIBITMAP* bmp_24 = FreeImage_ConvertTo24Bits(bmp);
    // enlarges the color bitmap to add the additional column of the font
    FIBITMAP* bmp_24_l = FreeImage_EnlargeCanvas(bmp_24, 0, 0, (CHAR_DISPLAY_WIDTH - CHAR_WIDTH), 0, pal, 0);
    // gets a pointer to the color bitmap
    uint8_t* buffer = FreeImage_GetBits(bmp_24_l);
    // creates a OpenCV Mat object using the color bitmap of the character
    Mat m = Mat(CHAR_HEIGHT, CHAR_DISPLAY_WIDTH, CV_8UC3, buffer, FreeImage_GetPitch(bmp_24_l));
    // clones the obtained matrix so it has its own copy of the data and we can free out bitmaps
    m = m.clone();
    // for line-drawing characters (range 0xC0 to 0xDF in VGA fonts)...
    if (c >= 0xC0 && c <= 0xDF) {
      // ...copy the last defined column to the additional one(s) (i.e. 9th column when displaying a 8x16 font as 9x16)
      for (unsigned int i(CHAR_WIDTH); i < CHAR_DISPLAY_WIDTH; i++) {
        m(Rect(CHAR_WIDTH - 1, 0, 1, CHAR_HEIGHT)).copyTo(m(Rect(i, 0, 1, CHAR_HEIGHT)));
      }
    }
    font[c] = m;
    // frees all the FreeImage bitmaps we used
    FreeImage_Unload(bmp);
    FreeImage_Unload(bmp_24);
    FreeImage_Unload(bmp_24_l);
    // updates pointer for next character
    ptr += CHAR_HEIGHT;
  }
}
