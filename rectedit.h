#include <vector>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

class RectangleEditor
{
public:
  RectangleEditor();
  ~RectangleEditor();
  
  std::vector<cv::Rect> get_rectangles(cv::Mat& image, const double scale = 1.0);
  
private:

  /// The actual mouse callback function, handling mouse events
  void mouse_callback(int event, int x, int y, int flags);
  
  /// OpenCV callback function for mouse events, must be static (called by OpenCV)
  /// \param inst A pointer to the instance of the RectangleEditor object
  static void mouse_callback_static(int event, int x, int y, int flags, void* inst);
  
  /// updates the overlay on the internal image (img)
  void update_overlay(const std::string error_msg = "");

  /// tests if the given point is inside any of the already defined rectangles
  bool in_any_rect(const cv::Point p);

  /// tests if the given rectangle would intersect with any existing one
  bool would_intersect(const cv::Rect r);

  /// used internally, current status and coordinates
  int status;
  cv::Point last;
  
  /// resulting rectangles
  std::vector<cv::Rect> rects;  
  
  /// temporary image with the overlaid rectangles
  cv::Mat img;
  
  /// scaling
  double scale;
};
