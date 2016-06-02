#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <face_tracker/rect.h>
#include <face_tracker/templMatch.h>
#include <geometry_msgs/Point.h>

using namespace std;
using namespace cv;
using namespace cv::gpu;

// define global variables
int TEMPLATES_RECEIVED = 0;
int FRAME_W;
int FRAME_H;
int FRAME_AREA;
int NUM_FACES;
int FRAME_COUNT = 0;
float TEMPL_SCALE;
Mat FULL_FRAME;
vector<Mat> TEMPLATES;
vector<Rect> ROI_COORDS;
//Mat TEMPLATES;
//Rect ROI_COORDS;

// define template matching class to run algorithm
class yoloTemplateMatching
{
  // initialize subscribers and publishers
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  ros::Subscriber template_sub;
  ros::Publisher face_center_pub;

public:
  yoloTemplateMatching() : it(nh)
  {
	// subscribe to camera frames and templates received from YOLO
	image_sub = it.subscribe("/usb_cam/image_raw", 1,
		&yoloTemplateMatching::frameCallback, this);
	template_sub = nh.subscribe("/YOLO_templates", 1,
		&yoloTemplateMatching::templatesCallback, this);
	face_center_pub = nh.advertise<geometry_msgs::Point>("ROI_coordinate", 1);
  }

  ~yoloTemplateMatching()
  {
  }

private:
  Rect getCroppedDimensions(Rect box, float scale)
  {
	// scale box dimensions
	int newWidth = scale*box.width;
	int newHeight = scale*box.height;
	int newx = box.x - (scale - 1)*box.width/2;
	int newy = box.y - (scale - 1)*box.height/2;

	// ensure that dimensions remain within frame dimensions
        if (newx < 0) newx = 0;
        if (newy < 0) newy = 0;
        if (newx + newWidth > FRAME_W) newWidth = FRAME_W - newx;
        if (newy + newHeight > FRAME_H) newHeight = FRAME_H - newy;

	// return new dimensions
	Rect cropDimensions = Rect(newx, newy, newWidth, newHeight);
	return cropDimensions;
  }

  int checkThreshold(Mat matchingResult)
  {
	Mat checkResult = matchingResult.clone();

	// check template max and min values of template matching and compare to threshold
	float threshold = 0.3;
	double minCheck, maxCheck;
	Point minLocCheck, maxLocCheck;
	minMaxLoc(checkResult, &minCheck, &maxCheck, &minLocCheck, &maxLocCheck);

	// if threshold surpassed, return a failure flag
	if (minCheck > threshold) return 1;
	else return 0;
  }

  void runTemplateMatching()
  {
	// create local copies of full frame, templates, and ROI coordinates
	Mat full_frame = FULL_FRAME.clone();

	for (int i = 0; i < NUM_FACES; i++) {
	  Mat templates = TEMPLATES[i].clone();
	  Rect roi_coords = ROI_COORDS[i];

	  // extract search frame out of full frame using ROI coordinates
	  Mat searchFrame = full_frame.clone()(roi_coords);

	  // define matrix to contain template matching results
	  Mat matchingResult;

	  // run template matching
	  matchTemplate(searchFrame, templates, matchingResult, CV_TM_SQDIFF_NORMED);

	  // check if template matching failed to find a good result
          int TMfailure = checkThreshold(matchingResult);

	  // if template matching does not fail
	  if (TMfailure == 0 && FRAME_COUNT < 30) {

	    // normalize results between 0 and 1
	    normalize(matchingResult, matchingResult, 0, 1, NORM_MINMAX, -1, Mat());

	    // find location of best match
	    double min, max;
	    Point minLoc, maxLoc;
	    minMaxLoc(matchingResult, &min, &max, &minLoc, &maxLoc);

	    // convert template dimensions into a bounding box and expand it back
	    // to original cropped dimensions
	    Rect templ_bbox;
	    templ_bbox.width = templates.cols;
	    templ_bbox.height = templates.rows;
	    templ_bbox.x = minLoc.x + roi_coords.x;
	    templ_bbox.y = minLoc.y + roi_coords.y;

	    // get distance between old ROI coordinates and new ROI coordinates
	    int xdiff = minLoc.x - (roi_coords.width/2 - templates.cols/2);
	    int ydiff = minLoc.y - (roi_coords.height/2 - templates.rows/2);

            // apply smoother to ROI coordinate movement to reduce drift
	    int movement_thresh = 50;
	    if (xdiff > movement_thresh) xdiff = movement_thresh;
	    if (ydiff > movement_thresh) ydiff = movement_thresh;

	    // update search ROI coordinates
	    float x_momentum = 0.6;
	    float y_momentum = 0.3;
	    ROI_COORDS[i].x = roi_coords.x + xdiff + xdiff*x_momentum;
	    ROI_COORDS[i].y = roi_coords.y + ydiff + ydiff*y_momentum;

	    // make sure new ROI coordinates do not extend beyond frame dimensions
	    if (ROI_COORDS[i].x < 0) ROI_COORDS[i].x = 0;
	    if (ROI_COORDS[i].y < 0) ROI_COORDS[i].y = 0;
	    if (ROI_COORDS[i].x + ROI_COORDS[i].width > FRAME_W) ROI_COORDS[i].width = FRAME_W - ROI_COORDS[i].x;
	    if (ROI_COORDS[i].y + ROI_COORDS[i].height > FRAME_H) ROI_COORDS[i].height = FRAME_H - ROI_COORDS[i].y;

	    // convert template size back to original cropped dimensions and save bbox
	    Rect bbox = getCroppedDimensions(templ_bbox, 1/TEMPL_SCALE);

	    // draw template matched bbox
	    Point topLeftCorner = Point(bbox.x, bbox.y);
	    Point botRightCorner = Point(bbox.x + bbox.width, bbox.y + bbox.height);
	    rectangle(full_frame, topLeftCorner, botRightCorner, Scalar(0,255,255), 2);

	    // draw template matched bbox center
	    Point face_center(bbox.x + bbox.width/2, bbox.y + bbox.height/2);
	    circle(full_frame, face_center, 2, Scalar(255,0,255), 2, 8, 0);

	    // publish focal point for face tracking
	    geometry_msgs::Point focal_point;
	    focal_point.x = face_center.x;
	    focal_point.y = face_center.y;
	    //face_center_pub.publish(focal_point);
	  }

          // display full frame, search image, and template image
	  imshow("search ROI", searchFrame);
          waitKey(3);

          imshow("template frame", templates);
          waitKey(3);
	}

        imshow("face detector", full_frame);
        waitKey(3);
        return;
  }

  void frameCallback(const sensor_msgs::ImageConstPtr& msg)
  {
	cv_bridge::CvImagePtr cv_ptr;

	// convert sensor message into CV mat
	try {
	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}

	// if templates received from YOLO node, run template matching
	if (cv_ptr && TEMPLATES_RECEIVED == 1) {
	  FULL_FRAME = cv_ptr->image.clone();
FRAME_COUNT++;
	  runTemplateMatching();
	}
	return;
  }

  void templatesCallback(const face_tracker::templMatch msg)
  {
	TEMPLATES_RECEIVED = 1;
FRAME_COUNT = 0;

	int num = msg.num;

	// if faces found, define templates and ROI coordinates
	if (num != 0) {
	  TEMPL_SCALE = msg.scale;
	  NUM_FACES = num;

	  TEMPLATES.clear();
	  ROI_COORDS.clear();

	  for (int i = 0; i < num; i++) {
	  	// convert image message into cv mat
  	  	cv_bridge::CvImagePtr cv_template;
  	  	cv_template = cv_bridge::toCvCopy(msg.templates[i], sensor_msgs::image_encodings::BGR8);

		TEMPLATES.push_back(cv_template->image.clone());

       	  	// get ROI coordinates from message
  	  	face_tracker::rect rect_msg = msg.ROIcoords[i];
  	  	Rect searchROI;
  	  	searchROI.x = rect_msg.x;
  	  	searchROI.y = rect_msg.y;
  	  	searchROI.width = rect_msg.w;
  	  	searchROI.height = rect_msg.h;

	  	ROI_COORDS.push_back(searchROI);
	  }
	}
 	return;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "YOLO_template_matching");

  // get frame dimensions
  ros::param::get("/usb_cam/image_width", FRAME_W);
  ros::param::get("/usb_cam/image_height", FRAME_H);
  FRAME_AREA = FRAME_W * FRAME_H;

  yoloTemplateMatching ytm;
  ros::spin();
  return 0;
}
