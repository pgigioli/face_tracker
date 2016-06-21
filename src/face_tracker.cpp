#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include <iostream>
#include <string>
#include <std_msgs/Int8.h>

using namespace std;

float PAN_ZERO = 0.0;
float TILT_ZERO = -5.0;
int WAIT_TO_RECENTER = 0;
int FOUND_OBJECT = 0;

float image_width;
float image_height;
float pan_angle = PAN_ZERO;
float tilt_angle = TILT_ZERO;

// pan/tilt ranges. Pan range = 189 degress, Tilt range = 102 degrees
// 1 degree horizontally = 47.41 steps, 1 degree vertically = 37.65 steps
float pan_max = 4480/47.41; float pan_min = -4480/47.41;
float tilt_max = 1920/37.65; float tilt_min = -1920/37.65;

float panIncr5 = 237/47.41; // 5 degrees
float tiltIncr5 = 188/37.65; // 5 degrees
float panIncr10 = 2*237/47.41; // 10 degrees
float tiltIncr10 = 2*188/37.65; // 10 degrees

string tiltIncrString5 = "188";
string panIncrString5 = "237";
string tiltIncrString10 = "376";
string panIncrString10 = "474";

class faceTracker
{
  ros::NodeHandle nh;
  ros::Subscriber coord_sub;
  ros::Publisher cam_angles;
  ros::Subscriber found_object_sub;

public:
  faceTracker()
  {
	this->resetPanTilt();
	found_object_sub = nh.subscribe("/found_object", 1, &faceTracker::foundObjectCb, this);
	coord_sub = nh.subscribe("/ROI_coordinate", 1, &faceTracker::callback, this);
	cam_angles = nh.advertise<std_msgs::Float64MultiArray>("/cam_angles", 1);
  }

  ~faceTracker()
  {
	this->resetPanTilt();
  }

  void resetPanTilt()
  {
	float delay = 1.5;
	string command1 = "uvcdynctrl -s \"Tilt Reset\" 1";
	string command2 = "uvcdynctrl -s \"Pan Reset\" 1";
	system(command1.c_str());
	ros::Duration(delay).sleep();
	system(command2.c_str());
	ros::Duration(delay).sleep();
  }

private:
  void publishCamAngles(float pan_angle, float tilt_angle)
  {
	std_msgs::Float64MultiArray anglesArray;
	anglesArray.data.clear();
	anglesArray.data.push_back(pan_angle);
	anglesArray.data.push_back(tilt_angle);

	cam_angles.publish(anglesArray);
  }

  void foundObjectCb(std_msgs::Int8 msg)
  {
	if (msg.data == 1) {
	   FOUND_OBJECT = 1;
	} else if (msg.data == 0) {

	   FOUND_OBJECT = 0;
           WAIT_TO_RECENTER++;

           if (WAIT_TO_RECENTER > 5) {
              recenterCam();
           }
	}
  }

  void recenterCam()
  {
	cout << "recentering cam" << endl;
	float tilt_buffer_pos = -18.0;
	float tilt_buffer_neg = -23.0;
	float pan_buffer_pos = 3.0;
	float pan_buffer_neg = -3.0;
	float delay = 0.1;

        float tiltIncr_1 = tiltIncr5;
        float panIncr_1 = panIncr5;
        string tiltIncrString_1 = tiltIncrString5;
        string panIncrString_1 = panIncrString5;


	if (pan_angle < pan_buffer_neg) {
	  pan_angle += panIncr_1;
          string command = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString_1;
          system(command.c_str());
          ros::Duration(delay).sleep();
	}
	if (pan_angle > pan_buffer_pos) {
	  pan_angle -= panIncr_1;
	  string command = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString_1;
	  system(command.c_str());
	  ros::Duration(delay).sleep();
	}
	if (tilt_angle < tilt_buffer_neg) {
	  tilt_angle += tiltIncr_1;
	  string command = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString_1;
	  system(command.c_str());
          ros::Duration(delay).sleep();
	}
	if (tilt_angle > tilt_buffer_pos) {
	  tilt_angle -= tiltIncr_1;
	  string command = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString_1;
	  system(command.c_str());
          ros::Duration(delay).sleep();
	}

        if (tilt_angle > tilt_max) { tilt_angle = tilt_max; }
        if (tilt_angle < tilt_min) { tilt_angle = tilt_min; }
        if (pan_angle > pan_max) { pan_angle = pan_max; }
        if (pan_angle < pan_min) { pan_angle = pan_min; }

        publishCamAngles(pan_angle, tilt_angle);

	cout << "pan: " << pan_angle << endl;
        cout << "tilt: " << tilt_angle << endl;

  	return;
  }

  void callback(const geometry_msgs::Point::ConstPtr& msg)
  {
  	if (FOUND_OBJECT == 1) {
	WAIT_TO_RECENTER = 0;
	float center_x = msg->x;
  	float center_y = msg->y;
	//cout << "x: " << center_x << endl;
  	//cout << "y: " << center_y << endl;
  	//cout << " " << endl;

	float z_scale = 0.25;
  	float zone_x_min = (image_width * 0.5) - (image_width * z_scale * 0.5);
  	float zone_x_max = (image_width * 0.5) + (image_width * z_scale * 0.5);
  	float zone_y_min = (image_height * 0.5) - (image_height * z_scale * 0.5);
  	float zone_y_max = (image_height * 0.5) + (image_height * z_scale * 0.5);

  	float delay = 0.1;

	float tiltIncr_0 = tiltIncr5;
	float panIncr_0 = panIncr5;
	string tiltIncrString_0 = tiltIncrString5;
	string panIncrString_0 = panIncrString5;

  	// Zone 1
  	if (center_y < zone_y_min && center_x > zone_x_min && center_x < zone_x_max)
  	{
	  tilt_angle -= tiltIncr_0;
	  string command = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString_0;
          system(command.c_str());
          ros::Duration(delay).sleep();
  	}

  	// Zone 2
  	if (center_y > zone_y_max && center_x > zone_x_min && center_x < zone_x_max)
  	{
	  tilt_angle += tiltIncr_0;
	  string command = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString_0;
	  system(command.c_str());
          ros::Duration(delay).sleep();
  	}

  	// Zone 3
  	if (center_x < zone_x_min && center_y > zone_y_min && center_y < zone_y_max)
  	{
	  pan_angle += panIncr_0;
	  string command = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString_0;
          system(command.c_str());
          ros::Duration(delay).sleep();
  	}

  	// Zone 4
  	if (center_x > zone_x_max && center_y > zone_y_min && center_y < zone_y_max)
  	{
	  pan_angle -= panIncr_0;
	  string command = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString_0;
          system(command.c_str());
          ros::Duration(delay).sleep();
  	}

  	// Zone 5
  	if (center_y < zone_y_min && center_x < zone_x_min)
  	{
	  tilt_angle -= tiltIncr_0;
	  pan_angle += panIncr_0;
	  string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString_0;
	  string command2 = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString_0;
	  system(command1.c_str());
	  ros::Duration(delay).sleep();
	  system(command2.c_str());
	  ros::Duration(delay).sleep();
  	}

  	// Zone 6
  	if (center_y < zone_y_min && center_x > zone_x_max)
  	{
	  tilt_angle -= tiltIncr_0;
	  pan_angle -= panIncr_0;
	  string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString_0;
	  string command2 = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString_0;
	  system(command1.c_str());
          ros::Duration(delay).sleep();
          system(command2.c_str());
	  ros::Duration(delay).sleep();
  	}

  	// Zone 7
  	if (center_y > zone_y_max && center_x < zone_x_min)
  	{
	  tilt_angle += tiltIncr_0;
	  pan_angle += panIncr_0;
	  string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString_0;
	  string command2 = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString_0;
	  system(command1.c_str());
          ros::Duration(delay).sleep();
          system(command2.c_str());
	  ros::Duration(delay).sleep();
  	}

  	//Zone 8
  	if (center_y > zone_y_max && center_x > zone_x_max)
  	{
	  tilt_angle += tiltIncr_0;
	  pan_angle -= panIncr_0;
	  string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString_0;
	  string command2 = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString_0;
	  system(command1.c_str());
          ros::Duration(delay).sleep();
          system(command2.c_str());
	  ros::Duration(delay).sleep();
  	}

	if (tilt_angle > tilt_max) { tilt_angle = tilt_max; }
	if (tilt_angle < tilt_min) { tilt_angle = tilt_min; }
	if (pan_angle > pan_max) { pan_angle = pan_max; }
	if (pan_angle < pan_min) { pan_angle = pan_min; }

	publishCamAngles(pan_angle, tilt_angle);

	cout << "pan: " << pan_angle << endl;
	cout << "tilt: " << tilt_angle << endl;
   	}
  }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "face_tracker");

  ros::param::get("/usb_cam/image_width", image_width);
  ros::param::get("/usb_cam/image_height", image_height);

  while(ros::ok()) {
    faceTracker ft;
    ros::spin();
  }

  return 0;
}
