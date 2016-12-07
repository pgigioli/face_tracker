#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include <iostream>
#include <string>
#include <std_msgs/Int8.h>

const float PAN_ZERO = 0.0;
const float TILT_ZERO = -5.0;
int WAIT_TO_RECENTER = 0;
int FOUND_OBJECT = 0;

float FRAME_W;
float FRAME_H;

// pan/tilt ranges. Pan range = 189 degress, Tilt range = 102 degrees
// 1 degree horizontally = 47.41 steps, 1 degree vertically = 37.65 steps
const float pan_max = 4480/47.41; const float pan_min = -4480/47.41;
const float tilt_max = 1920/37.65; const float tilt_min = -1920/37.65;

const float panIncr5 = 237/47.41; // 5 degrees
const float tiltIncr5 = 188/37.65; // 5 degrees
const float panIncr10 = 2*237/47.41; // 10 degrees
const float tiltIncr10 = 2*188/37.65; // 10 degrees

const std::string tiltIncrString5 = "188";
const std::string panIncrString5 = "237";
const std::string tiltIncrString10 = "376";
const std::string panIncrString10 = "474";

const std::string FOUND_OBJECT_TOPIC_NAME = "/found_object";
const std::string ROI_COORDINATE_TOPIC_NAME = "/ROI_coordinate";
const std::string FRAME_WIDTH_PARAM_NAME = "/usb_cam/image_width";
const std::string FRAME_HEIGHT_PARAM_NAME = "/usb_cam/image_height";

class faceTracker
{
   ros::NodeHandle _nh;
   ros::Subscriber _coord_sub;
   ros::Publisher _cam_angles;
   ros::Subscriber _found_object_sub;
   float _pan_angle;
   float _tilt_angle;

public:
   faceTracker()
   {
      _pan_angle = PAN_ZERO;
      _tilt_angle = TILT_ZERO;
      this->resetPanTilt();
      _found_object_sub = _nh.subscribe(FOUND_OBJECT_TOPIC_NAME, 1, &faceTracker::foundObjectCb, this);
      _coord_sub = _nh.subscribe(ROI_COORDINATE_TOPIC_NAME, 1, &faceTracker::callback, this);
      _cam_angles = _nh.advertise<std_msgs::Float64MultiArray>("/cam_angles", 1);
   }

   ~faceTracker()
   {
      this->resetPanTilt();
   }

   void resetPanTilt()
   {
      float delay = 1.5;
      std::string command1 = "uvcdynctrl -s \"Tilt Reset\" 1";
      std::string command2 = "uvcdynctrl -s \"Pan Reset\" 1";
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
      anglesArray.data.push_back(_pan_angle);
      anglesArray.data.push_back(_tilt_angle);

      _cam_angles.publish(anglesArray);
   }

   void foundObjectCb(std_msgs::Int8 msg)
   {
      if (msg.data == 1)
      {
         FOUND_OBJECT = 1;
      }
      else if (msg.data == 0)
      {
         FOUND_OBJECT = 0;
         WAIT_TO_RECENTER++;

         if (WAIT_TO_RECENTER > 5)
         {
            recenterCam();
         }
      }
   }

   void recenterCam()
   {
      std::cout << "recentering cam" << std::endl;
      float tilt_buffer_pos = -18.0;
      float tilt_buffer_neg = -23.0;
      float pan_buffer_pos = 3.0;
      float pan_buffer_neg = -3.0;
      float delay = 0.1;

      float tiltIncr_1 = tiltIncr5;
      float panIncr_1 = panIncr5;
      std::string tiltIncrString_1 = tiltIncrString5;
      std::string panIncrString_1 = panIncrString5;

      if (_pan_angle < pan_buffer_neg)
      {
         _pan_angle += panIncr_1;
         std::string command = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString_1;
         system(command.c_str());
         ros::Duration(delay).sleep();
      }
      if (_pan_angle > pan_buffer_pos)
      {
         _pan_angle -= panIncr_1;
         std::string command = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString_1;
	 system(command.c_str());
	 ros::Duration(delay).sleep();
      }
      if (_tilt_angle < tilt_buffer_neg)
      {
         _tilt_angle += tiltIncr_1;
         std::string command = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString_1;
	 system(command.c_str());
         ros::Duration(delay).sleep();
      }
      if (_tilt_angle > tilt_buffer_pos)
      {
         _tilt_angle -= tiltIncr_1;
         std::string command = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString_1;
         system(command.c_str());
         ros::Duration(delay).sleep();
      }

      if (_tilt_angle > tilt_max) { _tilt_angle = tilt_max; }
      if (_tilt_angle < tilt_min) { _tilt_angle = tilt_min; }
      if (_pan_angle > pan_max) { _pan_angle = pan_max; }
      if (_pan_angle < pan_min) { _pan_angle = pan_min; }

      publishCamAngles(_pan_angle, _tilt_angle);

      std::cout << "pan: " << _pan_angle << std::endl;
      std::cout << "tilt: " << _tilt_angle << std::endl;

      return;
   }

   void callback(const geometry_msgs::Point::ConstPtr& msg)
   {
      if (FOUND_OBJECT == 1) {
      WAIT_TO_RECENTER = 0;
      float center_x = msg->x;
      float center_y = msg->y;
      //std::cout << "x: " << center_x << std::endl;
      //std::cout << "y: " << center_y << std::endl;
      //std::cout << " " << std::endl;

      float z_scale = 0.25;
      float zone_x_min = (FRAME_W * 0.5) - (FRAME_W * z_scale * 0.5);
      float zone_x_max = (FRAME_W * 0.5) + (FRAME_W * z_scale * 0.5);
      float zone_y_min = (FRAME_H * 0.5) - (FRAME_H * z_scale * 0.5);
      float zone_y_max = (FRAME_H * 0.5) + (FRAME_H * z_scale * 0.5);

      float delay = 0.1;

      float tiltIncr_0 = tiltIncr5;
      float panIncr_0 = panIncr5;
      std::string tiltIncrString_0 = tiltIncrString5;
      std::string panIncrString_0 = panIncrString5;

      // Zone 1
      if (center_y < zone_y_min && center_x > zone_x_min && center_x < zone_x_max)
      {
         _tilt_angle -= tiltIncr_0;
	 std::string command = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString_0;
         system(command.c_str());
         ros::Duration(delay).sleep();
      }

      // Zone 2
      if (center_y > zone_y_max && center_x > zone_x_min && center_x < zone_x_max)
      {
         _tilt_angle += tiltIncr_0;
         std::string command = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString_0;
         system(command.c_str());
         ros::Duration(delay).sleep();
      }

      // Zone 3
      if (center_x < zone_x_min && center_y > zone_y_min && center_y < zone_y_max)
      {
         _pan_angle += panIncr_0;
         std::string command = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString_0;
         system(command.c_str());
         ros::Duration(delay).sleep();
      }

      // Zone 4
      if (center_x > zone_x_max && center_y > zone_y_min && center_y < zone_y_max)
      {
         _pan_angle -= panIncr_0;
         std::string command = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString_0;
         system(command.c_str());
         ros::Duration(delay).sleep();
      }

      // Zone 5
      if (center_y < zone_y_min && center_x < zone_x_min)
      {
         _tilt_angle -= tiltIncr_0;
         _pan_angle += panIncr_0;
         std::string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString_0;
         std::string command2 = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString_0;
         system(command1.c_str());
         ros::Duration(delay).sleep();
         system(command2.c_str());
         ros::Duration(delay).sleep();
      }

      // Zone 6
      if (center_y < zone_y_min && center_x > zone_x_max)
      {
         _tilt_angle -= tiltIncr_0;
         _pan_angle -= panIncr_0;
         std::string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- -" + tiltIncrString_0;
         std::string command2 = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString_0;
         system(command1.c_str());
         ros::Duration(delay).sleep();
         system(command2.c_str());
         ros::Duration(delay).sleep();
      }

      // Zone 7
      if (center_y > zone_y_max && center_x < zone_x_min)
      {
         _tilt_angle += tiltIncr_0;
         _pan_angle += panIncr_0;
         std::string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString_0;
         std::string command2 = "uvcdynctrl -s \"Pan (relative)\" -- " + panIncrString_0;
         system(command1.c_str());
         ros::Duration(delay).sleep();
         system(command2.c_str());
	  ros::Duration(delay).sleep();
      }

      //Zone 8
      if (center_y > zone_y_max && center_x > zone_x_max)
      {
         _tilt_angle += tiltIncr_0;
         _pan_angle -= panIncr_0;
         std::string command1 = "uvcdynctrl -s \"Tilt (relative)\" -- " + tiltIncrString_0;
         std::string command2 = "uvcdynctrl -s \"Pan (relative)\" -- -" + panIncrString_0;
         system(command1.c_str());
         ros::Duration(delay).sleep();
         system(command2.c_str());
         ros::Duration(delay).sleep();
      }

      if (_tilt_angle > tilt_max) { _tilt_angle = tilt_max; }
      if (_tilt_angle < tilt_min) { _tilt_angle = tilt_min; }
      if (_pan_angle > pan_max) { _pan_angle = pan_max; }
      if (_pan_angle < pan_min) { _pan_angle = pan_min; }

      publishCamAngles(_pan_angle, _tilt_angle);

      std::cout << "pan: " << _pan_angle << std::endl;
      std::cout << "tilt: " << _tilt_angle << std::endl;
   }
   }
};

int main (int argc, char **argv)
{
   ros::init(argc, argv, "face_tracker");

   ros::param::get(FRAME_WIDTH_PARAM_NAME, FRAME_W);
   ros::param::get(FRAME_HEIGHT_PARAM_NAME, FRAME_H);

   while(ros::ok())
   {
      faceTracker ft;
      ros::spin();
   }
  return 0;
}
