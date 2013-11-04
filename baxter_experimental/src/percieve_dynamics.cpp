/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Perception for a swining pendulum
*/

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace baxter_experiemental
{

static const std::string OPENCV_WINDOW1 = "Filtered Colors";
static const std::string OPENCV_WINDOW2 = "Canny Lines";

class PercieveDynamics
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  cv::Mat hsv_;
  cv::Mat lines_;
  int hue_low_, hue_high_;
  int rho_, theta_, threshold_;
  int min_lin_length_, max_line_gap_;

  // Start time of camera capture
  ros::Time start_time_;

public:

  /**
   * \brief Constructor
   */
  PercieveDynamics()
    : it_(nh_),
      hue_low_(5),
      hue_high_(157),
      rho_(1), 
      theta_(3),  // degrees
      threshold_(36),
      min_lin_length_(50),
      max_line_gap_(10),
      start_time_(0)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cameras/right_hand_camera/image", 1, &PercieveDynamics::imageCb, this);
    //image_pub_ = it_.advertise("/percieved_dynamics/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW1);
    cv::namedWindow(OPENCV_WINDOW2);
    cv::createTrackbar("Hue Low: ", OPENCV_WINDOW1, &hue_low_, 255);
    cv::createTrackbar("Hue High: ", OPENCV_WINDOW1, &hue_high_, 255);

    cv::createTrackbar("Rho: ", OPENCV_WINDOW2, &rho_, 10);
    cv::createTrackbar("Theta: ", OPENCV_WINDOW2, &theta_, 10); // degrees
    cv::createTrackbar("Threshold: ", OPENCV_WINDOW2, &threshold_, 200);
    cv::createTrackbar("Min Line Len:: ", OPENCV_WINDOW2, &min_lin_length_, 200); 
    cv::createTrackbar("Max Line Gap: ", OPENCV_WINDOW2, &max_line_gap_, 200);
  }

  /**
   * \brief Destructor
   */
  ~PercieveDynamics()
  {
    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
  }


  /**
   * \brief Callback from input image subscriber
   */
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Record start time if this is the first
    if(start_time_.toSec() == 0)
    {
      // this is the first callback
      start_time_ = msg->header.stamp;
    }
    ros::Duration elapsed = msg->header.stamp - start_time_;

    // Smooth Image
    cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(9,9), 0, 0); // smooth orig image with Gaussian kernel

    // Convert to HSV format
    cv::cvtColor(cv_ptr->image, hsv_, CV_BGR2HSV);

    // Use only the Hue value
    cv::inRange(hsv_, cv::Scalar(hue_low_,0,0), cv::Scalar(hue_high_,255,255), hsv_);

    // Detect the edges of image
    cv::Canny(hsv_, lines_, 50, 200, 3);

    // Check for zeros
    if(!rho_) rho_ = 1;
    if(!theta_) theta_ = 1;
    if(!threshold_) threshold_ = 1;
    if(!min_lin_length_) min_lin_length_ = 1;
    if(!max_line_gap_) max_line_gap_ = 1;

    // Extract lines
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(lines_, lines, rho_, theta_*CV_PI/180, threshold_, 0, 0); //min_lin_length_, max_line_gap_ );
    cv::Mat cdst;
    cv::cvtColor(lines_, cdst, CV_GRAY2BGR);

    // Average the measured angles
    double total_angles = 0;
    
    // Display the angles
    for( size_t i = 0; i < lines.size(); i++ )
    {
      float rho = lines[i][0], theta = lines[i][1];
      total_angles += theta;
      cv::Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      cv::line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
    }
    
    std::cout << elapsed.toSec() << ", " << total_angles / lines.size() << std::endl;

    // Draw an example circle on the video stream
    //if (hsv_.rows > 60 && hsv_.cols > 60)
    //  cv::circle(hsv_, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW1, hsv_);
    cv::imshow(OPENCV_WINDOW2, cdst);
    cv::waitKey(3);
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }

  int cvRound(double x)
  {
    int y;
    if(x >= (int)x+0,5)
      y = (int)x++;
    else
      y = (int)x;
    return y;
  }

  /*
  void histAndBackproj(int, void*)
  {
    cv::MatND hist;
    int histSize = MAX( bins_, 2 );
    float hue_range[] = { 0, 180 };
    const float* ranges = { hue_range };

    /// Get the Histogram and normalize it
    cv::calcHist( &hue_, 1, 0, cv::Mat(), hist, 1, &histSize, &ranges, true, false );
    cv::normalize( hist, hist, 0, 255, cv::NORM_MINMAX, -1, cv::Mat() );

    /// Get Backprojection
    cv::MatND backproj;
    cv::calcBackProject( &hue_, 1, 0, hist, backproj, &ranges, 1, true );

    /// Draw the backproj
    imshow( "BackProj", backproj );
  }
  */

}; // end class

} // end namespace

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("percieve_dynamics","OpenCV Perception for Moving Object Dynamics");

  ros::init(argc, argv, "percieve_dynamics");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  baxter_experiemental::PercieveDynamics tester;
  ros::spin();

  ROS_INFO_STREAM_NAMED("percieve_dynamics","Shutting down.");

  return 0;
}

