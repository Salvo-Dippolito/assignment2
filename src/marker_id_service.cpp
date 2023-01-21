#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <std_srvs/Trigger.h>



#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


class ArucoMarkerPublisherServer{
private:
  // ArUco stuff
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  aruco::CameraParameters camParam_;

  // node params
  double marker_size_;
  bool useCamInfo_;
  int marker_id;
  cv::Mat inImage_;

  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::ServiceServer service;
  
  
  
  
public:
  ArucoMarkerPublisherServer() :
      nh_("~"), it_(nh_), useCamInfo_(true)
  
  {
    image_sub_ = it_.subscribe("/image", 1, &ArucoMarkerPublisherServer::image_callback, this);
    service = nh_.advertiseService("/get_marker_id", &ArucoMarkerPublisherServer::get_id, this);
   
   
    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();
  }


  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    inImage_ = cv_ptr->image;

  }

  bool get_id(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

    res.success = true;
    aruco::MarkerDetector mDetector_;
    std::vector<aruco::Marker> markers_;
    aruco::CameraParameters camParam_;

    try{
        
        marker_id=0;
        
        printf("trying now");

        // clear out previous detection results
        markers_.clear();
        marker_id=0;
        // ok, let's detect
        mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

        for (std::size_t i = 0; i < markers_.size(); ++i){
          marker_id=markers_.at(i).id;
        }
        
        if(marker_id==0){

          res.success=false;
          std::cout << ".";
        
        }else{       

          std::cout << "The id of the detected marker detected is: ";

          for (std::size_t i = 0; i < markers_.size(); ++i){
            std::cout << markers_.at(i).id << " ";
        
          }
        }

        // draw detected markers on the image for visualization
        for (std::size_t i = 0; i < markers_.size(); ++i)
        {
          markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
        }

      }catch (cv_bridge::Exception& e){

        ROS_ERROR("cv_bridge exception: %s", e.what());
      }

  res.message = std::to_string(marker_id);
  

  return true;
}


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_server");
  ros::NodeHandle nh_;
  
  

  ArucoMarkerPublisherServer node;




  ros::spin();
}