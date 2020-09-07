#include <esim/data_provider/data_provider_from_client.hpp>
#include <ze/common/file_utils.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <ze/cameras/camera_impl.hpp>

#include <esim/common/utils.hpp>
#include <esim/visualization/ros_utils.hpp>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/Event.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"

namespace event_camera_simulator {

DataProviderFromClient::DataProviderFromClient(const std::string& node_name)
  : DataProviderBase(DataProviderType::Folder),
    finished_parsing_(false)
{
  // Load CSV image file
//   images_in_str_.open(ze::joinPath(path_to_data_folder, "images.csv"));
//   CHECK(images_in_str_.is_open());

  // Create dummy camera rig
  // these intrinsic values are filled with garbage (width = height = 1) since the actual values are not known
  ze::CameraVector cameras;
  cameras.emplace_back(ze::createEquidistantCameraShared(1, 1, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  ze::TransformationVector extrinsics;
  extrinsics.push_back(ze::Transformation());
  camera_rig_ = std::make_shared<ze::CameraRig>(extrinsics, cameras, "camera");

  // Allocate memory for image data
  sim_data_.images.emplace_back(ImagePtr(new Image(
                                           cv::Size(camera_rig_->at(0).width(),
                                                    camera_rig_->at(0).height()))));

  sim_data_.camera_rig = camera_rig_;
  sim_data_.images_updated = true;
  sim_data_.depthmaps_updated = false;
  sim_data_.optic_flows_updated = false;
  sim_data_.twists_updated = false;
  sim_data_.poses_updated = false;
  sim_data_.imu_updated = false;
  num_cameras_ = camera_rig_->size();

  // this->startServer(node_name);
}

// bool DataProviderFromClient::startServer(std::string node_name)
// {
//   VLOG(1) << "Initializing ROS";
//   int argc = 0;
//   ros::init(argc, nullptr, std::string(node_name));
//   ros::NodeHandle n;

//   ros::ServiceServer service = n.advertiseService("event_simulator", &DataProviderFromClient::simulate, this);
  
//   ROS_INFO("[data_provider_from_client.cpp] Ready to transfor image to events. service name: %s", service.getService().c_str());

//   while (!ros::service::waitForService("event_simulator", ros::Duration(3.0))) {
//        ROS_INFO("Waiting for service event_simulator to become available");
//   }

//   ros::spin();
//   return true;
// }

size_t DataProviderFromClient::numCameras() const
{
  return camera_rig_->size();
}

bool DataProviderFromClient::spinOnce()
{
  return true;
}

bool DataProviderFromClient::ok() const
{
  if (!running_ || finished_parsing_)
  {
    VLOG(1) << "Data Provider was paused/terminated.";
    ROS_INFO("Data Provider was paused/terminated.");
    return false;
  }
  return true;
}

void DataProviderFromClient::shutdown(){
    VLOG(1) << "Data Provider was paused/terminated.";
}

bool DataProviderFromClient::simulate(esim_srvs::EsimSrv::Request  &req, esim_srvs::EsimSrv::Response &res)
{
  ROS_INFO("[data_provider_from_client.cpp] Server CallBack.");
  std_msgs::Header h = req.image.header;
  cv_bridge::CvImagePtr cv_ptr;
  EventsVector events(num_cameras_);

  width = req.image.width;
  height = req.image.height;

  try{
    cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::TYPE_8UC1);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  int64_t stamp = h.stamp.sec;

  cv::Mat image = cv_ptr->image;
//   CHECK(image.data) << "Could not load image from Client";

//   VLOG(3) << "Read image from Client";
  if (image.empty())
  {
    ROS_ERROR("empty image!!!");
    return false;
  }
  else{
    cv::namedWindow("MyWindow", CV_WINDOW_AUTOSIZE);
    cv::imshow("MyWindow", image);
    cv::waitKey(1);

    image.convertTo(*sim_data_.images[0], cv::DataType<ImageFloatType>::type, 1./255.);
  }


  if(callback_server_)
  {
    sim_data_.timestamp = static_cast<Time>(stamp);
    callback_server_(sim_data_, events);
  }

  for(size_t i=0; i<num_cameras_; ++i)
  {
    if(events[i].empty())
    {
      continue;
    }
    this->eventsToSrv(events[i], width, height, res);
  }

  ROS_INFO("[data_provider_from_client.cpp] sending back response");
  return true;
}

void DataProviderFromClient::eventsToSrv(const Events& events, int width, int height, esim_srvs::EsimSrv::Response& msg)
{
//   CHECK(msg);
  ROS_INFO("[data_provider_from_client.cpp] start eventsToSrv...");
  std::vector<dvs_msgs::Event> events_list;
  for(const Event& e : events)
  {
    dvs_msgs::Event ev;
    ev.x = e.x;
    ev.y = e.y;
    ev.ts = toRosTime(e.t);
    ev.polarity = e.pol;

    events_list.push_back(ev);
  }
  msg.events = events_list;
  msg.height = uint32_t(height);
  msg.width = uint32_t(width);
  msg.header.stamp = events_list.back().ts;
}
} // namespace event_camera_simulator