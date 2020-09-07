#include <esim/esim/simulator_server.hpp>
#include <esim/visualization/ros_publisher.hpp>
#include <esim/visualization/rosbag_writer.hpp>
#include <esim/visualization/adaptive_sampling_benchmark_publisher.hpp>
#include <esim/visualization/synthetic_optic_flow_publisher.hpp>
#include <esim/data_provider/data_provider_from_client.hpp>

#include "ros/ros.h"

#include <params.hpp>

using namespace event_camera_simulator;

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  if (FLAGS_random_seed == 0) FLAGS_random_seed = (unsigned int) time(0);
  srand(FLAGS_random_seed);

  // DataProviderBase::Ptr data_provider_;
  // data_provider_.reset(new DataProviderFromClient("esim_server"));
  DataProviderFromClient data_provider_("esim_server");
  // CHECK(data_provider_);

  EventSimulator::Config event_sim_config;
  event_sim_config.Cp = FLAGS_contrast_threshold_pos;
  event_sim_config.Cm = FLAGS_contrast_threshold_neg;
  event_sim_config.sigma_Cp = FLAGS_contrast_threshold_sigma_pos;
  event_sim_config.sigma_Cm = FLAGS_contrast_threshold_sigma_neg;
  event_sim_config.refractory_period_ns = FLAGS_refractory_period_ns;
  event_sim_config.use_log_image = FLAGS_use_log_image;
  event_sim_config.log_eps = FLAGS_log_eps;
  std::shared_ptr<SimulatorForServer> sim;
  sim.reset(new SimulatorForServer(data_provider_.numCameras(),
                          event_sim_config,
                          FLAGS_exposure_time_ms));
  CHECK(sim);

  data_provider_.registerCallbackServer(
      std::bind(&SimulatorForServer::dataProviderCallback, sim.get(), std::placeholders::_1, std::placeholders::_2));

  VLOG(1) << "Initializing ROS";
  
  // int argc = 0;
  ros::init(argc, argv, std::string("esim_server"));
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService(FLAGS_service_name, &DataProviderFromClient::simulate, &data_provider_);
  
  ROS_INFO("Ready to transfor image to events. service name: %s", service.getService().c_str());

  while (!ros::service::waitForService(FLAGS_service_name, ros::Duration(3.0))) {
       ROS_INFO("Waiting for service event_simulator to become available");
  }
  
  ros::spin();
  return 0;

}