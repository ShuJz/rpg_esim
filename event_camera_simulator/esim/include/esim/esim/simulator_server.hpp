#pragma once

#include <esim/esim/event_simulator.hpp>
#include <esim/esim/camera_simulator.hpp>
#include <esim/common/types.hpp>
// #include <esim/visualization/publisher_interface.hpp>

namespace event_camera_simulator {

/* The Simulator forwards the simulated images / depth maps
 * from the data provider to multiple, specialized camera simulators, such as:
 * (i) event camera simulators that simulate events based on sequences of images
 * (ii) camera simulators that simulate real cameras
 *      (including motion blur, camera response function, noise, etc.)
 *
 * The Simulator then forwards the simulated data to one or more publishers.
*/
class SimulatorForServer
{
public:
  SimulatorForServer(size_t num_cameras,
            const EventSimulator::Config& event_sim_config,
            double exposure_time_ms)
    : num_cameras_(num_cameras),
      exposure_time_(ze::millisecToNanosec(exposure_time_ms))
  {
    for(size_t i=0; i<num_cameras_; ++i)
    {
      event_simulators_.push_back(EventSimulator(event_sim_config));
      camera_simulators_.push_back(CameraSimulator(exposure_time_ms));
    }
  }

  ~SimulatorForServer();
  void dataProviderCallback(const SimulatorData& sim_data, EventsVector& events);

private:
  size_t num_cameras_;
  std::vector<EventSimulator> event_simulators_;

  std::vector<CameraSimulator> camera_simulators_;
  Duration exposure_time_;

  ImagePtrVector corrupted_camera_images_;
};

} // namespace event_camera_simulator
