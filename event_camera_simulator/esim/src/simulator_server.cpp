#include <esim/esim/simulator_server.hpp>
#include <ze/common/timer_collection.hpp>
#include <esim/common/utils.hpp>

#include "ros/ros.h"
namespace event_camera_simulator {

DECLARE_TIMER(TimerEventSimulator, timers_event_simulator_server_, simulate_events, visualization);

SimulatorForServer::~SimulatorForServer()
{
  timers_event_simulator_server_.saveToFile("/tmp", "event_simulator.csv");
}

void SimulatorForServer::dataProviderCallback(const SimulatorData &sim_data, EventsVector &events)
{
  CHECK_EQ(event_simulators_.size(), num_cameras_);

  bool camera_simulator_success;

  if(sim_data.images_updated)
  {
    // EventsVector events(num_cameras_);
    Time time = sim_data.timestamp;
    // simulate the events and camera images for every sensor in the rig
    {
      auto t = timers_event_simulator_server_[TimerEventSimulator::simulate_events].timeScope();
      for(size_t i=0; i<num_cameras_; ++i)
      {
        events[i] = event_simulators_[i].imageCallback(*sim_data.images[i], time);

        if(corrupted_camera_images_.size() < num_cameras_)
        {
          // allocate memory for the corrupted camera images and set them to 0
          corrupted_camera_images_.emplace_back(std::make_shared<Image>(sim_data.images[i]->size()));
          corrupted_camera_images_[i]->setTo(0.);
        }

        camera_simulator_success = camera_simulators_[i].imageCallback(*sim_data.images[i], time, corrupted_camera_images_[i]);
      }
    }

    // publish the simulation data + events
    {
      auto t = timers_event_simulator_server_[TimerEventSimulator::visualization].timeScope();
    //   publishData(sim_data, events, camera_simulator_success, corrupted_camera_images_);
    }
  }
  else
  {
    {
      // just forward the simulation data to the publisher
      auto t = timers_event_simulator_server_[TimerEventSimulator::visualization].timeScope();
    //   publishData(sim_data, {}, camera_simulator_success, corrupted_camera_images_);
    }
  }
}
} // namespace event_camera_simulator