#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ze/common/macros.hpp>
#include <ze/common/types.hpp>
#include <esim/data_provider/data_provider_base.hpp>
#include <esim/common/types.hpp>
#include <ze/cameras/camera_rig.hpp>
#include <fstream>

#include <esim_srvs/EsimSrv.h>

namespace event_camera_simulator {

class DataProviderFromClient : public DataProviderBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DataProviderFromClient(const std::string& node_name);

  virtual ~DataProviderFromClient() = default;

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  virtual void shutdown() override;

  size_t numCameras() const override;

  bool simulate(esim_srvs::EsimSrv::Request  &req, esim_srvs::EsimSrv::Response &res);

  bool startServer(std::string node_name);

private:

  int64_t getTimeStamp(const std::string& ts_str) const;

//   std::ifstream images_in_str_;
  const char delimiter_{','};
  const size_t num_tokens_in_line_ = 2; // stamp, image
  bool finished_parsing_;

  size_t num_cameras_;

  SimulatorData sim_data_;

  int width, height;

  void eventsToSrv(const Events& events, int width, int height, esim_srvs::EsimSrv::Response& msg);
};

} // namespace event_camera_simulator