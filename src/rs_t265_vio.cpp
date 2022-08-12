/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko, Michael Loipführer and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <thread>

#include <sophus/se3.hpp>

#include <tbb/concurrent_queue.h>

#include <basalt/device/rs_t265.h>
#include <basalt/io/dataset_io.h>
#include <basalt/vi_estimator/vio_estimator.h>

#include <basalt/serialization/headers_serialization.h>

void load_data(const std::string& calib_path);

basalt::RsT265Device::Ptr t265_device;

basalt::VioVisualizationData::Ptr curr_vis_data;

tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> out_vis_queue;

basalt::Calibration<double> calib;
basalt::VioConfig vio_config;
basalt::OpticalFlowBase::Ptr opt_flow_ptr;
basalt::VioEstimatorBase::Ptr vio;

namespace basalt {
	struct Pose {
    std::int64_t timestamp;   //!< In same clock as input samples
		float px, py, pz;         //!< Position vector
		float rx, ry, rz, rw = 1; //!< Orientation quaternion
	};
}

basalt::Pose get_pose_from_state(Sophus::SE3d&, int64_t);

int main() {
  std::string cam_calib_path;//{ "C:/Users/Josep/Documents/Github/BASALT_V2/build/Release/calibration.json" };
	std::string config_path;//{ "C:/Users/Josep/Documents/Github/BASALT_V2/build/Release/euroc_config.json" };
  bool use_imu = false;
  bool use_double = false;

  if (!config_path.empty()) {
    vio_config.load(config_path);
  } else {
    vio_config.optical_flow_skip_frames = 2;
  }

  // realsense
  t265_device.reset(new basalt::RsT265Device(false, 1, 90, 10.0));  // TODO: add options?

  // startup device and load calibration
  t265_device->start();

  if (cam_calib_path.empty()) {
    calib = *t265_device->exportCalibration();
  } else {
    load_data(cam_calib_path);
  }

  opt_flow_ptr = basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
  t265_device->image_data_queue = &opt_flow_ptr->input_queue;

  vio = basalt::VioEstimatorFactory::getVioEstimator(vio_config, calib, basalt::constants::g, use_imu, use_double);
  vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  t265_device->imu_data_queue = &vio->imu_data_queue;

  opt_flow_ptr->output_queue = &vio->vision_data_queue;
  vio->out_vis_queue = &out_vis_queue;

  while (true) {
    // prepare pose struct
    basalt::Pose pose_basalt{};
    // get latest vio data
		out_vis_queue.pop(curr_vis_data);

    if (curr_vis_data.get()) {
      if (use_imu) {
        pose_basalt = get_pose_from_state(curr_vis_data->states.back(), curr_vis_data->t_ns);
        std::cout << "x: " << pose_basalt.px << " y: " << pose_basalt.py << " z: " << pose_basalt.pz << std::endl;
      } else {
        pose_basalt = get_pose_from_state(curr_vis_data->frames.back(), curr_vis_data->t_ns);
        std::cout << "x: " << pose_basalt.px << " y: " << pose_basalt.py << " z: " << pose_basalt.pz << std::endl;
      }
    }
  }

  t265_device->stop();

  return 0;
}

basalt::Pose get_pose_from_state(Sophus::SE3d& T_w_i, int64_t t_ns) {
	basalt::Pose pose_basalt{};

	pose_basalt.px = T_w_i.translation().x();
	pose_basalt.py = T_w_i.translation().y();
	pose_basalt.pz = T_w_i.translation().z();
	pose_basalt.rx = T_w_i.unit_quaternion().x();
	pose_basalt.ry = T_w_i.unit_quaternion().y();
	pose_basalt.rz = T_w_i.unit_quaternion().z();
	pose_basalt.rw = T_w_i.unit_quaternion().w();

	pose_basalt.timestamp = t_ns;

	return pose_basalt;
}

void load_data(const std::string& calib_path) {
  std::ifstream os(calib_path, std::ios::binary);

  if (os.is_open()) {
    cereal::JSONInputArchive archive(os);
    archive(calib);
    std::cout << "Loaded camera with " << calib.intrinsics.size() << " cameras"
              << std::endl;

  } else {
    std::cerr << "could not load camera calibration " << calib_path
              << std::endl;
    std::abort();
  }
}
