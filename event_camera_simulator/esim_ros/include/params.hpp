#pragma once

#include <glog/logging.h>
#include <gflags/gflags.h>


DEFINE_double(contrast_threshold_pos, 1.0,
              "Contrast threshold (positive)");

DEFINE_double(contrast_threshold_neg, 1.0,
              "Contrast threshold  (negative))");

DEFINE_double(contrast_threshold_sigma_pos, 0.021,
              "Standard deviation of contrast threshold (positive)");

DEFINE_double(contrast_threshold_sigma_neg, 0.021,
              "Standard deviation of contrast threshold  (negative))");

DEFINE_int64(refractory_period_ns, 0,
             "Refractory period (time during which a pixel cannot fire events just after it fired one), in nanoseconds");

DEFINE_double(exposure_time_ms, 10.0,
              "Exposure time in milliseconds, used to simulate motion blur");

DEFINE_bool(use_log_image, true,
            "Whether to convert images to log images in the preprocessing step.");

DEFINE_double(log_eps, 0.001,
              "Epsilon value used to convert images to log: L = log(eps + I / 255.0).");

DEFINE_int32(random_seed, 0,
              "Random seed used to generate the trajectories. If set to 0 the current time(0) is taken as seed.");

DEFINE_string(service_name, "",
       "Path to folder containing the data.");