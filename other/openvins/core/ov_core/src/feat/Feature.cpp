/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "Feature.h"

using namespace ov_core;

void Feature::clean_old_measurements(const std::vector<double> &valid_times) {

  // Loop through each of the cameras we have
  for (auto &pair : timestamps) {
    auto cam_id = pair.first;
    auto &ts_vec = pair.second;
    auto &uv_vec = uvs[cam_id];
    auto &uvn_vec = uvs_norm[cam_id];

    // Assert that we have all the parts of a measurement
    assert(ts_vec.size() == uv_vec.size());
    assert(ts_vec.size() == uvn_vec.size());

    // Our iterators
    auto it1 = ts_vec.begin();
    auto it2 = uv_vec.begin();
    auto it3 = uvn_vec.begin();

    // Loop through measurement times, remove ones that are not in our timestamps
    while (it1 != ts_vec.end()) {
      if (std::find(valid_times.begin(), valid_times.end(), *it1) == valid_times.end()) {
        it1 = ts_vec.erase(it1);
        it2 = uv_vec.erase(it2);
        it3 = uvn_vec.erase(it3);
      } else {
        ++it1;
        ++it2;
        ++it3;
      }
    }
  }
}

void Feature::clean_invalid_measurements(const std::vector<double> &invalid_times) {

  // Loop through each of the cameras we have
  for (auto &pair : timestamps) {
    auto cam_id = pair.first;
    auto &ts_vec = pair.second;
    auto &uv_vec = uvs[cam_id];
    auto &uvn_vec = uvs_norm[cam_id];

    // Assert that we have all the parts of a measurement
    assert(ts_vec.size() == uv_vec.size());
    assert(ts_vec.size() == uvn_vec.size());

    // Our iterators
    auto it1 = ts_vec.begin();
    auto it2 = uv_vec.begin();
    auto it3 = uvn_vec.begin();

    // Loop through measurement times, remove ones that are in our timestamps
    while (it1 != ts_vec.end()) {
      if (std::find(invalid_times.begin(), invalid_times.end(), *it1) != invalid_times.end()) {
        it1 = ts_vec.erase(it1);
        it2 = uv_vec.erase(it2);
        it3 = uvn_vec.erase(it3);
      } else {
        ++it1;
        ++it2;
        ++it3;
      }
    }
  }
}

void Feature::clean_older_measurements(double timestamp) {

  // Loop through each of the cameras we have
  for (auto &pair : timestamps) {
    auto cam_id = pair.first;
    auto &ts_vec = pair.second;
    auto &uv_vec = uvs[cam_id];
    auto &uvn_vec = uvs_norm[cam_id];

    // Assert that we have all the parts of a measurement
    assert(ts_vec.size() == uv_vec.size());
    assert(ts_vec.size() == uvn_vec.size());

    // Our iterators
    auto it1 = ts_vec.begin();
    auto it2 = uv_vec.begin();
    auto it3 = uvn_vec.begin();

    // Loop through measurement times, remove ones that are older then the specified one
    while (it1 != ts_vec.end()) {
      if (*it1 <= timestamp) {
        it1 = ts_vec.erase(it1);
        it2 = uv_vec.erase(it2);
        it3 = uvn_vec.erase(it3);
      } else {
        ++it1;
        ++it2;
        ++it3;
      }
    }
  }
}
