/**
 * @file main.cpp
 * @brief Camera data process. Reads camera images and sends them via Shared
 * Memory.
 */

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <map>
#include <semaphore.h>
#include <string>
#include <thread>

#include "DataTypes.h"
#include "EuRoCDataLoader.h"
#include "IpcManager.h"

using namespace std;
using namespace ov_core;

int main(int argc, char **argv) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <path_to_dataset>" << endl;
    return EXIT_FAILURE;
  }
  string dataset_path = argv[1];
  size_t max_frames = 0;
  if (const char *env_max_frames = std::getenv("OPENVINS_MAX_FRAMES")) {
    max_frames = static_cast<size_t>(std::stoul(env_max_frames));
  }
  bool offline_benchmark = true;
  if (const char *env_realtime = std::getenv("OPENVINS_REALTIME")) {
    offline_benchmark = std::string(env_realtime) != "1";
  }
  double speedup = 20.0;
  if (const char *env_speedup = std::getenv("OPENVINS_SPEEDUP")) {
    speedup = std::max(1.0, std::stod(env_speedup));
  }

  bool use_stereo = false;
  if (const char *env_stereo = std::getenv("OPENVINS_STEREO")) {
    use_stereo = (std::string(env_stereo) == "1");
  }

  IpcManager ipc;
  sem_t *ready_sem = ipc.open_ready_sem(false);

  // Wait for VINS process to be ready
  cout << "[Camera Process] Waiting for VINS process readiness..." << endl;
  sem_wait(ready_sem);
  sem_post(ready_sem); // Relay the signal to next process

  ShmRingBuffer *ring_buffer =
      (ShmRingBuffer *)ipc.open_cam_shm(false, sizeof(ShmRingBuffer));
  sem_t *sem = ipc.open_cam_sem(false);

  EuRoCDataLoader loader(dataset_path);
  map<double, string> cam0_filenames, cam1_filenames;
  if (!loader.load_cam_filenames(0, cam0_filenames)) {
    cerr << "[Camera Process] Failed to load cam0 data from " << dataset_path
         << endl;
    return EXIT_FAILURE;
  }
  if (use_stereo) {
    if (!loader.load_cam_filenames(1, cam1_filenames)) {
      cerr << "[Camera Process] Failed to load cam1 data from " << dataset_path
           << endl;
      return EXIT_FAILURE;
    }
    cout << "[Camera Process] Loaded " << cam0_filenames.size()
         << " stereo camera image pairs." << endl;
  } else {
    cout << "[Camera Process] Loaded " << cam0_filenames.size()
         << " monocular camera images." << endl;
  }

  // Ensure IMU process starts first. Short delay in accelerated offline mode.
  this_thread::sleep_for(chrono::duration<double>(0.5 / (offline_benchmark ? speedup : 1.0)));

  auto start_wall_time = chrono::steady_clock::now();
  double start_data_time = -1.0;

  size_t frames_sent = 0;
  for (const auto &entry : cam0_filenames) {
    if (max_frames > 0 && frames_sent >= max_frames) {
      break;
    }
    double timestamp = entry.first;
    string filename0 = entry.second;

    string filename1;
    if (use_stereo) {
      if (cam1_filenames.find(timestamp) == cam1_filenames.end())
        continue;
      filename1 = cam1_filenames[timestamp];
    }

    if (start_data_time < 0) {
      start_data_time = timestamp;
      start_wall_time = chrono::steady_clock::now();
    }

    {
      double elapsed_data_time = (timestamp - start_data_time) / (offline_benchmark ? speedup : 1.0);
      auto now = chrono::steady_clock::now();
      double elapsed_wall_time =
          chrono::duration<double>(now - start_wall_time).count();

      if (elapsed_data_time > elapsed_wall_time) {
        this_thread::sleep_for(
            chrono::duration<double>(elapsed_data_time - elapsed_wall_time));
      }
    }

    cv::Mat image0 = loader.get_image(0, filename0);
    if (image0.empty())
      continue;

    cv::Mat image1;
    if (use_stereo) {
      image1 = loader.get_image(1, filename1);
      if (image1.empty())
        continue;
    }

    // Offline benchmark: wait for consumer instead of dropping frames.
    while (((ring_buffer->write_idx + 1) % ShmRingBuffer::BUFFER_SIZE) ==
        ring_buffer->read_idx) {
      if (offline_benchmark) {
        this_thread::sleep_for(chrono::microseconds(500));
      } else {
        ring_buffer->read_idx =
            (ring_buffer->read_idx + 1) % ShmRingBuffer::BUFFER_SIZE;
        break;
      }
    }
    CameraDataPacket &packet0 = ring_buffer->packets[ring_buffer->write_idx];
    packet0.timestamp = timestamp;
    packet0.cam_id = 0;
    packet0.width = image0.cols;
    packet0.height = image0.rows;
    memcpy(packet0.data, image0.data, image0.cols * image0.rows);
    ring_buffer->write_idx =
        (ring_buffer->write_idx + 1) % ShmRingBuffer::BUFFER_SIZE;
    sem_post(sem);

    if (use_stereo) {
      // Offline benchmark: wait for consumer instead of dropping frames.
      while (((ring_buffer->write_idx + 1) % ShmRingBuffer::BUFFER_SIZE) ==
          ring_buffer->read_idx) {
        if (offline_benchmark) {
          this_thread::sleep_for(chrono::microseconds(500));
        } else {
          ring_buffer->read_idx =
              (ring_buffer->read_idx + 1) % ShmRingBuffer::BUFFER_SIZE;
          break;
        }
      }
      CameraDataPacket &packet1 = ring_buffer->packets[ring_buffer->write_idx];
      packet1.timestamp = timestamp;
      packet1.cam_id = 1;
      packet1.width = image1.cols;
      packet1.height = image1.rows;
      memcpy(packet1.data, image1.data, image1.cols * image1.rows);
      ring_buffer->write_idx =
          (ring_buffer->write_idx + 1) % ShmRingBuffer::BUFFER_SIZE;
      sem_post(sem);
    }
    frames_sent++;
  }

  cout << "[Camera Process] Finished sending all camera data." << endl;
  sem_close(sem);
  sem_close(ready_sem);

  return EXIT_SUCCESS;
}
