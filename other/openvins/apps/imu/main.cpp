/**
 * @file main.cpp
 * @brief IMU data process. Reads IMU data from CSV and sends it via Unix domain socket.
 */

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "EuRoCDataLoader.h"
#include "DataTypes.h"
#include "IpcManager.h"

using namespace std;
using namespace ov_core;

int main(int argc, char **argv) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <path_to_dataset>" << endl;
    return EXIT_FAILURE;
  }
  string dataset_path = argv[1];

  IpcManager ipc;
  sem_t *ready_sem = ipc.open_ready_sem(false);

  // Wait for VINS process to be ready
  cout << "[IMU Process] Waiting for VINS process readiness..." << endl;
  sem_wait(ready_sem);
  sem_post(ready_sem); // Relay the signal to next process

  int imu_sock = ipc.open_imu_mq(false);

  EuRoCDataLoader loader(dataset_path);
  vector<ImuData> imu_data;
  if (!loader.load_imu_data(imu_data)) {
    cerr << "[IMU Process] Failed to load IMU data from " << dataset_path << endl;
    return EXIT_FAILURE;
  }
  cout << "[IMU Process] Loaded " << imu_data.size() << " IMU measurements." << endl;

  // Wait for a short duration to ensure main process is ready
  this_thread::sleep_for(chrono::milliseconds(200));

  auto start_wall_time = chrono::steady_clock::now();
  double start_data_time = -1.0;

  bool offline_benchmark = true;
  if (const char *env_realtime = std::getenv("OPENVINS_REALTIME")) {
    offline_benchmark = std::string(env_realtime) != "1";
  }
  // ponytail: MAX_FRAMES is camera frames at 20Hz; cap IMU replay time so a
  // paced smoke doesn't run the whole sequence. Raise to a full-sequence
  // limit env if longer replays are ever needed.
  double replay_seconds_limit = -1.0;
  if (const char *env_max_frames = std::getenv("OPENVINS_MAX_FRAMES")) {
    replay_seconds_limit = std::stod(env_max_frames) / 20.0;
  }
  double speedup = 20.0;
  if (const char *env_speedup = std::getenv("OPENVINS_SPEEDUP")) {
    speedup = std::max(1.0, std::stod(env_speedup));
  }

  // Offline benchmark mode: accelerated playback unless OPENVINS_REALTIME=1.
  for (const auto &data : imu_data) {
    if (start_data_time < 0) {
      start_data_time = data.timestamp;
      start_wall_time = chrono::steady_clock::now();
    }

    {
      double elapsed_data_time = (data.timestamp - start_data_time) / (offline_benchmark ? speedup : 1.0);
      if (replay_seconds_limit > 0 && (data.timestamp - start_data_time) / 1.0 > replay_seconds_limit) {
        break; // paced smoke cap: data-time seconds, independent of speedup
      }
      auto now = chrono::steady_clock::now();
      double elapsed_wall_time = chrono::duration<double>(now - start_wall_time).count();

      if (elapsed_data_time > elapsed_wall_time) {
        this_thread::sleep_for(chrono::duration<double>(elapsed_data_time - elapsed_wall_time));
      }
    }

    ImuDataPacket packet;
    packet.timestamp = data.timestamp;
    packet.wm[0] = data.wm[0];
    packet.wm[1] = data.wm[1];
    packet.wm[2] = data.wm[2];
    packet.am[0] = data.am[0];
    packet.am[1] = data.am[1];
    packet.am[2] = data.am[2];

    try {
      ipc.send_imu(imu_sock, packet);
    } catch (const std::exception &e) {
      cerr << "[IMU Process] Failed to send IMU packet: " << e.what() << endl;
      break;
    }
  }

  cout << "[IMU Process] Finished sending all IMU data." << endl;
  close(imu_sock);
  sem_close(ready_sem);

  return EXIT_SUCCESS;
}
