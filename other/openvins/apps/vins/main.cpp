#include <atomic>
#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <semaphore.h>
#include <thread>
#include <unistd.h>

#include "DataTypes.h"
#include "IpcManager.h"
#include "ThreadSafeQueue.h"
#include "PerformanceLogger.h"
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"

using namespace std;
using namespace ov_core;
using namespace ov_msckf;

// Global flag to stop threads
atomic<bool> stop_flag(false);
constexpr size_t MAX_CAMERA_QUEUE_PACKETS = 4;

/**
 * @brief Signal handler to ensure graceful shutdown.
 */
void signalHandler(int signum) {
  cout << "\n[VINS Process] Interrupt signal (" << signum << ") received.\n";
  stop_flag = true;
}

/**
 * @brief Thread for reading IMU data from Unix domain socket.
 */
void imu_read_thread(int sock_fd, IpcManager *ipc, ThreadSafeQueue<ImuDataPacket> *queue) {
  while (!stop_flag) {
    ImuDataPacket packet;
    if (ipc->recv_imu(sock_fd, packet, 1000)) {
      queue->push(packet);
    }
  }
  cout << "[VINS Process] IMU read thread stopped." << endl;
}

/**
 * @brief Thread for reading Camera data from Shared Memory.
 */
void camera_read_thread(sem_t *sem, ShmRingBuffer *ring_buffer, ThreadSafeQueue<CameraDataPacket> *queue) {
  int count = 0;
  while (!stop_flag) {
    // Bound memory use and let offline playback back-pressure the camera feeder.
    while (!stop_flag && queue->size() >= MAX_CAMERA_QUEUE_PACKETS) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    if (stop_flag)
      break;

    // Poll ring buffer directly — semaphore signalling is unreliable across
    // processes on macOS, but the ring buffer indices are in shared memory
    // and are visible immediately after camera_process writes them.
    if (ring_buffer->read_idx != ring_buffer->write_idx) {
      CameraDataPacket packet = ring_buffer->packets[ring_buffer->read_idx];
      queue->push(packet);
      ring_buffer->read_idx = (ring_buffer->read_idx + 1) % ShmRingBuffer::BUFFER_SIZE;
      if (++count % 20 == 0)
        cout << "[CAM thread] received " << count << " packets" << endl;
    } else {
      // Nothing to read — sleep briefly to avoid busy-wait
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }
  cout << "[VINS Process] Camera read thread stopped." << endl;
}

int main(int argc, char **argv) {
  // Suppress verbose cout output for fair benchmark timing
  std::cout.setstate(std::ios::failbit);
  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " <path_to_config.yaml> <log_dir>" << endl;
    return EXIT_FAILURE;
  }
  string config_path = argv[1];
  string log_dir = argv[2];

  signal(SIGINT, signalHandler);

  // Initialize OpenVINS VioManager
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  VioManagerOptions params;
  params.print_and_load(parser);
  auto sys = std::make_shared<VioManager>(params);
  bool is_stereo = (params.state_options.num_cameras > 1 && params.use_stereo);
  if (const char *env_stereo = std::getenv("OPENVINS_STEREO")) {
    is_stereo = (std::string(env_stereo) == "1");
  }
  cout << "[VINS Process] VioManager initialized (" << (is_stereo ? "Stereo" : "Monocular") << ")." << endl;

  // Setup IPC
  IpcManager ipc;
  int imu_sock = ipc.open_imu_mq(true);
  ShmRingBuffer *ring_buffer = (ShmRingBuffer *)ipc.open_cam_shm(true, sizeof(ShmRingBuffer));
  ring_buffer->write_idx = 0;
  ring_buffer->read_idx = 0;
  sem_t *sem = ipc.open_cam_sem(true);
  sem_t *ready_sem = ipc.open_ready_sem(true);

  // Signal readiness via semaphore
  sem_post(ready_sem);

  // Queues for data buffering
  ThreadSafeQueue<ImuDataPacket> imu_queue;
  ThreadSafeQueue<CameraDataPacket> camera_queue;

  // Start read threads
  thread imu_thread(imu_read_thread, imu_sock, &ipc, &imu_queue);
  thread cam_thread(camera_read_thread, sem, ring_buffer, &camera_queue);

  // Trajectory logging
  ofstream traj_file(log_dir + "/trajectory.txt");
  ofstream processed_file(log_dir + "/processed_frames.csv");
  processed_file << "frame,stamp,initialized,frame_interval_us,pipeline_latency_us,"
                    "arrival_s,start_s,finish_s\n";
  size_t processed_frames = 0;
  PerformanceLogger perf_logger;
  auto previous_frame_start = chrono::steady_clock::time_point{};
  auto realtime_epoch = chrono::steady_clock::time_point{};
  double realtime_first_timestamp = -1.0;

  cout << "[VINS Process] System ready. Waiting for data..." << endl;

  // Buffer some IMU data before starting
  this_thread::sleep_for(chrono::seconds(1));

  // Main processing loop
  double last_imu_time = -1.0;
  while (!stop_flag) {
    // 1. Wait for camera packet(s)
    if (is_stereo) {
      if (camera_queue.size() < 2) {
        this_thread::sleep_for(chrono::milliseconds(1));
        continue;
      }
    } else {
      if (camera_queue.empty()) {
        this_thread::sleep_for(chrono::milliseconds(1));
        continue;
      }
    }

    // 2. Get camera frame(s)
    CameraData message;
    double cam_time = -1.0;

    if (is_stereo) {
      auto p1_opt = camera_queue.try_pop();
      auto p2_opt = camera_queue.try_pop();
      if (!p1_opt || !p2_opt)
        continue;

      auto &p1 = *p1_opt;
      auto &p2 = *p2_opt;

      if (abs(p1.timestamp - p2.timestamp) > 1e-6) {
        // Not a pair, skip p1
        continue;
      }
      cam_time = p1.timestamp;
      message.timestamp = cam_time;

      static cv::Mat default_mask_cam0, default_mask_cam1;
      if (default_mask_cam0.empty() || default_mask_cam0.rows != p1.height || default_mask_cam0.cols != p1.width) {
        default_mask_cam0 = cv::Mat::zeros(p1.height, p1.width, CV_8UC1);
      }
      if (default_mask_cam1.empty() || default_mask_cam1.rows != p2.height || default_mask_cam1.cols != p2.width) {
        default_mask_cam1 = cv::Mat::zeros(p2.height, p2.width, CV_8UC1);
      }

      message.sensor_ids.push_back(p1.cam_id);
      message.images.push_back(cv::Mat(p1.height, p1.width, CV_8UC1, p1.data).clone());
      message.masks.push_back(default_mask_cam0);

      message.sensor_ids.push_back(p2.cam_id);
      message.images.push_back(cv::Mat(p2.height, p2.width, CV_8UC1, p2.data).clone());
      message.masks.push_back(default_mask_cam1);
    } else {
      auto p1_opt = camera_queue.try_pop();
      if (!p1_opt)
        continue;

      auto &p1 = *p1_opt;
      cam_time = p1.timestamp;
      message.timestamp = cam_time;

      static cv::Mat default_mask_cam0;
      if (default_mask_cam0.empty() || default_mask_cam0.rows != p1.height || default_mask_cam0.cols != p1.width) {
        default_mask_cam0 = cv::Mat::zeros(p1.height, p1.width, CV_8UC1);
      }

      message.sensor_ids.push_back(p1.cam_id);
      message.images.push_back(cv::Mat(p1.height, p1.width, CV_8UC1, p1.data).clone());
      message.masks.push_back(default_mask_cam0);
    }

    // The propagator integrates to camera time plus the estimated camera-IMU offset.
    double imu_target_time = cam_time;
    const auto state = sys->get_state();
    if (state->_options.do_calib_camera_timeoffset) {
      imu_target_time += std::max(0.0, state->_calib_dt_CAMtoIMU->value()(0));
    }

    // 3. Feed enough IMU data to cover the time-offset-adjusted camera timestamp.
    bool imu_caught_up = (last_imu_time != -1.0 && last_imu_time >= imu_target_time);
    while (!stop_flag && !imu_caught_up) {
      if (imu_queue.empty()) {
        this_thread::sleep_for(chrono::milliseconds(1));
      }
      while (!imu_queue.empty()) {
        auto imu_packet = imu_queue.pop();
        ImuData data;
        data.timestamp = imu_packet.timestamp;
        data.wm << imu_packet.wm[0], imu_packet.wm[1], imu_packet.wm[2];
        data.am << imu_packet.am[0], imu_packet.am[1], imu_packet.am[2];
        sys->feed_measurement_imu(data);
        last_imu_time = data.timestamp;
        if (last_imu_time >= imu_target_time) {
          imu_caught_up = true;
          break;
        }
      }
    }

    // 4. Feed camera measurement
    const auto frame_start = chrono::steady_clock::now();
    if (realtime_epoch == chrono::steady_clock::time_point{}) {
      realtime_epoch = frame_start;
      realtime_first_timestamp = message.timestamp;
    }
    int64_t frame_interval_us = 0;
    if (previous_frame_start != chrono::steady_clock::time_point{}) {
      frame_interval_us = chrono::duration_cast<chrono::microseconds>(
          frame_start - previous_frame_start).count();
    }
    perf_logger.start_frame();
    sys->feed_measurement_camera(message);
    perf_logger.end_frame();
    const auto frame_end = chrono::steady_clock::now();
    const int64_t pipeline_latency_us = chrono::duration_cast<chrono::microseconds>(
        frame_end - frame_start).count();
    previous_frame_start = frame_start;
    processed_file << ++processed_frames << "," << fixed << setprecision(15)
                   << message.timestamp << "," << sys->initialized() << ","
                   << frame_interval_us << "," << pipeline_latency_us << ","
                   << message.timestamp - realtime_first_timestamp << ","
                   << chrono::duration<double>(frame_start - realtime_epoch).count() << ","
                   << chrono::duration<double>(frame_end - realtime_epoch).count() << "\n";
    processed_file.flush();

    // 5. Log status and trajectory
    if (sys->initialized()) {
      static bool first_init = true;
      if (first_init) {
        cout << "[VINS Process] System Initialized!" << endl;
        first_init = false;
      }
      std::shared_ptr<ov_msckf::State> state = sys->get_state();
      Eigen::Vector3d pos = state->_imu->pos();
      Eigen::Vector4d quat = state->_imu->quat();
      traj_file << fixed << setprecision(15)
                << message.timestamp << " "
                << pos.x() << " " << pos.y() << " " << pos.z() << " "
                << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << "\n";
      traj_file.flush();
    } else {
      static int fail_count = 0;
      if (++fail_count % 20 == 0) {
        cout << "[VINS Process] Waiting for initialization... (Camera timestamp: "
             << fixed << setprecision(3) << message.timestamp << ")" << endl;
      }
    }
  }

  // Cleanup
  imu_thread.join();
  cam_thread.join();
  close(imu_sock);
  sem_close(sem);
  sem_close(ready_sem);
  traj_file.close();
  IpcManager::cleanup_all();

  return EXIT_SUCCESS;
}
