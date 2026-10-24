/*
 * @Descripttion: Main function to boost LARVIO (without visualization for benchmark)
 * @Author: Xiaochen Qiu
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "utils/DataReader.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include "sensors/ImageData.hpp"

#include "larvio/image_processor.h"
#include "larvio/larvio.h"

#include "Eigen/Dense"

#ifdef HAS_PANGOLAN
#include "visualization/visualize.hpp"
#endif

using namespace std;

namespace {

struct PerFrameRecord {
    int frame_idx = 0;
    double timestamp_s = 0.0;
    int64_t frame_interval_us = 0;
    int64_t pipeline_latency_us = 0;
    double arrival_s = 0.0;
    double start_s = 0.0;
    double finish_s = 0.0;
    double tx = 0.0, ty = 0.0, tz = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double bgx = 0.0, bgy = 0.0, bgz = 0.0;
    double bax = 0.0, bay = 0.0, baz = 0.0;
    int num_kfs = 0;
    int num_lms = 0;
    int num_obs = 0;
    int num_features = 0;
    int tracking_state = 0;
};

struct EvalOptions {
    bool per_frame = false;
    string per_frame_csv;
    string per_frame_json;
    int max_frames = 0;
    string dataset_name = "unknown";
};

struct StatsSummary {
    double mean = 0.0;
    double min = 0.0;
    double max = 0.0;
};

bool starts_with(const string& value, const string& prefix) {
    return value.rfind(prefix, 0) == 0;
}

bool parse_eval_options(int argc, char** argv, EvalOptions& opts) {
    for (int i = 5; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--per-frame-csv") {
            if (i + 1 >= argc) return false;
            opts.per_frame = true;
            opts.per_frame_csv = argv[++i];
        } else if (arg == "--per-frame-json") {
            if (i + 1 >= argc) return false;
            opts.per_frame_json = argv[++i];
        } else if (arg == "--max-frames") {
            if (i + 1 >= argc) return false;
            opts.max_frames = std::max(0, atoi(argv[++i]));
        } else if (arg == "--dataset-name") {
            if (i + 1 >= argc) return false;
            opts.dataset_name = argv[++i];
        } else {
            cerr << "Unknown option: " << arg << endl;
            return false;
        }
    }

    if (!opts.per_frame_csv.empty()) {
        opts.per_frame = true;
    }

    if (opts.per_frame && opts.per_frame_json.empty()) {
        opts.per_frame_json = opts.per_frame_csv;
        size_t pos = opts.per_frame_json.rfind("_framelog.csv");
        if (pos != string::npos) {
            opts.per_frame_json.replace(pos, string("_framelog.csv").size(), "_summary.json");
        } else {
            opts.per_frame_json += ".summary.json";
        }
    }

    return true;
}

StatsSummary summarize_metric(const vector<PerFrameRecord>& records, bool frame_interval) {
    vector<double> values;
    values.reserve(records.size());
    for (const auto& rec : records) {
        double value = frame_interval ? static_cast<double>(rec.frame_interval_us)
                                      : static_cast<double>(rec.pipeline_latency_us);
        if (value > 0.0) values.push_back(value);
    }

    if (values.empty()) return {};

    StatsSummary out;
    out.mean = accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
    auto mm = minmax_element(values.begin(), values.end());
    out.min = *mm.first;
    out.max = *mm.second;
    return out;
}

void write_framelog_csv(const string& path, const vector<PerFrameRecord>& records) {
    ofstream out(path);
    out << "frame_idx,timestamp_s,frame_interval_us,pipeline_latency_us,"
        << "arrival_s,start_s,finish_s,"
        << "tx,ty,tz,qx,qy,qz,qw,"
        << "vx,vy,vz,"
        << "bgx,bgy,bgz,bax,bay,baz,"
        << "num_kfs,num_lms,num_obs,num_features,tracking_state\n";

    out << fixed << setprecision(9);
    for (const auto& rec : records) {
        out << rec.frame_idx << ","
            << rec.timestamp_s << ","
            << rec.frame_interval_us << ","
            << rec.pipeline_latency_us << ","
            << rec.arrival_s << "," << rec.start_s << "," << rec.finish_s << ","
            << rec.tx << "," << rec.ty << "," << rec.tz << ","
            << rec.qx << "," << rec.qy << "," << rec.qz << "," << rec.qw << ","
            << rec.vx << "," << rec.vy << "," << rec.vz << ","
            << rec.bgx << "," << rec.bgy << "," << rec.bgz << ","
            << rec.bax << "," << rec.bay << "," << rec.baz << ","
            << rec.num_kfs << "," << rec.num_lms << "," << rec.num_obs << "," << rec.num_features << ","
            << rec.tracking_state << "\n";
    }
}

void write_summary_json(
    const string& path,
    const string& dataset_name,
    const vector<PerFrameRecord>& records,
    double exec_time_s) {

    int num_processed = 0;
    for (const auto& rec : records) {
        if (rec.tracking_state == 1) ++num_processed;
    }

    double tsr = records.empty() ? 0.0 : (100.0 * static_cast<double>(num_processed) / static_cast<double>(records.size()));
    double fps_avg = exec_time_s > 0.0 ? static_cast<double>(records.size()) / exec_time_s : 0.0;
    StatsSummary frame_interval = summarize_metric(records, true);
    StatsSummary pipeline_latency = summarize_metric(records, false);

    ofstream out(path);
    out << fixed << setprecision(6);
    out << "{\n";
    out << "  \"dataset\": \"" << dataset_name << "\",\n";
    out << "  \"num_frames_total\": " << records.size() << ",\n";
    out << "  \"num_frames_processed\": " << num_processed << ",\n";
    out << "  \"exec_time_s\": " << exec_time_s << ",\n";
    out << "  \"fps_avg\": " << fps_avg << ",\n";
    out << "  \"ate_rmse_m\": null,\n";
    out << "  \"tsr_pct\": " << tsr << ",\n";
    out << "  \"timing_us\": {\n";
    out << "    \"frame_interval\": { \"mean\": " << frame_interval.mean
        << ", \"min\": " << frame_interval.min
        << ", \"max\": " << frame_interval.max << " },\n";
    out << "    \"pipeline_latency\": { \"mean\": " << pipeline_latency.mean
        << ", \"min\": " << pipeline_latency.min
        << ", \"max\": " << pipeline_latency.max << " }\n";
    out << "  },\n";
    out << "  \"metrics\": {\n";
    out << "    \"throughput\": { \"value\": " << fps_avg << ", \"unit\": \"fps\" },\n";
    out << "    \"tsr\": { \"value\": " << tsr << ", \"unit\": \"%\" }\n";
    out << "  }\n";
    out << "}\n";
}

}  // namespace

int main(int argc, char **argv) {
#ifndef LARVIO_EVAL_MODE
    std::cout.setstate(std::ios::failbit);
#endif

#ifdef LARVIO_EVAL_MODE
    if (argc < 5) {
        cerr << endl << "Usage: ./larvio_eval imu.csv cam0.csv cam0_dir config.yaml [--per-frame-csv path] [--per-frame-json path] [--max-frames N] [--dataset-name NAME]" << endl;
        return 1;
    }
#else
    if(argc != 5) {
        cerr << endl << "Usage: ./larvio path_to_imu/data.csv path_to_cam0/data.csv path_to_cam0/data config_file_path" << endl;
        return 1;
    }
#endif

    EvalOptions eval_opts;
#ifdef LARVIO_EVAL_MODE
    if (!parse_eval_options(argc, argv, eval_opts)) {
        cerr << "Error: invalid eval options" << endl;
        return 1;
    }
#endif

    vector<larvio::ImuData> allImuData;
    vector<larvio::ImgInfo> allImgInfo;
    larvio::loadImuFile(argv[1], allImuData);
    larvio::loadImageList(argv[2], allImgInfo);

    if (eval_opts.max_frames > 0 && static_cast<size_t>(eval_opts.max_frames) < allImgInfo.size()) {
        allImgInfo.resize(static_cast<size_t>(eval_opts.max_frames));
    }

    std::string config_file(argv[4]);

    larvio::ImageProcessorPtr ImgProcesser;
    ImgProcesser.reset(new larvio::ImageProcessor(config_file));
    if (!ImgProcesser->initialize()) {
        cerr << "Image Processer initialization failed!" << endl;
        return 1;
    }

    larvio::LarVioPtr Estimator;
    Estimator.reset(new larvio::LarVio(config_file));
    if (!Estimator->initialize()) {
        cerr << "Estimator initialization failed!" << endl;
        return 1;
    }

#ifdef HAS_PANGOLAN
    pangolin::CreateWindowAndBind("LARVIO Viewer",1024,768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowActivePoints("menu.Show Active Points",true,true);
    pangolin::Var<bool> menuShowStablePoints("menu.Show Stable Points",true,true);
    pangolin::Var<bool> menuShowSlideWindow("menu.Show Slide Window",true,true);
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                pangolin::ModelViewLookAt(0,3.5,9, 0,0,0, 0,-1,0)
                );
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::View& d_image = pangolin::CreateDisplay()
            .SetBounds(pangolin::Attach::Pix(0),1.0f,pangolin::Attach::Pix(175),1/3.5f,752.0/480)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);
    bool bFollow = true;
    pangolin::OpenGlMatrix Tbw_pgl;
    std::vector<pangolin::OpenGlMatrix> vPoses;
    std::vector<pangolin::OpenGlMatrix> vPoses_SW;
    std::map<larvio::FeatureIDType,Eigen::Vector3d> mActiveMapPoints;
    std::map<larvio::FeatureIDType,Eigen::Vector3d> mStableMapPoints;
#endif

    vector<PerFrameRecord> records;
#ifdef LARVIO_EVAL_MODE
    records.reserve(allImgInfo.size());
#endif

    auto run_start = std::chrono::steady_clock::now();
    int k = 0;
    std::vector<larvio::ImuData> imu_msg_buffer;
    auto last_frame_out = run_start;
    bool has_last_frame_out = false;

    // ponytail: env-gated real-time pacing; use a proper scheduler if drift matters
    const bool paced = std::getenv("LARVIO_PACED") != nullptr;
    std::chrono::steady_clock::time_point paced_wall_start;
    double paced_data_start = -1.0;

    for(size_t j=0; j<allImgInfo.size(); j++) {
        string temp = allImgInfo[j].imgName;
        while (!temp.empty() && (temp.back() == '\n' || temp.back() == '\r')) temp.pop_back();
        string fullPath = string(argv[3]) + "/" + temp;

        if (paced) {
            double ts = allImgInfo[j].timeStampToSec;
            if (paced_data_start < 0.0) {
                paced_wall_start = std::chrono::steady_clock::now();
                paced_data_start = ts;
            } else {
                auto target = paced_wall_start +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(ts - paced_data_start));
                std::this_thread::sleep_until(target);
            }
        }

        larvio::ImageDataPtr imgPtr(new larvio::ImgData);
        imgPtr->timeStampToSec = allImgInfo[j].timeStampToSec;
        imgPtr->image = cv::imread(fullPath,0);
        if (imgPtr->image.empty()) {
            cerr << "Error: failed to read image: " << fullPath << endl;
            return 1;
        }

        while (k < static_cast<int>(allImuData.size()) &&
               allImuData[k].timeStampToSec - imgPtr->timeStampToSec < 0.05) {
            imu_msg_buffer.push_back(
                larvio::ImuData(allImuData[k].timeStampToSec, allImuData[k].angular_velocity, allImuData[k].linear_acceleration));
            ++k;
        }

        larvio::MonoCameraMeasurementPtr features = new larvio::MonoCameraMeasurement;
        auto frame_start = std::chrono::steady_clock::now();
        int64_t start_time_fe = cv::getTickCount();
        bool bProcess = ImgProcesser->processImage(imgPtr, imu_msg_buffer, features);
        int64_t end_time_fe = cv::getTickCount();
        double processing_time_fe = (end_time_fe - start_time_fe)/cv::getTickFrequency();
        double processing_time_be = 0.0;
        bool bPubOdo = false;
        if (bProcess) {
            int64_t start_time_be = cv::getTickCount();
            bPubOdo = Estimator->processFeatures(features, imu_msg_buffer);
            int64_t end_time_be = cv::getTickCount();
            processing_time_be = (end_time_be - start_time_be)/cv::getTickFrequency();
        }
        auto frame_end = std::chrono::steady_clock::now();

#ifdef LARVIO_EVAL_MODE
        if (eval_opts.per_frame) {
            PerFrameRecord rec;
            rec.frame_idx = static_cast<int>(j);
            rec.timestamp_s = imgPtr->timeStampToSec;
            rec.pipeline_latency_us = std::chrono::duration_cast<std::chrono::microseconds>(frame_end - frame_start).count();
            if (paced) {
                rec.arrival_s = imgPtr->timeStampToSec - paced_data_start;
                rec.start_s = std::chrono::duration<double>(
                    frame_start - paced_wall_start).count();
                rec.finish_s = std::chrono::duration<double>(
                    frame_end - paced_wall_start).count();
            }
            rec.frame_interval_us = has_last_frame_out
                ? std::chrono::duration_cast<std::chrono::microseconds>(frame_end - last_frame_out).count()
                : 0;
            rec.tracking_state = bPubOdo ? 1 : 0;
            if (bPubOdo) {
                Eigen::Isometry3d Tbw = Estimator->getTbw();
                Eigen::Quaterniond q(Tbw.rotation());
                Eigen::Vector3d t = Tbw.translation();
                Eigen::Vector3d v = Estimator->getVel();
                rec.tx = t.x(); rec.ty = t.y(); rec.tz = t.z();
                rec.qx = q.x(); rec.qy = q.y(); rec.qz = q.z(); rec.qw = q.w();
                rec.vx = v.x(); rec.vy = v.y(); rec.vz = v.z();
            }
            records.push_back(rec);
            last_frame_out = frame_end;
            has_last_frame_out = true;
        }
#endif

#ifdef HAS_PANGOLAN
        if (bPubOdo) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            larvio::visualize::GetCurrentOpenGLPoseMatrix(Tbw_pgl, Estimator->getTbw());
            if(menuFollowCamera && bFollow) {
                s_cam.Follow(Tbw_pgl);
            }
            else if(menuFollowCamera && !bFollow) {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,3.5,9, 0,0,0, 0,-1,0));
                s_cam.Follow(Tbw_pgl);
                bFollow = true;
            }
            else if(!menuFollowCamera && bFollow) {
                bFollow = false;
            }
            d_cam.Activate(s_cam);
            glClearColor(0.0f,0.0f,0.0f,1.0f);
            larvio::visualize::DrawCurrentPose(Tbw_pgl);
            larvio::visualize::DrawKeyFrames(vPoses);
            if (menuShowSlideWindow) {
                vector<Eigen::Isometry3d> swPoses;
                Estimator->getSwPoses(swPoses);
                larvio::visualize::GetSwOpenGLPoseMatrices(vPoses_SW, swPoses);
                larvio::visualize::DrawSlideWindow(vPoses_SW);
            }
            if (menuShowActivePoints) {
                Estimator->getActiveeMapPointPositions(mActiveMapPoints);
                larvio::visualize::DrawActiveMapPoints(mActiveMapPoints);
                mActiveMapPoints.clear();
            }
            if (menuShowStablePoints) {
                Estimator->getStableMapPointPositions(mStableMapPoints);
                larvio::visualize::DrawStableMapPoints(mStableMapPoints);
            }

            cv::Mat img4Show;
            ImgProcesser->getVisualImg().copyTo(img4Show);
            stringstream ss;
            ss << "fps: " << 1.0/(processing_time_fe+processing_time_be) << " HZ";
            string msg = ss.str();
            int baseLine=0;
            cv::Size textSize = cv::getTextSize(msg, 0, 0.01, 10, &baseLine);
            cv::Point textOrigin(2*baseLine,textSize.height+4*baseLine);
            putText(img4Show, msg, textOrigin, 1, 2, cv::Scalar(0,165,255), 2);
            pangolin::GlTexture imageTexture(img4Show.cols,img4Show.rows);;
            imageTexture.Upload(img4Show.data,GL_BGR,GL_UNSIGNED_BYTE);
            d_image.Activate();
            glColor3f(1.0,1.0,1.0);
            imageTexture.RenderToViewportFlipY();

            pangolin::FinishFrame();

            vPoses.push_back(Tbw_pgl);
        }
#endif
    }

    auto run_end = std::chrono::steady_clock::now();
    double exec_time_s = std::chrono::duration<double>(run_end - run_start).count();

#ifdef LARVIO_EVAL_MODE
    if (eval_opts.per_frame && !eval_opts.per_frame_csv.empty()) {
        write_framelog_csv(eval_opts.per_frame_csv, records);
        write_summary_json(eval_opts.per_frame_json, eval_opts.dataset_name, records, exec_time_s);
    }
#endif

#ifdef HAS_PANGOLAN
    cout << "Totally " << mStableMapPoints.size() << " SLAM points" << endl;

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if(menuFollowCamera && bFollow) {
            s_cam.Follow(Tbw_pgl);
        }
        else if(menuFollowCamera && !bFollow) {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,3.5,9, 0,0,0, 0,-1,0));
            s_cam.Follow(Tbw_pgl);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow) {
            bFollow = false;
        }
        d_cam.Activate(s_cam);
        glClearColor(0.0f,0.0f,0.0f,1.0f);
        larvio::visualize::DrawCurrentPose(Tbw_pgl);
        larvio::visualize::DrawKeyFrames(vPoses);
        if (menuShowSlideWindow) {
            larvio::visualize::DrawSlideWindow(vPoses_SW);
        }
        if (menuShowStablePoints) {
            larvio::visualize::DrawStableMapPoints(mStableMapPoints);
        }
        pangolin::FinishFrame();
    }
#endif

    return 0;
}
