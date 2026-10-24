/*
    This file is part of EqVIO.

    EqVIO is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    EqVIO is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with EqVIO.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "eqvio/dataserver/ASLDatasetReader.h"
#include "yaml-cpp/yaml.h"
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

ASLDatasetReader::ASLDatasetReader(const std::string& datasetMainDir) {
    // Set up required structures
    // --------------------------

    // Set up the data iterator for IMU
    imuFileName = datasetMainDir + "mav0/imu0/" + "data.csv";
    IMUCSVFile = CSVFile(imuFileName);
    IMUCSVFile.nextLine(); // skip the header

    // Read the image data file
    cam_dir = datasetMainDir + "mav0/cam0/";
    ImageCSVFile = CSVFile(cam_dir + "data.csv");
    ImageCSVFile.nextLine(); // skip the header

    // Get the ground truth
    groundtruthFileName = datasetMainDir + "mav0/state_groundtruth_estimate0/data.csv";

    // Read the camera file
    std::string cameraFileName = cam_dir + "sensor.yaml";
    if (std::filesystem::exists(cameraFileName)) {
        readCamera(cameraFileName);
    }
}

std::unique_ptr<IMUVelocity> ASLDatasetReader::nextIMU() {
    // Discard everything before the quasi-static start gate.
    while (startGateStampNs > 0 && IMUCSVFile) {
        const CSVLine peek = *IMUCSVFile.reader;
        if (std::atoll(peek[0].c_str()) >= startGateStampNs) break;
        IMUCSVFile.nextLine();
    }
    if (!IMUCSVFile) {
        return nullptr;
    }

    CSVLine imuLine = IMUCSVFile.nextLine();
    IMUVelocity temp;
    imuLine >> temp;
    temp.stamp *= 1e-9;
    return std::make_unique<IMUVelocity>(temp);
}

std::unique_ptr<StampedImage> ASLDatasetReader::nextImage() {
    // Discard everything before the quasi-static start gate.
    while (startGateStampNs > 0 && ImageCSVFile) {
        const CSVLine peek = *ImageCSVFile.reader;
        if (std::atoll(peek[0].c_str()) >= startGateStampNs) break;
        ImageCSVFile.nextLine();
    }
    if (!ImageCSVFile) {
        return nullptr;
    }

    CSVLine imageLine = ImageCSVFile.nextLine();
    std::string nextImageFname;
    double rawStamp;
    imageLine >> rawStamp >> nextImageFname;
    nextImageFname = cam_dir + "data/" + nextImageFname;

    if (*nextImageFname.rbegin() == '\r') {
        nextImageFname.erase(nextImageFname.end() - 1);
    }

    StampedImage temp;
    temp.stamp = 1e-9 * rawStamp - cameraLag;
    // Grayscale, not the default 3-channel BGR. GIFT hands the image straight to
    // calcOpticalFlowPyrLK, which then does three channels' worth of pyramid and
    // gradient work on data where all three are identical: 13.53 ms against
    // 4.52 ms per frame on the K230 at 376x240, a 2.99x cost for nothing. The
    // tracked positions agree to 0.00009 px, so this is free.
    // (GIFT only converts internally when equaliseImageHistogram is on, which
    // is why the TUM-VI config never hit this.)
    temp.image = cv::imread(nextImageFname, cv::IMREAD_GRAYSCALE);
    return std::make_unique<StampedImage>(temp);
}

void ASLDatasetReader::readCamera(const std::string& cameraFileName) {
    if (!std::filesystem::exists(cameraFileName)) {
        return;
    }
    YAML::Node cameraFileNode = YAML::LoadFile(cameraFileName);
    if (cameraFileNode["cam0"]) {
        cameraFileNode = cameraFileNode["cam0"];
    }

    // Read the intrinsics
    cv::Size imageSize(0, 0);
    if (cameraFileNode["resolution"]) {
        imageSize.width = cameraFileNode["resolution"][0].as<int>();
        imageSize.height = cameraFileNode["resolution"][1].as<int>();
    }

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    if (cameraFileNode["intrinsics"]) {
        K.at<double>(0, 0) = cameraFileNode["intrinsics"][0].as<double>();
        K.at<double>(1, 1) = cameraFileNode["intrinsics"][1].as<double>();
        K.at<double>(0, 2) = cameraFileNode["intrinsics"][2].as<double>();
        K.at<double>(1, 2) = cameraFileNode["intrinsics"][3].as<double>();
    } else if (cameraFileNode["camera_matrix"] && cameraFileNode["camera_matrix"]["data"]) {
        const std::vector<double> k_data = cameraFileNode["camera_matrix"]["data"].as<std::vector<double>>();
        if (k_data.size() == 9) {
            K.at<double>(0, 0) = k_data[0];
            K.at<double>(0, 2) = k_data[2];
            K.at<double>(1, 1) = k_data[4];
            K.at<double>(1, 2) = k_data[5];
        }
    }

    std::vector<double> distortion;
    if (cameraFileNode["distortion_coefficients"]) {
        distortion = cameraFileNode["distortion_coefficients"].as<std::vector<double>>();
    } else if (cameraFileNode["distortion_coeffs"]) {
        distortion = cameraFileNode["distortion_coeffs"].as<std::vector<double>>();
    } else if (cameraFileNode["distortion"]) {
        distortion = cameraFileNode["distortion"].as<std::vector<double>>();
    }

    std::string distortionModel = "";
    if (cameraFileNode["distortion_model"]) {
        distortionModel = cameraFileNode["distortion_model"].as<std::string>();
    }

    if (distortionModel == "fov" || distortionModel == "field-of-view") {
        const double omega = distortion.empty() ? 0.0 : distortion.front();
        this->camera = std::make_shared<GIFT::FOVCamera>(imageSize, K, omega);
    } else if (distortionModel == "equidistant" || distortionModel == "kb4" || distortionModel == "fisheye") {
        std::array<double, 4> distArray = {0, 0, 0, 0};
        for (size_t i = 0; i < std::min<size_t>(4, distortion.size()); ++i) {
            distArray[i] = distortion[i];
        }
        this->camera = std::make_shared<GIFT::EquidistantCamera>(imageSize, K, distArray);
    } else if (distortionModel == "doublesphere") {
        std::vector<double> dsIntrinsics;
        if (cameraFileNode["intrinsics"]) {
            dsIntrinsics = cameraFileNode["intrinsics"].as<std::vector<double>>();
        }
        dsIntrinsics.insert(dsIntrinsics.end(), distortion.begin(), distortion.end());
        std::array<double, 6> dsArray = {0, 0, 0, 0, 0, 0};
        for (size_t i = 0; i < std::min<size_t>(6, dsIntrinsics.size()); ++i) {
            dsArray[i] = dsIntrinsics[i];
        }
        this->camera = std::make_shared<GIFT::DoubleSphereCamera>(dsArray, imageSize);
    } else {
        GIFT::StandardCamera stdCamera = GIFT::StandardCamera(imageSize, K, distortion);
        this->camera = std::make_shared<GIFT::StandardCamera>(stdCamera);
    }

    // Read the extrinsics
    if (cameraFileNode["T_BS"] && cameraFileNode["T_BS"]["data"]) {
        const std::vector<double> extrinsics_entries = cameraFileNode["T_BS"]["data"].as<std::vector<double>>();
        if (extrinsics_entries.size() == 16) {
            Eigen::Matrix4d extrinsics_matrix(extrinsics_entries.data());
            extrinsics_matrix.transposeInPlace();
            cameraExtrinsics = std::make_unique<liepp::SE3d>(extrinsics_matrix);
        }
    } else if (cameraFileNode["T_cam_imu"]) {
        const std::vector<std::vector<double>> extrinsics_rows =
            cameraFileNode["T_cam_imu"].as<std::vector<std::vector<double>>>();
        if (extrinsics_rows.size() == 4) {
            Eigen::Matrix4d extrinsics_matrix;
            for (int i = 0; i < 4; ++i) {
                extrinsics_matrix.row(i) = Eigen::Vector4d(extrinsics_rows[i].data()).transpose();
            }
            cameraExtrinsics = std::make_unique<liepp::SE3d>(extrinsics_matrix.inverse());
        }
    } else if (cameraFileNode["T_imu_cam"]) {
        const std::vector<std::vector<double>> extrinsics_rows =
            cameraFileNode["T_imu_cam"].as<std::vector<std::vector<double>>>();
        if (extrinsics_rows.size() == 4) {
            Eigen::Matrix4d extrinsics_matrix;
            for (int i = 0; i < 4; ++i) {
                extrinsics_matrix.row(i) = Eigen::Vector4d(extrinsics_rows[i].data()).transpose();
            }
            cameraExtrinsics = std::make_unique<liepp::SE3d>(extrinsics_matrix);
        }
    }
}

void ASLDatasetReader::enableAutoStartGate(double maxWait) {
    std::ifstream imuFile(imuFileName);
    if (!imuFile.is_open()) {
        return;
    }
    double firstStamp = -1.0;
    const double gateRel = detectQuasiStaticStart(imuFile, maxWait, &firstStamp);
    if (gateRel < 0.0 || firstStamp < 0.0) {
        std::cout << "Auto-start gate: no quasi-static window within " << maxWait
                  << " s; starting at the beginning of the data." << std::endl;
        return;
    }
    startGateStampNs = static_cast<long long int>(firstStamp + gateRel * 1e9);
    std::cout << "Auto-start gate: quasi-static window found at t+" << gateRel
              << " s; the estimator starts there." << std::endl;
}

double ASLDatasetReader::detectQuasiStaticStart(std::ifstream& imuFile, double maxWait, double* firstStampOut) const {
    // A 1.5 s window qualifies when the magnitude standard deviations of both
    // the gyro and the accelerometer stay below these thresholds for a further
    // 2 s. The values were read off the EuRoC sequences: slow translation
    // keeps the gyro above 0.09 rad/s while a stationary platform stays below
    // 0.05, so the two phases separate cleanly.
    const double winLen = 1.5, holdLen = 2.0, gyroThr = 0.05, accThr = 0.15;
    struct Sample {
        double t, w, a;
    };
    std::deque<Sample> win;
    std::string line;
    double firstStamp = -1.0, holdStart = -1.0;
    while (std::getline(imuFile, line)) {
        if (line.empty() || line[0] == '#') continue;
        double vals[7];
        int n = 0;
        std::stringstream lineStream(line);
        std::string tok;
        while (n < 7 && std::getline(lineStream, tok, ',')) vals[n++] = std::stod(tok);
        if (n < 7) continue;
        const double stamp = vals[0];
        if (firstStamp < 0.0) firstStamp = stamp;
        const double rel = (stamp - firstStamp) * 1e-9;
        if (rel > maxWait) break;
        const double wNorm = std::sqrt(vals[1] * vals[1] + vals[2] * vals[2] + vals[3] * vals[3]);
        const double aNorm = std::sqrt(vals[4] * vals[4] + vals[5] * vals[5] + vals[6] * vals[6]);
        win.push_back({rel, wNorm, aNorm});
        while (!win.empty() && win.front().t < rel - winLen) win.pop_front();
        if (win.size() < 10) continue;
        double wMean = 0.0, aMean = 0.0;
        for (const auto& s : win) {
            wMean += s.w;
            aMean += s.a;
        }
        wMean /= win.size();
        aMean /= win.size();
        double wVar = 0.0, aVar = 0.0;
        for (const auto& s : win) {
            wVar += (s.w - wMean) * (s.w - wMean);
            aVar += (s.a - aMean) * (s.a - aMean);
        }
        wVar /= win.size();
        aVar /= win.size();
        if (std::sqrt(wVar) < gyroThr && std::sqrt(aVar) < accThr) {
            if (holdStart < 0.0) holdStart = win.front().t;
            if (rel - holdStart >= holdLen) {
                if (firstStampOut) *firstStampOut = firstStamp;
                return holdStart;
            }
        } else {
            holdStart = -1.0;
        }
    }
    if (firstStampOut) *firstStampOut = firstStamp;
    return -1.0;
}

std::vector<StampedPose> ASLDatasetReader::groundtruth() {
    // Find the poses file
    if (!std::filesystem::exists(groundtruthFileName)) {
        return {};
    }
    std::ifstream poseFile = std::ifstream(groundtruthFileName);
    CSVReader poseFileIter(poseFile, ',');
    ++poseFileIter; // skip header

    std::vector<StampedPose> poses;

    double prevPoseTime = -1e8;
    for (CSVLine row : poseFileIter) {
        StampedPose pose;
        row >> pose.t >> pose.pose;
        pose.t *= 1.0e-9;
        if (pose.t > prevPoseTime + 1e-8) {
            // Avoid poses with the same timestamp.
            poses.emplace_back(pose);
            prevPoseTime = pose.t;
        }
    }

    return poses;
}
