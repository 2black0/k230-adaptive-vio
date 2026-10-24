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

#include "eqvio/dataserver/UZHFPVDatasetReader.h"
#include "yaml-cpp/yaml.h"
#include <filesystem>
#include <iostream>

UZHFPVDatasetReader::UZHFPVDatasetReader(const std::string& datasetMainDir) {
    // Set up required structures
    // --------------------------

    baseDir = datasetMainDir;

    // Set up the data iterator for IMU
    IMUCSVFile = CSVFile(datasetMainDir + "imu.txt", ' ');
    IMUCSVFile.nextLine(); // skip the header

    // Read the image data file
    ImageCSVFile = CSVFile(datasetMainDir + "left_images.txt", ' ');
    ImageCSVFile.nextLine(); // skip the header

    // Read the camera file
    std::vector<std::string> candidateCameraFiles = {
        baseDir + "sensor.yaml",
        baseDir + "../indoor_forward_calib_snapdragon/camchain-imucam-..indoor_forward_calib_snapdragon_imu.yaml",
        baseDir + "../calibration/indoor_forward_calib_snapdragon/camchain-imucam-..indoor_forward_calib_snapdragon_imu.yaml",
        baseDir + "../../calibration/indoor_forward_calib_snapdragon/camchain-imucam-..indoor_forward_calib_snapdragon_imu.yaml",
    };
    for (const auto& cand : candidateCameraFiles) {
        if (std::filesystem::exists(cand)) {
            readCamera(cand);
            break;
        }
    }
}

std::unique_ptr<IMUVelocity> UZHFPVDatasetReader::nextIMU() {
    if (!IMUCSVFile) {
        return nullptr;
    }

    CSVLine imuLine = IMUCSVFile.nextLine();
    int num;
    IMUVelocity temp;
    imuLine >> num >> temp;
    return std::make_unique<IMUVelocity>(temp);
}

std::unique_ptr<StampedImage> UZHFPVDatasetReader::nextImage() {
    if (!ImageCSVFile) {
        return nullptr;
    }
    StampedImage temp;

    CSVLine imageLine = ImageCSVFile.nextLine();
    std::string nextImageFname;
    double rawStamp;
    int num;
    imageLine >> num >> rawStamp >> nextImageFname;

    nextImageFname = baseDir + nextImageFname;
    if (*nextImageFname.rbegin() == '\r') {
        nextImageFname.erase(nextImageFname.end() - 1);
    }

    temp.stamp = rawStamp - cameraLag;
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

void UZHFPVDatasetReader::readCamera(const std::string& cameraFileName) {
    if (!std::filesystem::exists(cameraFileName)) {
        return;
    }
    YAML::Node cameraFileNode = YAML::LoadFile(cameraFileName);

    if (cameraFileNode["cam0"]) {
        cameraFileNode = cameraFileNode["cam0"];
    }

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
    }

    std::array<double, 4> distortion = {0, 0, 0, 0};
    if (cameraFileNode["distortion_coeffs"]) {
        const auto v = cameraFileNode["distortion_coeffs"].as<std::vector<double>>();
        for (size_t i = 0; i < std::min<size_t>(4, v.size()); ++i) {
            distortion[i] = v[i];
        }
    } else if (cameraFileNode["distortion_coefficients"]) {
        const auto v = cameraFileNode["distortion_coefficients"].as<std::vector<double>>();
        for (size_t i = 0; i < std::min<size_t>(4, v.size()); ++i) {
            distortion[i] = v[i];
        }
    }

    camera = std::make_shared<GIFT::EquidistantCamera>(imageSize, K, distortion);

    // Read the camera extrinsics
    if (cameraFileNode["T_cam_imu"]) {
        const std::vector<std::vector<double>> extrinsics_rows =
            cameraFileNode["T_cam_imu"].as<std::vector<std::vector<double>>>();
        if (extrinsics_rows.size() == 4) {
            Eigen::Matrix4d extrinsics_matrix;
            for (int i = 0; i < 4; ++i) {
                extrinsics_matrix.row(i) = Eigen::Vector4d(extrinsics_rows[i].data()).transpose();
            }
            cameraExtrinsics = std::make_unique<liepp::SE3d>(extrinsics_matrix.inverse());
        }
    } else if (cameraFileNode["T_BS"] && cameraFileNode["T_BS"]["data"]) {
        const std::vector<double> extrinsics_entries = cameraFileNode["T_BS"]["data"].as<std::vector<double>>();
        if (extrinsics_entries.size() == 16) {
            Eigen::Matrix4d extrinsics_matrix(extrinsics_entries.data());
            extrinsics_matrix.transposeInPlace();
            cameraExtrinsics = std::make_unique<liepp::SE3d>(extrinsics_matrix);
        }
    }
}

std::vector<StampedPose> UZHFPVDatasetReader::groundtruth() {
    const std::string groundtruthFileName = baseDir + "groundtruth.txt";

    if (!std::filesystem::exists(groundtruthFileName)) {
        return {};
    }
    std::ifstream poseFile = std::ifstream(groundtruthFileName);
    CSVReader poseFileIter(poseFile, ' ');
    ++poseFileIter; // skip header

    std::vector<StampedPose> poses;

    double prevPoseTime = -1e8;
    for (CSVLine row : poseFileIter) {

        StampedPose pose;
        row >> pose.t >> pose.pose;
        if (pose.t > prevPoseTime + 1e-8) {
            // Avoid poses with the same timestamp.
            poses.emplace_back(pose);
            prevPoseTime = pose.t;
        }
    }

    return poses;
}