/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
bool loadTimestamp(string path, vector<double> &vTimestamps);

int main(int argc, char **argv) {
  if (argc != 4) {
    cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimestamps;
  LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

  const int nImages = vstrImageLeft.size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  cv::Mat imLeft, imRight;
  for (int ni = 0; ni < nImages; ni++) {
    // Read left and right images from file
    imLeft = cv::imread(vstrImageLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
    imRight = cv::imread(vstrImageRight[ni], CV_LOAD_IMAGE_UNCHANGED);
    double tframe = vTimestamps[ni] / 1e9;

    if (imLeft.empty()) {
      cerr << endl << "Failed to load image at: "
           << string(vstrImageLeft[ni]) << endl;
      return 1;
    }

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

    // Pass the images to the SLAM system
    SLAM.TrackStereo(imLeft, imRight, tframe);

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

    auto ttrack = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();

    vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    auto T = 0;
    if (ni < nImages - 1)
      T = vTimestamps[ni + 1] - tframe;
    else if (ni > 0)
      T = tframe - vTimestamps[ni - 1];

//    if (ttrack < T) {
//      cout << "sleeping for: " << (T - ttrack) * 1e6 << endl;
//      usleep((T - ttrack) * 1e6);
//    }
    auto result = SLAM.gtsam_transformer_.checkForNewData();
    if (std::get<0>(result)) {
      std::cout << "Got new data! new_states: " << std::get<4>(result)->size() << " removed_states: " << std::get<5>(result)->size() << std::endl;
      std::cout << "Most recent timestamp: " << std::get<1>(std::get<7>(result).value()) << std::endl;
    }
    usleep(1 / 10 * 1e6);
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

  return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps) {
  ifstream fTimes;
  string strPathTimeFile = strPathToSequence + "image_00/timestamps.txt";
  if (!loadTimestamp(strPathTimeFile, vTimestamps))
    return;

  string strPrefixLeft = strPathToSequence + "image_00/data/";
  string strPrefixRight = strPathToSequence + "image_01/data/";

  const int nTimes = vTimestamps.size();
  vstrImageLeft.resize(nTimes);
  vstrImageRight.resize(nTimes);

  for (int i = 0; i < nTimes; i++) {
    std::string filename = std::string(9, '0').append(std::to_string(i));
    if (filename.length() > 10)
      filename.erase(0, filename.length() - 10);
    vstrImageLeft[i] = strPrefixLeft + filename + ".png";
    vstrImageRight[i] = strPrefixRight + filename + ".png";
  }
}

bool loadTimestamp(string path, vector<double> &vTimestamps) {
  std::ifstream import_file(path, std::ios::in);
  if (!import_file) {
    return false;
  }

  vTimestamps.clear();
  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    std::string timestamp_string = line_stream.str();
    std::tm t = {};
    t.tm_year = std::stoi(timestamp_string.substr(0, 4)) - 1900;
    t.tm_mon = std::stoi(timestamp_string.substr(5, 2)) - 1;
    t.tm_mday = std::stoi(timestamp_string.substr(8, 2));
    t.tm_hour = std::stoi(timestamp_string.substr(11, 2));
    t.tm_min = std::stoi(timestamp_string.substr(14, 2));
    t.tm_sec = std::stoi(timestamp_string.substr(17, 2));
    t.tm_isdst = -1;

    static const uint64_t kSecondsToNanoSeconds = 1e9;
    time_t time_since_epoch = mktime(&t);

    uint64_t timestamp = time_since_epoch * kSecondsToNanoSeconds +
        std::stoi(timestamp_string.substr(20, 9));
    vTimestamps.push_back(timestamp);
  }

  std::cout << "Timestamps: " << std::endl
            << "from: " << vTimestamps.front()
            << " to: " << vTimestamps.back() << std::endl;
  return true;
}

