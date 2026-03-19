// Example usage of the improved box tracker with presets.
// Compile: g++ -std=c++14 example_usage.cpp -I include src/box_tracker.cc `pkg-config --cflags --libs opencv4` -o example

#include "box_tracker.h"
#include "tracker_presets.h"
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <video_file> [profile]" << std::endl;
    std::cerr << "Profiles: stable (default), balanced, accurate, fast" << std::endl;
    return 1;
  }
  
  std::string video_path = argv[1];
  std::string profile_str = (argc > 2) ? argv[2] : "stable";
  
  // Select profile.
  tracking::TrackingProfile profile = tracking::TrackingProfile::STABLE;
  if (profile_str == "balanced") {
    profile = tracking::TrackingProfile::BALANCED;
  } else if (profile_str == "accurate") {
    profile = tracking::TrackingProfile::ACCURATE;
  } else if (profile_str == "fast") {
    profile = tracking::TrackingProfile::FAST;
  }
  
  std::cout << "Using profile: " << profile_str << std::endl;
  
  // Create tracker with preset.
  tracking::TrackerConfig config = tracking::GetPresetConfig(profile);
  tracking::BoxTracker tracker(config);
  
  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    std::cerr << "Cannot open video: " << video_path << std::endl;
    return 1;
  }
  
  int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
  int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  double fps = cap.get(cv::CAP_PROP_FPS);
  
  std::cout << "Video: " << frame_width << "x" << frame_height 
            << " @ " << fps << " fps" << std::endl;
  
  // Read first frame.
  cv::Mat frame;
  cap >> frame;
  if (frame.empty()) {
    std::cerr << "Cannot read first frame" << std::endl;
    return 1;
  }
  
  // Let user draw initial box.
  std::cout << "Draw a rectangle around the target, then press ENTER" << std::endl;
  cv::Rect box = cv::selectROI("Select Target", frame, false, false);
  cv::destroyWindow("Select Target");
  
  if (box.width < 10 || box.height < 10) {
    std::cerr << "Box too small" << std::endl;
    return 1;
  }
  
  std::cout << "Selected box: " << box << std::endl;
  
  // Initialize tracker.
  tracker.ProcessFrame(frame);
  tracker.SetBox(box, frame_width, frame_height);
  
  std::cout << "Tracking started. Press 'q' to quit." << std::endl;
  
  int frame_idx = 1;
  while (cap >> frame) {
    if (frame.empty()) break;
    
    tracker.ProcessFrame(frame);
    
    cv::Rect tracked_box;
    float confidence;
    if (tracker.GetTrackedBox(tracked_box, confidence)) {
      // Color by confidence: green (high) -> red (low).
      float c = std::min(1.0f, std::max(0.0f, confidence));
      cv::Scalar color(0, static_cast<int>(255 * c),
                       static_cast<int>(255 * (1.0f - c)));
      cv::rectangle(frame, tracked_box, color, 2);
      
      char text[64];
      snprintf(text, sizeof(text), "conf: %.2f", confidence);
      cv::putText(frame, text, tracked_box.tl() + cv::Point(0, -5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
      
      std::cout << "Frame " << frame_idx << ": conf=" << confidence 
                << ", inliers=" << inliers << std::endl;
    } else {
      cv::putText(frame, "Tracking lost", cv::Point(10, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
      std::cout << "Frame " << frame_idx << ": LOST" << std::endl;
    }
    
    cv::imshow("Tracking", frame);
    
    int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break;
    
    ++frame_idx;
  }
  
  std::cout << "Done. Tracked " << frame_idx << " frames" << std::endl;
  cv::destroyAllWindows();
  
  return 0;
}
