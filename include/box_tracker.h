// Box Tracker -- Standalone implementation inspired by MediaPipe's tracking
// module.
//
// Algorithm: KLT optical flow + RANSAC outlier rejection + IRLS motion
// estimation for object box tracking.
//
// Dependencies: OpenCV only.

#ifndef BOX_TRACKER_H_
#define BOX_TRACKER_H_

#include <array>
#include <cmath>
#include <random>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

namespace tracking {

// A tracked feature point with its flow and metadata.
struct TrackedFeature {
  cv::Point2f position;       // Feature position in current frame.
  cv::Point2f flow;           // Motion vector (current - previous).
  float irls_weight = 1.0f;   // IRLS weight after estimation.
  int track_id = -1;          // Unique track ID for long tracks.
  bool is_inlier = true;
};

// Motion vector decomposed into background (camera) and object components.
struct MotionVector {
  cv::Point2f pos;         // Position (normalized [0,1]).
  cv::Point2f background;  // Camera motion component.
  cv::Point2f object;      // Object-specific motion.
  int track_id = -1;

  cv::Point2f total_motion() const {
    return background + object;
  }
};

// Tracked box state.
struct BoxState {
  // Position and size in normalized coordinates [0, 1].
  float x = 0;       // top-left x
  float y = 0;       // top-left y
  float width = 0;
  float height = 0;
  float rotation = 0;  // radians
  float dx = 0;        // velocity x
  float dy = 0;        // velocity y
  float scale = 1.0f;  // Stage 3: scale factor

  // Tracking quality metrics.
  float confidence = 0;
  int num_inliers = 0;
  bool tracked = false;

  cv::Point2f center() const {
    return cv::Point2f(x + width * 0.5f, y + height * 0.5f);
  }

  cv::Rect2f rect() const { return cv::Rect2f(x, y, width, height); }
};

// Per-frame tracking data: features + camera motion.
struct FrameTrackingData {
  std::vector<MotionVector> motion_vectors;
  cv::Matx33f camera_homography = cv::Matx33f::eye();
  bool valid_camera_model = true;
  float duration_ms = 33.3f;
};

// Configuration for the tracker.
struct TrackerConfig {
  // KLT optical flow parameters.
  int max_features = 500;
  int pyramid_levels = 3;
  cv::Size klt_window = cv::Size(21, 21);
  float klt_min_eigen_threshold = 0.001f;

  // Feature grid for uniform distribution.
  int grid_cols = 20;
  int grid_rows = 15;

  // RANSAC parameters for region flow outlier rejection.
  int ransac_rounds = 10;
  float ransac_inlier_threshold = 3.0f;  // pixels

  // IRLS parameters for box motion estimation.
  int irls_iterations = 5;
  float spatial_sigma = 0.15f;  // relative to box size
  float motion_sigma = 0.3f;   // relative to previous motion

  // Box tracking behavior.
  float min_inlier_ratio = 0.15f;
  float spring_force = 0.15f;              // Optimized from 0.1
  float confidence_decay = 0.9f;

  // Forward-backward verification threshold (pixels).
  float fb_verify_threshold = 1.5f;        // Optimized from 2.0
  
  // Stage 2: Advanced tracking features.
  bool adaptive_spring_force = true;
  float spring_force_max = 0.25f;
  float spring_force_min = 0.05f;
  int max_track_length = 30;
  float temporal_smooth_weight = 0.3f;
  bool use_spatial_prior = true;
  
  // Stage 3: Motion model selection.
  enum class MotionModel {
    TRANSLATION,    // Translation only (default, original).
    SIMILARITY      // Translation + rotation + scale.
  };
  MotionModel motion_model = MotionModel::SIMILARITY;
  bool allow_rotation = true;
  bool allow_scale = true;
  float min_scale = 0.8f;     // Minimum allowed scale.
  float max_scale = 1.2f;     // Maximum allowed scale.
};

// Computes optical flow features between frames.
class FlowComputation {
 public:
  explicit FlowComputation(const TrackerConfig& config);

  // Process a new frame. Returns tracked features from previous to current.
  // First call returns empty features (no previous frame).
  std::vector<TrackedFeature> ProcessFrame(const cv::Mat& gray_frame);

  void Reset();

 private:
  // Extract features uniformly distributed on a grid.
  void ExtractGridFeatures(const cv::Mat& gray, std::vector<cv::Point2f>& pts);

  TrackerConfig config_;
  cv::Mat prev_gray_;
  std::vector<cv::Point2f> prev_points_;
  std::vector<int> prev_track_ids_;
  int next_track_id_ = 0;
  bool has_prev_ = false;
};

// Estimates camera motion model from tracked features using IRLS.
class CameraMotionEstimator {
 public:
  explicit CameraMotionEstimator(const TrackerConfig& config);

  // Estimate camera homography from features.
  // Updates features' irls_weight and is_inlier fields.
  cv::Matx33f Estimate(std::vector<TrackedFeature>& features,
                       int frame_width, int frame_height);

 private:
  TrackerConfig config_;
};

// Stage 3: Similarity transform result.
struct SimilarityTransform {
  cv::Point2f translation;
  float scale = 1.0f;
  float rotation = 0.0f;
};

// Tracks a single box across frames using motion vectors.
class MotionBoxTracker {
 public:
  explicit MotionBoxTracker(const TrackerConfig& config);

  // Initialize tracking with an initial box (normalized coordinates).
  void Init(const BoxState& initial_state);

  // Track one step forward using frame tracking data.
  // Returns updated BoxState.
  BoxState TrackStep(const FrameTrackingData& data, float aspect_ratio);

  // Get current state.
  const BoxState& state() const { return state_; }
  bool is_initialized() const { return initialized_; }

 private:
  // Get motion vectors within or near the box.
  void GetVectorsAndWeights(
      const std::vector<MotionVector>& vectors,
      const BoxState& box,
      std::vector<const MotionVector*>& selected,
      std::vector<float>& weights);

  // RANSAC initialization for translation estimation.
  void RansacTranslationInit(
      const std::vector<const MotionVector*>& vectors,
      std::vector<float>& weights);

  // IRLS translation estimation.
  cv::Point2f EstimateTranslation(
      const std::vector<const MotionVector*>& vectors,
      const std::vector<float>& prior_weights,
      std::vector<float>& weights);
  
  // Stage 3: IRLS similarity transform estimation.
  SimilarityTransform EstimateSimilarity(
      const std::vector<const MotionVector*>& vectors,
      const std::vector<float>& prior_weights,
      std::vector<float>& weights);

  // Score inliers and compute confidence.
  float ScoreInliers(
      const std::vector<const MotionVector*>& vectors,
      const std::vector<float>& weights,
      const cv::Point2f& translation,
      BoxState& next_state);
  
  // Stage 3: Score inliers for similarity transform.
  float ScoreInliersSimilarity(
      const std::vector<const MotionVector*>& vectors,
      const std::vector<float>& weights,
      const SimilarityTransform& transform,
      BoxState& next_state);

  TrackerConfig config_;
  BoxState state_;
  bool initialized_ = false;
  std::default_random_engine rng_{42};
};

// High-level tracker that combines all components.
// Usage:
//   BoxTracker tracker(config);
//   tracker.ProcessFrame(frame);  // first frame, empty
//   tracker.SetBox(rect, frame_w, frame_h);
//   while (has_frames) {
//     tracker.ProcessFrame(frame);
//     BoxState s = tracker.GetTrackedBox();
//   }
class BoxTracker {
 public:
  explicit BoxTracker(const TrackerConfig& config = TrackerConfig());

  // Process a new video frame (BGR or grayscale).
  void ProcessFrame(const cv::Mat& frame);

  // Set the initial tracking box (in pixel coordinates).
  void SetBox(const cv::Rect& rect, int frame_width, int frame_height);

  // Get current tracked box in pixel coordinates.
  // Returns false if tracking is not active or lost.
  bool GetTrackedBox(cv::Rect& out_rect, float& out_confidence) const;

  // Reset tracker state.
  void Reset();

  bool is_tracking() const { return tracking_active_; }

 private:
  TrackerConfig config_;
  FlowComputation flow_;
  CameraMotionEstimator camera_estimator_;
  MotionBoxTracker box_tracker_;

  int frame_width_ = 0;
  int frame_height_ = 0;
  bool tracking_active_ = false;
};

}  // namespace tracking

#endif  // BOX_TRACKER_H_
