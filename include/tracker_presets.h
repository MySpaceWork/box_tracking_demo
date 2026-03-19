// Tracker configuration presets for different use cases.

#ifndef TRACKER_PRESETS_H_
#define TRACKER_PRESETS_H_

#include "box_tracker.h"

namespace tracking {

// Preset profiles for different scenarios.
enum class TrackingProfile {
  STABLE,      // Most stable, rarely lost tracking (recommended for testing).
  BALANCED,    // Balanced between speed and accuracy (default).
  ACCURATE,    // Most accurate, slower.
  FAST         // Fastest, may lose tracking easier.
};

// Get configuration for a specific profile.
inline TrackerConfig GetPresetConfig(TrackingProfile profile) {
  TrackerConfig config;
  
  switch (profile) {
    case TrackingProfile::STABLE:
      // Maximum stability configuration.
      config.max_features = 700;
      config.grid_cols = 25;
      config.grid_rows = 18;
      config.pyramid_levels = 3;
      
      // Relaxed verification to keep more features.
      config.fb_verify_threshold = 2.0f;
      config.ransac_rounds = 12;
      config.ransac_inlier_threshold = 4.0f;
      
      // Relaxed inlier requirement.
      config.min_inlier_ratio = 0.10f;  // Only need 10% inliers.
      
      // More IRLS iterations for robustness.
      config.irls_iterations = 6;
      config.spatial_sigma = 0.18f;
      config.motion_sigma = 0.35f;
      
      // Strong spring force to prevent drift.
      config.spring_force = 0.18f;
      config.confidence_decay = 0.95f;
      
      // Stage 2 features: all enabled.
      config.adaptive_spring_force = true;
      config.spring_force_min = 0.08f;
      config.spring_force_max = 0.28f;
      config.max_track_length = 40;  // Allow longer tracks.
      config.temporal_smooth_weight = 0.35f;
      config.use_spatial_prior = true;
      
      // Stage 3: Use translation only for maximum stability.
      config.motion_model = TrackerConfig::MotionModel::TRANSLATION;
      config.allow_rotation = false;
      config.allow_scale = false;
      break;
      
    case TrackingProfile::BALANCED:
      // Balanced configuration (good default).
      config.max_features = 600;
      config.grid_cols = 22;
      config.grid_rows = 16;
      config.pyramid_levels = 3;
      
      config.fb_verify_threshold = 1.5f;
      config.ransac_rounds = 12;
      config.ransac_inlier_threshold = 3.5f;
      
      config.min_inlier_ratio = 0.15f;
      config.irls_iterations = 5;
      config.spatial_sigma = 0.15f;
      config.motion_sigma = 0.3f;
      
      config.spring_force = 0.15f;
      config.confidence_decay = 0.9f;
      
      // Stage 2 features.
      config.adaptive_spring_force = true;
      config.spring_force_min = 0.05f;
      config.spring_force_max = 0.25f;
      config.max_track_length = 30;
      config.temporal_smooth_weight = 0.3f;
      config.use_spatial_prior = true;
      
      // Stage 3: Similarity with conservative constraints.
      config.motion_model = TrackerConfig::MotionModel::SIMILARITY;
      config.allow_rotation = true;
      config.allow_scale = true;
      config.min_scale = 0.9f;
      config.max_scale = 1.1f;
      break;
      
    case TrackingProfile::ACCURATE:
      // High accuracy configuration.
      config.max_features = 800;
      config.grid_cols = 28;
      config.grid_rows = 20;
      config.pyramid_levels = 4;
      
      config.fb_verify_threshold = 1.0f;
      config.ransac_rounds = 15;
      config.ransac_inlier_threshold = 3.0f;
      
      config.min_inlier_ratio = 0.15f;
      config.irls_iterations = 8;
      config.spatial_sigma = 0.12f;
      config.motion_sigma = 0.25f;
      
      config.spring_force = 0.18f;
      config.confidence_decay = 0.88f;
      
      // Stage 2 features.
      config.adaptive_spring_force = true;
      config.spring_force_min = 0.08f;
      config.spring_force_max = 0.3f;
      config.max_track_length = 25;
      config.temporal_smooth_weight = 0.25f;
      config.use_spatial_prior = true;
      
      // Stage 3: Full similarity transform.
      config.motion_model = TrackerConfig::MotionModel::SIMILARITY;
      config.allow_rotation = true;
      config.allow_scale = true;
      config.min_scale = 0.85f;
      config.max_scale = 1.15f;
      break;
      
    case TrackingProfile::FAST:
      // Fast configuration, may lose tracking easier.
      config.max_features = 400;
      config.grid_cols = 18;
      config.grid_rows = 12;
      config.pyramid_levels = 3;
      
      config.fb_verify_threshold = 2.0f;
      config.ransac_rounds = 8;
      config.ransac_inlier_threshold = 4.0f;
      
      config.min_inlier_ratio = 0.15f;
      config.irls_iterations = 3;
      config.spatial_sigma = 0.15f;
      config.motion_sigma = 0.3f;
      
      config.spring_force = 0.12f;
      config.confidence_decay = 0.9f;
      
      // Stage 2: minimal features.
      config.adaptive_spring_force = false;
      config.spring_force_min = 0.05f;
      config.spring_force_max = 0.2f;
      config.max_track_length = 50;
      config.temporal_smooth_weight = 0.2f;
      config.use_spatial_prior = false;
      
      // Stage 3: Translation only for speed.
      config.motion_model = TrackerConfig::MotionModel::TRANSLATION;
      config.allow_rotation = false;
      config.allow_scale = false;
      break;
  }
  
  return config;
}

}  // namespace tracking

#endif  // TRACKER_PRESETS_H_
