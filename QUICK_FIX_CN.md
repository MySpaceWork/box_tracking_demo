# 快速修复 "Tracking Lost" 问题

## 🔍 问题诊断

如果运行阶段3时仍然出现 "tracking lost"，可能是以下原因：

### 1. 默认配置过于激进

相似变换模型需要更多的特征点和更宽松的约束。

### 2. 可能的解决方案

创建一个推荐配置文件：

**文件**: `include/tracker_presets.h`

```cpp
#ifndef TRACKER_PRESETS_H_
#define TRACKER_PRESETS_H_

#include "box_tracker.h"

namespace tracking {

// 获取推荐的配置预设
inline TrackerConfig GetBalancedConfig() {
  TrackerConfig config;
  
  // 基础参数
  config.max_features = 600;
  config.grid_cols = 22;
  config.grid_rows = 16;
  config.pyramid_levels = 3;
  
  // RANSAC
  config.ransac_rounds = 12;
  config.ransac_inlier_threshold = 3.5f;
  
  // IRLS
  config.irls_iterations = 5;
  config.spatial_sigma = 0.15f;
  config.motion_sigma = 0.3f;
  
  // 验证和跟踪
  config.fb_verify_threshold = 1.5f;
  config.min_inlier_ratio = 0.15f;
  config.confidence_decay = 0.9f;
  
  // 阶段2：防漂移
  config.spring_force = 0.15f;
  config.adaptive_spring_force = true;
  config.spring_force_min = 0.05f;
  config.spring_force_max = 0.25f;
  config.max_track_length = 30;
  config.temporal_smooth_weight = 0.3f;
  config.use_spatial_prior = true;
  
  // 阶段3：相似变换（默认启用）
  config.motion_model = TrackerConfig::MotionModel::SIMILARITY;
  config.allow_rotation = true;
  config.allow_scale = true;
  config.min_scale = 0.85f;
  config.max_scale = 1.15f;
  
  return config;
}

// 保守配置（更稳定，不易 tracking lost）
inline TrackerConfig GetStableConfig() {
  TrackerConfig config;
  
  // 更多特征点
  config.max_features = 700;
  config.grid_cols = 25;
  config.grid_rows = 18;
  
  // 更宽松的验证
  config.fb_verify_threshold = 2.0f;
  config.ransac_inlier_threshold = 4.0f;
  config.min_inlier_ratio = 0.12f;  // 降低最小内点比例
  
  // 更强的弹簧力
  config.spring_force = 0.2f;
  config.spring_force_max = 0.3f;
  
  // 更多 IRLS 迭代
  config.irls_iterations = 6;
  
  // 相似变换（但限制范围）
  config.motion_model = TrackerConfig::MotionModel::SIMILARITY;
  config.allow_rotation = true;
  config.allow_scale = true;
  config.min_scale = 0.9f;   // 更保守的缩放范围
  config.max_scale = 1.1f;
  
  return config;
}

// 如果相似变换有问题，使用纯平移
inline TrackerConfig GetTranslationOnlyConfig() {
  TrackerConfig config = GetBalancedConfig();
  
  // 禁用相似变换，回退到纯平移
  config.motion_model = TrackerConfig::MotionModel::TRANSLATION;
  
  return config;
}

}  // namespace tracking

#endif  // TRACKER_PRESETS_H_
```

---

## 💡 临时修复（快速测试）

如果不想创建新文件，在 `main.cc` 中直接修改：

**文件**: `src/main.cc` (第 143 行附近)

```cpp
// 原来的代码：
tracking::TrackerConfig config;
tracking::BoxTracker tracker(config);

// 改为：
tracking::TrackerConfig config;

// 方案 1：使用纯平移模型（最稳定）
config.motion_model = tracking::TrackerConfig::MotionModel::TRANSLATION;
config.spring_force = 0.15f;
config.fb_verify_threshold = 1.5f;
config.adaptive_spring_force = true;
config.spring_force_min = 0.05f;
config.spring_force_max = 0.25f;
config.max_track_length = 30;
config.use_spatial_prior = true;

tracking::BoxTracker tracker(config);
```

---

## 🔧 调试步骤

### 步骤 1：添加调试输出

在 `src/box_tracker.cc` 的 `MotionBoxTracker::TrackStep` 开头添加：

```cpp
BoxState MotionBoxTracker::TrackStep(...) {
  if (!initialized_) {
    std::cout << "[DEBUG] Not initialized!" << std::endl;
    BoxState invalid;
    invalid.tracked = false;
    return invalid;
  }
  
  BoxState next_state = state_;
  std::cout << "[DEBUG] Current state: x=" << state_.x << ", y=" << state_.y 
            << ", w=" << state_.width << ", h=" << state_.height << std::endl;

  // ... 原有代码 ...
  
  std::vector<const MotionVector*> selected;
  std::vector<float> prior_weights;
  GetVectorsAndWeights(data.motion_vectors, state_, selected, prior_weights);
  
  std::cout << "[DEBUG] Selected vectors: " << selected.size() << std::endl;
  std::cout << "[DEBUG] Motion vectors: " << data.motion_vectors.size() << std::endl;

  if (selected.size() < 3) {
    std::cout << "[DEBUG] Not enough vectors (< 3)" << std::endl;
    // ... 返回失败 ...
  }
  
  // ... 继续跟踪 ...
}
```

### 步骤 2：检查初始 box 大小

在 `src/main.cc` 绘制 box 后添加：

```cpp
std::cout << "Selected: " << selected_rect << std::endl;

// 新增：检查 box 大小
if (selected_rect.width < 50 || selected_rect.height < 50) {
  std::cout << "WARNING: Box too small! May cause tracking failure." << std::endl;
  std::cout << "Please draw a larger box (>50x50 pixels)." << std::endl;
}
```

### 步骤 3：检查特征点提取

在 `FlowComputation::ProcessFrame` 中添加：

```cpp
std::vector<TrackedFeature> FlowComputation::ProcessFrame(...) {
  // ... 代码 ...
  
  // 在返回前添加
  std::cout << "[DEBUG] Extracted features: " << result.size() << std::endl;
  return result;
}
```

---

## 🎯 推荐的测试配置

创建一个专门的测试配置类：

```cpp
// 在 main.cc 的 main 函数开头
tracking::TrackerConfig config;

// === 临时修复：使用保守配置 ===
config.max_features = 700;
config.grid_cols = 25;
config.grid_rows = 18;

// 更宽松的验证阈值
config.fb_verify_threshold = 2.0f;
config.ransac_inlier_threshold = 4.0f;
config.min_inlier_ratio = 0.10f;  // 降低到 10%

// 使用纯平移模式（最稳定）
config.motion_model = tracking::TrackerConfig::MotionModel::TRANSLATION;

// 阶段2功能
config.adaptive_spring_force = true;
config.spring_force = 0.15f;
config.spring_force_min = 0.05f;
config.spring_force_max = 0.25f;
config.max_track_length = 30;
config.use_spatial_prior = true;

tracking::BoxTracker tracker(config);
```

---

## 📝 分步测试流程

### 测试 1：最小配置（纯平移）

```bash
# 修改 main.cc 使用纯平移
config.motion_model = TrackerConfig::MotionModel::TRANSLATION;

# 重新编译
cd build && make -j4

# 运行测试
./box_tracking_demo test_video.mp4
```

**预期**：应该能正常跟踪，不会 tracking lost

### 测试 2：启用相似变换

如果测试1成功，则启用相似变换：

```cpp
config.motion_model = TrackerConfig::MotionModel::SIMILARITY;
config.allow_rotation = false;  // 先只测试缩放
config.allow_scale = true;
config.min_scale = 0.9f;
config.max_scale = 1.1f;
```

### 测试 3：完全启用

如果测试2成功，完全启用：

```cpp
config.allow_rotation = true;
config.allow_scale = true;
```

---

## ⚠️ 已知问题和解决方案

### 问题 1：第一帧就 tracking lost

**可能原因**：
- 绘制的 box 太小
- 特征点提取失败

**解决**：
```cpp
// 在 main.cc 中检查 box 大小
if (selected_rect.width < 40 || selected_rect.height < 40) {
  std::cerr << "Box too small! Please draw larger." << std::endl;
  continue;  // 重新绘制
}
```

### 问题 2：几帧后 tracking lost

**可能原因**：
- min_inlier_ratio 太高（0.15）
- 弹簧力过强导致跳出目标

**解决**：
```cpp
config.min_inlier_ratio = 0.10f;  // 从 0.15 降低
config.spring_force_max = 0.2f;   // 从 0.25 降低
```

### 问题 3：相似变换求解失败

**诊断**：查看是否有 fallback 信息

**解决**：
```cpp
// 在 EstimateSimilarity 的 fallback 处添加：
if (!cv::solve(A, b, solution, cv::DECOMP_QR)) {
  std::cout << "[WARNING] Similarity solve failed, using translation" << std::endl;
  // ... fallback 逻辑 ...
}
```

---

## 🚀 立即修复（patch）

如果仍有问题，使用这个最保守的配置：

```cpp
// 在 main.cc 第 143 行左右
tracking::TrackerConfig config;

// 最大化稳定性
config.max_features = 800;
config.min_inlier_ratio = 0.08f;  // 非常宽松
config.fb_verify_threshold = 2.5f;  // 宽松验证
config.spring_force = 0.12f;  // 温和修正

// 禁用可能导致问题的功能
config.motion_model = tracking::TrackerConfig::MotionModel::TRANSLATION;  // 先用平移
config.max_track_length = 100;  // 暂时不限制

// 保留有用的功能
config.adaptive_spring_force = true;
config.use_spatial_prior = false;  // 暂时禁用

tracking::BoxTracker tracker(config);
```

**如果这样还不行，说明可能有其他 bug。**

---

## 📧 报告问题

如果以上都无效，请提供：
1. 使用的配置
2. 视频分辨率
3. 绘制的 box 大小
4. 调试输出信息

我会进一步排查！
