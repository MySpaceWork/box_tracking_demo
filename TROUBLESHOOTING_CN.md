# 故障排除指南

## 问题 1：编译错误 - EstimateTranslation 返回类型不匹配

**错误信息**：
```
error: could not convert 'EstimateTranslation(...)' from 'cv::Point2f' to 'tracking::SimilarityTransform'
```

**原因**：在 `EstimateSimilarity` 函数中，fallback 逻辑直接返回 `EstimateTranslation` 的结果（`cv::Point2f`），但函数期望返回 `SimilarityTransform`。

**修复**：已在最新提交中修复（commit 2c88a21）
```cpp
// 修复前：
return EstimateTranslation(vectors, prior_weights, weights);

// 修复后：
cv::Point2f trans = EstimateTranslation(vectors, prior_weights, weights);
result.translation = trans;
result.scale = 1.0f;
result.rotation = 0.0f;
return result;
```

---

## 问题 2：运行时显示 "tracking lost"，无跟踪框

**症状**：
- 绘制矩形框后，左上角立即显示 "Tracking lost"
- 画面中没有绿色跟踪框
- 置信度为 0

**原因**：
1. `BoxState` 的新字段（`prev_inlier_center`, `inlier_density_map`）未初始化
2. `ScoreInliersSimilarity` 函数缺少时间平滑和空间先验逻辑

**修复**：已在最新提交中修复（commit 7f64c37）

### 修复 1：初始化新字段

**文件**：`src/box_tracker.cc`

```cpp
void MotionBoxTracker::Init(const BoxState& initial_state) {
  state_ = initial_state;
  state_.tracked = true;
  state_.confidence = 1.0f;
  
  // 修复：初始化 prev_inlier_center 为 box 中心
  state_.prev_inlier_center = state_.center();
  
  // 修复：初始化空间先验网格
  if (config_.use_spatial_prior) {
    state_.inlier_density_map = cv::Mat::zeros(3, 3, CV_32F);
  }
  
  initialized_ = true;
}
```

### 修复 2：ScoreInliersSimilarity 添加时间平滑

**文件**：`src/box_tracker.cc`

```cpp
float MotionBoxTracker::ScoreInliersSimilarity(...) {
  // ... 计算 inlier_center ...
  
  // 修复：添加时间平滑
  cv::Point2f prev_center = state_.prev_inlier_center;
  float prev_mag = cv::norm(prev_center);
  if (prev_mag > 0.001f) {  // 检查是否已初始化
    float rel_change = cv::norm(inlier_center - prev_center) / box_size;
    float blend_weight = std::min(0.5f, rel_change * 2.0f);
    inlier_center = (1.0f - blend_weight) * inlier_center + blend_weight * prev_center;
  }
  
  // 修复：存储当前 inlier_center
  next_state.prev_inlier_center = inlier_center;
  
  // 修复：添加空间先验更新
  if (config_.use_spatial_prior && !density_map.empty()) {
    next_state.inlier_density_map = 0.7f * density_map + 0.3f * state_.inlier_density_map;
  }
  
  // ... 其余代码 ...
}
```

**验证修复**：
```bash
# 重新拉取最新代码
git pull origin feature/stage-3-similarity-transform

# 重新编译
rm -rf build && mkdir build && cd build
cmake .. -DUSE_EIGEN3=OFF
make -j$(nproc)

# 测试运行
./box_tracking_demo /path/to/video.mp4
```

---

## 问题 3：跟踪出现抖动

**症状**：box 在目标周围抖动

**原因**：
- 旋转/缩放估计在每帧间波动
- 弹簧力过强

**解决方案**：

### 方案 1：限制旋转和缩放范围
```cpp
tracking::TrackerConfig config;
config.min_scale = 0.95f;  // 限制为 95%-105%
config.max_scale = 1.05f;
```

### 方案 2：禁用旋转
```cpp
config.allow_rotation = false;  // 只估计平移和缩放
```

### 方案 3：增加时间平滑
```cpp
config.temporal_smooth_weight = 0.5f;  // 更多历史权重（从 0.3 增加）
```

### 方案 4：降低弹簧力
```cpp
config.spring_force = 0.12f;  // 从 0.15 降低
config.spring_force_max = 0.2f;  // 从 0.25 降低
```

---

## 问题 4：跟踪仍有漂移

**症状**：长时间跟踪后，box 偏移目标

**解决方案**：

### 方案 1：更严格的前后验证
```cpp
config.fb_verify_threshold = 1.0f;  // 从 1.5 减小
```

### 方案 2：限制长特征
```cpp
config.max_track_length = 20;  // 从 30 减小到 20
```

### 方案 3：增强弹簧力
```cpp
config.spring_force = 0.2f;  // 从 0.15 增加
config.spring_force_max = 0.3f;  // 从 0.25 增加
```

### 方案 4：增加 IRLS 迭代
```cpp
config.irls_iterations = 8;  // 从 5 增加
```

---

## 问题 5：Eigen3 未找到

**错误信息**：
```
CMake Error: Could not find Eigen3
```

**解决方案**：

### 方案 1：安装 Eigen3
```bash
# Ubuntu/Debian
sudo apt-get install libeigen3-dev

# macOS
brew install eigen

# 验证安装
pkg-config --modversion eigen3
```

### 方案 2：使用 OpenCV 实现（无需 Eigen3）
```bash
mkdir build && cd build
cmake .. -DUSE_EIGEN3=OFF
make -j$(nproc)
```

**性能差异**：
- Eigen3 实现：~35 FPS
- OpenCV 实现：~32 FPS
- 功能完全相同！

---

## 问题 6：编译报错 - M_PI 未定义

**错误信息**：
```
error: 'M_PI' was not declared in this scope
```

**原因**：某些编译器需要定义 `_USE_MATH_DEFINES`

**解决方案**：

### 方案 1：在 CMakeLists.txt 中添加
```cmake
add_definitions(-D_USE_MATH_DEFINES)
```

### 方案 2：在源码中定义
在 `src/box_tracker.cc` 开头添加：
```cpp
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
```

---

## 问题 7：运行时崩溃 - 空指针访问

**症状**：运行时 Segmentation fault

**可能原因**：
1. `inlier_density_map` 未初始化就访问
2. `vectors` 为空时访问

**调试步骤**：

```bash
# 使用 gdb 调试
gdb ./box_tracking_demo
(gdb) run video.mp4
# 崩溃后
(gdb) bt  # 查看堆栈
```

**预防**：代码中已添加检查
```cpp
if (config_.use_spatial_prior && !density_map.empty()) {
  // 安全访问
}

if (prev_mag > 0.001f) {  // 检查是否初始化
  // 使用 prev_center
}
```

---

## 问题 8：性能过慢

**症状**：FPS < 20

**可能原因**：
1. 未使用 Eigen3
2. 特征点过多
3. IRLS 迭代过多

**解决方案**：

### 使用快速配置
```cpp
tracking::TrackerConfig config;

// 减少特征点
config.max_features = 300;
config.grid_cols = 15;
config.grid_rows = 10;

// 减少迭代
config.irls_iterations = 3;
config.ransac_rounds = 5;

// 使用平移模型（最快）
config.motion_model = tracking::TrackerConfig::MotionModel::TRANSLATION;
```

**预期 FPS**：~45

---

## 问题 9：检查当前使用的实现

**如何确认是否使用了 Eigen3**：

```bash
# 查看编译输出
cd build
cmake ..

# 应该看到以下之一：
# "Found Eigen3: x.x.x" -> 使用 Eigen3
# "Eigen3 not found, using OpenCV" -> 使用 OpenCV
```

**运行时检查**（可选）：
在 `src/box_tracker.cc` 中添加：
```cpp
#ifdef USE_EIGEN3
  std::cout << "Using Eigen3 implementation" << std::endl;
#else
  std::cout << "Using OpenCV implementation" << std::endl;
#endif
```

---

## 问题 10：如何回退到原始实现

如果新功能有问题，可以快速回退：

```cpp
tracking::TrackerConfig config;

// 禁用所有阶段3功能
config.motion_model = tracking::TrackerConfig::MotionModel::TRANSLATION;

// 禁用阶段2功能
config.adaptive_spring_force = false;
config.use_spatial_prior = false;

// 恢复原始参数
config.spring_force = 0.1f;
config.fb_verify_threshold = 2.0f;
config.max_track_length = 1000;  // 实际上不限制
```

或者直接 checkout 原始分支：
```bash
git checkout main
```

---

## 获取帮助

如果遇到未列出的问题：

1. 查看相关文档：
   - [`ANALYSIS_CN.md`](ANALYSIS_CN.md) - 原理分析
   - [`IMPROVEMENT_PLAN_CN.md`](IMPROVEMENT_PLAN_CN.md) - 改进方案
   - [`STAGE3_USAGE_CN.md`](STAGE3_USAGE_CN.md) - 使用说明

2. 检查 Pull Requests：
   - PR #3: https://github.com/MySpaceWork/box_tracking_demo/pull/3
   - PR #4: https://github.com/MySpaceWork/box_tracking_demo/pull/4

3. 参考 MediaPipe 源码：
   - `mediapipe/util/tracking/tracking.cc`
   - `mediapipe/util/tracking/motion_estimation.cc`

---

最后更新：2026-03-19  
版本：阶段3改进 + 修复
