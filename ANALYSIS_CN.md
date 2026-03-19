# box_tracking_demo 与 MediaPipe 目标跟踪对比分析

## 概述

[`box_tracking_demo`](box_tracking_demo) 是一个简化的C++目标跟踪实现，它复现了 MediaPipe 跟踪模块的核心算法，但仅依赖 OpenCV。本文档分析两者的区别，并重点讨论**单目标跟踪时的偏移问题**及 MediaPipe 的解决方案。

---

## 一、算法流程对比

### box_tracking_demo 的实现

box_tracking_demo 实现了以下8个步骤：

1. **特征提取**：使用 `cv::goodFeaturesToTrack` 在均匀网格上提取 Harris/Shi-Tomasi 角点
2. **光流跟踪**：使用金字塔 Lucas-Kanade (KLT) 光流 `cv::calcOpticalFlowPyrLK`
3. **前后验证**：前向跟踪后再反向跟踪，丢弃往返误差大的特征点
4. **RANSAC 离群值剔除**：移除不符合主导运动的流向量
5. **相机运动估计**：通过 RANSAC `cv::findHomography` 估计全局单应矩阵
6. **运动分解**：将每个特征的运动分解为背景（相机）和前景（物体）分量
7. **IRLS 物体运动估计**：迭代重加权最小二乘法估计物体平移，每次迭代降低离群值权重
8. **内点评分 + 弹簧修正**：基于内点比例计算置信度；将box中心拉向内点质心以防止漂移

### MediaPipe 的完整实现

MediaPipe 在 [`mediapipe/util/tracking/`](mediapipe/mediapipe/util/tracking/) 模块中提供了更完整的实现：

| MediaPipe 组件 | box_tracking_demo 对应 |
|---|---|
| [`RegionFlowComputation`](mediapipe/mediapipe/util/tracking/region_flow_computation.h) | [`FlowComputation`](box_tracking_demo/include/box_tracker.h:107) 类 |
| [`MotionEstimation`](mediapipe/mediapipe/util/tracking/motion_estimation.h) | [`CameraMotionEstimator`](box_tracking_demo/include/box_tracker.h:130) 类 |
| [`MotionBox::TrackStep`](mediapipe/mediapipe/util/tracking/tracking.h:302) | [`MotionBoxTracker::TrackStep`](box_tracking_demo/include/box_tracker.h:153) |
| [`MotionBox::TranslationIrlsInitialization`](mediapipe/mediapipe/util/tracking/tracking.cc:1837) | [`MotionBoxTracker::RansacTranslationInit`](box_tracking_demo/src/box_tracker.cc:313) |
| [`MotionBox::EstimateTranslation`](mediapipe/mediapipe/util/tracking/tracking.cc:2007) | [`MotionBoxTracker::EstimateTranslation`](box_tracking_demo/src/box_tracker.cc:353) |
| [`BoxTracker`](mediapipe/mediapipe/util/tracking/box_tracker.h:162) | [`BoxTracker`](box_tracking_demo/include/box_tracker.h:200) (高层封装) |

---

## 二、关键差异

### 1. 运动模型复杂度

**box_tracking_demo**:
- **仅支持平移**（translation-only）
- 代码见 [`box_tracker.cc:465-468`](box_tracking_demo/src/box_tracker.cc:465)
```cpp
// Step 4: Apply motion to box.
next_state.x += translation.x;
next_state.y += translation.y;
```

**MediaPipe**:
- 支持多种运动模型：平移（Translation）、相似变换（Similarity）、仿射（Affine）、单应（Homography）、透视变换（PnP Homography）
- 通过 [`TrackStepOptions::tracking_degrees`](mediapipe/mediapipe/util/tracking/tracking.cc:514) 配置
- 代码见 [`tracking.cc:525-568`](mediapipe/mediapipe/util/tracking/tracking.cc:525)

### 2. 跟踪方向

**box_tracking_demo**:
- **单向前向跟踪**（forward-only）

**MediaPipe**:
- **双向跟踪**（bidirectional tracking）
- 支持前向和后向跟踪，并通过交叉验证提高鲁棒性
- 代码见 [`MotionBox::TrackStep`](mediapipe/mediapipe/util/tracking/tracking.cc:1003)

### 3. 数据序列化与缓存

**box_tracking_demo**:
- 无 protobuf 序列化
- 所有数据在内存中处理

**MediaPipe**:
- 使用 protobuf 定义状态：[`box_tracker.proto`](mediapipe/mediapipe/util/tracking/box_tracker.proto)
- 支持分块（chunk）缓存和并行处理
- 可以序列化跟踪状态用于离线分析

### 4. 重检测与恢复

**box_tracking_demo**:
- **无重检测**机制
- 一旦跟踪失败（`confidence < 0.1`），无法自动恢复

**MediaPipe**:
- 支持 box re-detection 和 re-acquisition
- 代码见 [`box_detector.h`](mediapipe/mediapipe/util/tracking/box_detector.h)
- 可以在跟踪失败后自动重新初始化

---

## 三、单目标跟踪偏移问题分析

### 问题描述

在运行 demo 时，单目标跟踪会出现**明显偏移**（drift）。这是因为：

1. **累积误差**：每帧的小误差会随时间累积
2. **特征点分布不均**：局部特征聚集可能导致运动估计偏向某一方向
3. **遮挡与光照变化**：导致跟踪特征点发生变化
4. **纯平移模型的局限性**：无法处理旋转和缩放

### MediaPipe 的解决方案

MediaPipe 通过以下多层机制解决漂移问题：

#### 1. 弹簧力修正（Spring Force Correction）

**原理**：将 box 中心向内点质心（inlier center）拉回。

**实现**：
- 代码位置：[`tracking.cc:2549-2576`](mediapipe/mediapipe/util/tracking/tracking.cc:2549)
- 函数：[`MotionBox::ApplySpringForce`](mediapipe/mediapipe/util/tracking/tracking.cc:2549)

```cpp
void MotionBox::ApplySpringForce(const Vector2_f& center_of_interest,
                                 const float rel_threshold,
                                 const float spring_force,
                                 MotionBoxState* box_state) const {
  // Apply spring force towards center of interest.
  const Vector2_f center = MotionBoxCenter(*box_state);
  const float center_diff_x = center_of_interest.x() - center.x();
  const float center_diff_y = center_of_interest.y() - center.y();
  
  // 仅在差异超过阈值时应用修正
  if (diff_x > 0) {
    const float correction_mag = diff_x * spring_force;
    const float correction = correction_mag * (center_diff_x > 0 ? 1.0f : -1.0f);
    box_state->set_pos_x(box_state->pos_x() + correction);
  }
  // 对 y 方向同样处理...
}
```

**在 box_tracking_demo 中的对应实现**：
- 代码位置：[`box_tracker.cc:421-428`](box_tracking_demo/src/box_tracker.cc:421)
```cpp
// Apply spring force toward inlier center.
cv::Point2f box_center = next_state.center();
cv::Point2f diff = inlier_center - box_center;
float diff_mag = cv::norm(diff);
if (diff_mag > 0.01f) {
  next_state.x += diff.x * config_.spring_force;
  next_state.y += diff.y * config_.spring_force;
}
```

**关键参数**：
- `spring_force`：修正力度，默认值 [`0.1`](box_tracking_demo/include/box_tracker.h:99)
- `inlier_center_relative_distance`：相对距离阈值

#### 2. 内点中心跟踪（Inlier Center Tracking）

**原理**：记录并跟踪内点的质心和范围，作为物体真实位置的估计。

**实现**：
- 代码位置：[`tracking.cc:2432-2496`](mediapipe/mediapipe/util/tracking/tracking.cc:2432)
- 函数：[`MotionBox::ComputeInlierCenterAndExtent`](mediapipe/mediapipe/util/tracking/tracking.cc:2432)

```cpp
void MotionBox::ComputeInlierCenterAndExtent(
    const std::vector<const MotionVector*>& motion_vectors,
    const std::vector<float>& weights, const std::vector<float>& density,
    const MotionBoxState& box_state, float* min_inlier_sum, Vector2_f* center,
    Vector2_f* extent) const {
  
  // 计算加权内点中心
  Vector2_f inlier_sum(0, 0);
  float weight_sum = 0;
  for (size_t i = 0; i < motion_vectors.size(); ++i) {
    if (weights[i] > threshold) {
      inlier_sum += motion_vectors[i]->pos * weights[i];
      weight_sum += weights[i];
    }
  }
  
  if (weight_sum > *min_inlier_sum) {
    *center = inlier_sum / weight_sum;
    // 同时计算内点范围...
  }
}
```

**状态更新**：
- 代码位置：[`tracking.cc:3094-3095`](mediapipe/mediapipe/util/tracking/tracking.cc:3094)
```cpp
next_pos->set_inlier_center_x(inlier_center.x());
next_pos->set_inlier_center_y(inlier_center.y());
```

#### 3. 时间平滑与混合（Temporal Smoothing）

**原理**：将当前帧的内点中心与历史内点中心混合，避免突变。

**实现**：
- 代码位置：[`tracking.cc:3078-3092`](mediapipe/mediapipe/util/tracking/tracking.cc:3078)

```cpp
// 计算内点中心相对于前一帧的变化
Vector2_f prev_inlier_center(InlierCenter(curr_pos));
const float rel_inlier_center_diff =
    (inlier_center - prev_inlier_center).Norm() /
    MotionBoxSize(curr_pos).Norm();

// 根据背景区分度和变化幅度计算混合权重
const float center_blend =
    std::min(Lerp(0.95f, 0.6f, background_discrimination),
             rel_inlier_center_diff) *
    curr_pos.prior_weight();

// 混合当前和历史内点中心
inlier_center = Lerp(inlier_center, prev_inlier_center, center_blend);
```

**说明**：
- `background_discrimination`：背景运动区分度，越高说明物体运动与背景区分越明显
- `prior_weight`：先验权重，基于跟踪历史的置信度
- 当背景区分度高时，更多地采用当前估计；否则更多地依赖历史

#### 4. 空间先验（Spatial Prior）

**原理**：在网格上记录内点的空间分布，用于下一帧的权重初始化。

**实现**：
- 代码位置：[`tracking.cc:1145-1280`](mediapipe/mediapipe/util/tracking/tracking.cc:1145)
- 函数：`ComputeSpatialPrior`

这允许跟踪器"记住"物体的局部结构，即使某些区域暂时丢失特征点。

#### 5. IRLS 时间平滑（Temporal IRLS Smoothing）

**原理**：对多帧的 IRLS 权重进行时空双边滤波，减少单帧噪声影响。

**实现**：
- 代码位置：[`motion_estimation.cc:5868-5990`](mediapipe/mediapipe/util/tracking/motion_estimation.cc:5868)
- 函数：`MotionEstimation::RunTemporalIRLSSmoothing`

```cpp
void MotionEstimation::RunTemporalIRLSSmoothing(
    const std::vector<FeatureGrid<RegionFlowFeature>>& feature_grid,
    const std::vector<std::vector<int>>& feature_taps_3,
    const std::vector<CameraMotion>& camera_motions,
    const std::vector<float>& frame_confidence,
    std::vector<RegionFlowFeatureView>* feature_views) const {
  
  // Push: 从前一帧向后传播权重
  // Pull: 从后一帧向前传播权重
  // 结合空间邻域信息和特征描述符相似度
}
```

这在 [`PostIRLSSmoothing`](mediapipe/mediapipe/util/tracking/motion_estimation.cc:5588) 中被调用。

#### 6. 长特征跟踪与偏置（Long Feature Tracking）

**原理**：对于长期跟踪的特征点，使用其历史 IRLS 权重进行偏置，增强跟踪稳定性。

**实现**：
- 代码位置：[`motion_estimation.cc:1927-2090`](mediapipe/mediapipe/util/tracking/motion_estimation.cc:1927)
- 函数：`MotionEstimation::BiasLongFeatures`

**防止漂移**：
- 代码注释：[`region_flow_computation.cc:341`](mediapipe/mediapipe/util/tracking/region_flow_computation.cc:341)
```cpp
// Discard reason:
// (1) A tracked feature has too long track, which might create drift.
// (2) A tracked feature in a highly densed area, which provides little value.
```

MediaPipe 会主动丢弃过长的跟踪轨迹，防止累积漂移。

#### 7. 运动一致性加权（Motion Consistency Weighting）

**原理**：在 box_tracking_demo 中也有实现，但 MediaPipe 更复杂。

**box_tracking_demo 实现**：
- 代码位置：[`box_tracker.cc:291-300`](box_tracking_demo/src/box_tracker.cc:291)
```cpp
// Motion consistency weight (how close is this vector's motion to the
// box's previous velocity).
float motion_w = 1.0f;
if (std::abs(state_.dx) > 1e-6f || std::abs(state_.dy) > 1e-6f) {
  float mdx = mv.object.x - state_.dx;
  float mdy = mv.object.y - state_.dy;
  float motion_dist =
      std::sqrt(mdx * mdx + mdy * mdy) /
      std::max(0.001f,
               config_.motion_sigma *
                   std::sqrt(state_.dx * state_.dx + state_.dy * state_.dy));
  motion_w = std::exp(-0.5f * motion_dist * motion_dist);
}

weights.push_back(spatial_w * motion_w);
```

**MediaPipe 扩展**：
- 结合空间高斯权重、运动一致性、特征描述符相似度
- 动态调整权重参数

---

## 四、改进 box_tracking_demo 的建议

基于 MediaPipe 的经验，可以通过以下方式改进 box_tracking_demo 的偏移问题：

### 1. 调整弹簧力参数

修改 [`box_tracker.h:99`](box_tracking_demo/include/box_tracker.h:99):
```cpp
float spring_force = 0.1f;  // 增加到 0.15 或 0.2
```

**注意**：过大会导致抖动，需要根据场景调整。

### 2. 增强内点中心平滑

在 [`box_tracker.cc:421-428`](box_tracking_demo/src/box_tracker.cc:421) 添加时间混合：
```cpp
// 记录前一帧的内点中心
static cv::Point2f prev_inlier_center = inlier_center;

// 混合当前和历史
float blend_weight = 0.3f;  // 30% 使用历史
inlier_center = (1.0f - blend_weight) * inlier_center + 
                blend_weight * prev_inlier_center;

// 更新记录
prev_inlier_center = inlier_center;
```

### 3. 添加相似变换支持

扩展 [`MotionBoxTracker::EstimateTranslation`](box_tracking_demo/src/box_tracker.cc:353) 为 `EstimateSimilarity`，同时估计平移和缩放/旋转。

### 4. 实现长特征跟踪限制

在 [`FlowComputation`](box_tracking_demo/src/box_tracker.cc:55) 中添加：
```cpp
// 限制特征点最大跟踪帧数
const int max_track_length = 30;  // 最多跟踪 30 帧
if (feature.track_length > max_track_length) {
  // 标记为离群值或重新提取
}
```

### 5. 增加前后验证强度

修改 [`box_tracker.cc:106`](box_tracking_demo/src/box_tracker.cc:106):
```cpp
float fb_verify_threshold = 2.0f;  // 减小到 1.0 或 1.5
```

更严格的前后验证可以过滤更多不稳定的特征点。

---

## 五、总结

### box_tracking_demo 的优势

- ✅ **简洁**：仅 ~600 行代码
- ✅ **独立**：只依赖 OpenCV
- ✅ **教育性强**：易于理解核心算法

### MediaPipe 的优势

- ✅ **完整**：支持多种运动模型、双向跟踪、重检测
- ✅ **鲁棒**：多层防漂移机制
- ✅ **可扩展**：protobuf 序列化、并行处理

### 漂移问题的核心解决方案

MediaPipe 通过以下机制组合解决漂移：

1. **弹簧力修正**：实时拉回 box 中心 → **最直接有效**
2. **内点中心跟踪**：跟踪真实物体位置 → **核心机制**
3. **时间平滑**：混合历史信息 → **减少抖动**
4. **空间先验**：记忆物体结构 → **处理遮挡**
5. **IRLS 时空滤波**：多帧协同优化 → **提高稳定性**
6. **长特征限制**：主动丢弃可能漂移的轨迹 → **防止累积误差**

在 box_tracking_demo 中，**弹簧力修正 + 内点中心跟踪**已经实现，但参数可能需要针对具体场景调优。如果仍有明显偏移，建议：
- 增大 `spring_force`
- 减小 `fb_verify_threshold`
- 添加时间平滑
- 考虑使用相似变换而非纯平移

---

## 参考代码

### MediaPipe 核心文件

- [`tracking.h`](mediapipe/mediapipe/util/tracking/tracking.h) - MotionBox 类定义
- [`tracking.cc`](mediapipe/mediapipe/util/tracking/tracking.cc) - 跟踪实现（~3400 行）
- [`region_flow_computation.h/cc`](mediapipe/mediapipe/util/tracking/region_flow_computation.h) - 光流计算
- [`motion_estimation.h/cc`](mediapipe/mediapipe/util/tracking/motion_estimation.h) - 运动估计
- [`box_tracker.h/cc`](mediapipe/mediapipe/util/tracking/box_tracker.h) - 高层跟踪器

### box_tracking_demo 核心文件

- [`box_tracker.h`](box_tracking_demo/include/box_tracker.h) - 接口定义
- [`box_tracker.cc`](box_tracking_demo/src/box_tracker.cc) - 算法实现
- [`main.cc`](box_tracking_demo/src/main.cc) - Demo 应用

---

**作者注**：本分析基于 box_tracking_demo 的 README 和源码，以及 MediaPipe `mediapipe/util/tracking/` 模块的实现。
