#include "lrs_halmstad_nav_plugins/aerial_support_layer.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace lrs_halmstad_nav_plugins
{
namespace
{

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

bool finite(double value)
{
  return std::isfinite(value);
}

bool nonemptyId(const std::string & value)
{
  return std::any_of(value.begin(), value.end(), [](unsigned char character) {
             return !std::isspace(character);
  });
}

bool stampNanoseconds(const builtin_interfaces::msg::Time & stamp, int64_t & value)
{
  if (stamp.sec < 0 || stamp.nanosec >= static_cast<uint32_t>(kNanosecondsPerSecond)) {
    return false;
  }
  value = static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond + stamp.nanosec;
  return value > 0;
}

bool durationNanoseconds(const builtin_interfaces::msg::Duration & duration, int64_t & value)
{
  if (duration.sec < 0 || duration.nanosec >= static_cast<uint32_t>(kNanosecondsPerSecond)) {
    return false;
  }
  value = static_cast<int64_t>(duration.sec) * kNanosecondsPerSecond + duration.nanosec;
  return true;
}

int64_t saturatingAdd(int64_t lhs, int64_t rhs)
{
  if (rhs > 0 && lhs > std::numeric_limits<int64_t>::max() - rhs) {
    return std::numeric_limits<int64_t>::max();
  }
  return lhs + rhs;
}

}  // namespace

void AerialSupportLayer::Bounds::include(double x0, double y0, double x1, double y1)
{
  valid = true;
  min_x = std::min(min_x, x0);
  min_y = std::min(min_y, y0);
  max_x = std::max(max_x, x1);
  max_y = std::max(max_y, y1);
}

void AerialSupportLayer::Bounds::clear()
{
  valid = false;
  min_x = std::numeric_limits<double>::max();
  min_y = std::numeric_limits<double>::max();
  max_x = std::numeric_limits<double>::lowest();
  max_y = std::numeric_limits<double>::lowest();
}

void AerialSupportLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("AerialSupportLayer could not lock its lifecycle node");
  }

  logger_ = node->get_logger();
  clock_ = node->get_clock();
  declareAndLoadParameters();
  validateParameters();
  matchSize();

  parameter_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&AerialSupportLayer::dynamicParametersCallback, this, std::placeholders::_1));
  current_ = true;

  RCLCPP_INFO(
    logger_,
    "%s initialized: enabled=%s topic=%s target_frame=%s",
    name_.c_str(), enabled_ ? "true" : "false", topic_.c_str(), target_frame_.c_str());
}

void AerialSupportLayer::declareAndLoadParameters()
{
  declareParameter("enabled", rclcpp::ParameterValue(false));
  declareParameter("topic", rclcpp::ParameterValue("/coord/ugv/aerial_hazards"));
  declareParameter("target_frame", rclcpp::ParameterValue("map"));
  declareParameter("transform_tolerance_s", rclcpp::ParameterValue(0.30));
  declareParameter("max_observation_age_s", rclcpp::ParameterValue(1.00));
  declareParameter("default_ttl_s", rclcpp::ParameterValue(2.00));
  declareParameter("min_confidence", rclcpp::ParameterValue(0.35));
  declareParameter("max_xy_variance_m2", rclcpp::ParameterValue(1.00));
  declareParameter("confirmed_cost", rclcpp::ParameterValue(254));
  declareParameter("tentative_cost", rclcpp::ParameterValue(200));
  declareParameter("conflict_cost", rclcpp::ParameterValue(220));
  declareParameter("covariance_sigma_scale", rclcpp::ParameterValue(2.0));
  declareParameter("min_footprint_size_m", rclcpp::ParameterValue(0.30));
  declareParameter("subscription_depth", rclcpp::ParameterValue(10));

  auto node = node_.lock();
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("topic"), topic_);
  node->get_parameter(getFullName("target_frame"), target_frame_);
  node->get_parameter(getFullName("transform_tolerance_s"), transform_tolerance_s_);
  node->get_parameter(getFullName("max_observation_age_s"), max_observation_age_s_);
  node->get_parameter(getFullName("default_ttl_s"), default_ttl_s_);
  node->get_parameter(getFullName("min_confidence"), min_confidence_);
  node->get_parameter(getFullName("max_xy_variance_m2"), max_xy_variance_m2_);
  node->get_parameter(getFullName("confirmed_cost"), confirmed_cost_);
  node->get_parameter(getFullName("tentative_cost"), tentative_cost_);
  node->get_parameter(getFullName("conflict_cost"), conflict_cost_);
  node->get_parameter(getFullName("covariance_sigma_scale"), covariance_sigma_scale_);
  node->get_parameter(getFullName("min_footprint_size_m"), min_footprint_size_m_);
  node->get_parameter(getFullName("subscription_depth"), subscription_depth_);
}

void AerialSupportLayer::validateParameters() const
{
  if (topic_.empty() || topic_.front() != '/') {
    throw std::invalid_argument("AerialSupportLayer topic must be absolute");
  }
  if (target_frame_.empty() || target_frame_ != layered_costmap_->getGlobalFrameID()) {
    throw std::invalid_argument(
        "AerialSupportLayer target_frame must match the global costmap frame");
  }
  if (!finite(transform_tolerance_s_) || transform_tolerance_s_ < 0.0 ||
    !finite(max_observation_age_s_) || max_observation_age_s_ <= 0.0 ||
    !finite(default_ttl_s_) || default_ttl_s_ <= 0.0 ||
    !finite(min_confidence_) || min_confidence_ < 0.0 || min_confidence_ > 1.0 ||
    !finite(max_xy_variance_m2_) || max_xy_variance_m2_ <= 0.0 ||
    !finite(covariance_sigma_scale_) || covariance_sigma_scale_ < 0.0 ||
    !finite(min_footprint_size_m_) || min_footprint_size_m_ <= 0.0)
  {
    throw std::invalid_argument("AerialSupportLayer floating-point parameters are invalid");
  }
  if (confirmed_cost_ != nav2_costmap_2d::LETHAL_OBSTACLE) {
    throw std::invalid_argument("AerialSupportLayer confirmed_cost must be lethal (254)");
  }
  if (tentative_cost_ <= nav2_costmap_2d::FREE_SPACE ||
    tentative_cost_ >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
    conflict_cost_ <= nav2_costmap_2d::FREE_SPACE ||
    conflict_cost_ >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    throw std::invalid_argument(
        "AerialSupportLayer tentative/conflict costs must be high non-lethal costs");
  }
  if (subscription_depth_ <= 0) {
    throw std::invalid_argument("AerialSupportLayer subscription_depth must be positive");
  }
}

void AerialSupportLayer::matchSize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  nav2_costmap_2d::CostmapLayer::matchSize();
  setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
  if (getCharMap() != nullptr) {
    std::fill(
      getCharMap(), getCharMap() + getSizeInCellsX() * getSizeInCellsY(),
      nav2_costmap_2d::NO_INFORMATION);
  }
  tracks_.clear();
  pending_tracks_.clear();
  pending_snapshot_ = false;
  dirty_bounds_.clear();
}

void AerialSupportLayer::activate()
{
  bool should_subscribe = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = true;
    should_subscribe = enabled_;
  }
  if (should_subscribe) {
    createSubscription();
  }
}

void AerialSupportLayer::deactivate()
{
  subscription_.reset();
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  clearTracksLocked();
  current_ = false;
}

void AerialSupportLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  clearTracksLocked();
  current_ = false;
}

void AerialSupportLayer::createSubscription()
{
  if (subscription_) {
    return;
  }
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("AerialSupportLayer could not create its subscription");
  }
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<HazardArray>(
    topic_, rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(subscription_depth_))).reliable(),
    std::bind(&AerialSupportLayer::incomingHazards, this, std::placeholders::_1), options);
}

int64_t AerialSupportLayer::nowNanoseconds() const
{
  return clock_->now().nanoseconds();
}

bool AerialSupportLayer::validateArrayHeader(const HazardArray & message, int64_t now_ns) const
{
  int64_t stamp_ns = 0;
  if (message.header.frame_id.empty() || !stampNanoseconds(message.header.stamp, stamp_ns)) {
    return false;
  }
  const int64_t max_age_ns = static_cast<int64_t>(max_observation_age_s_ * kNanosecondsPerSecond);
  return stamp_ns <= now_ns && now_ns - stamp_ns <= max_age_ns;
}

void AerialSupportLayer::incomingHazards(HazardArray::ConstSharedPtr message)
{
  const int64_t now_ns = nowNanoseconds();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || !enabled_) {
      return;
    }
  }

  if (!validateArrayHeader(*message, now_ns)) {
    return;
  }

  TrackStore accepted;
  accepted.reserve(message->hazards.size());
  for (const auto & hazard : message->hazards) {
    Track track;
    if (buildTrack(hazard, message->header.frame_id, now_ns, track)) {
      accepted[hazard.detection.id] = track;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !enabled_) {
    return;
  }
  pending_tracks_ = std::move(accepted);
  pending_snapshot_ = true;
  current_ = false;
}

bool AerialSupportLayer::buildTrack(
  const lrs_halmstad_interfaces::msg::AerialHazard & hazard,
  const std::string & array_frame,
  int64_t now_ns,
  Track & track) const
{
  const auto & detection = hazard.detection;
  if (!nonemptyId(detection.id) || detection.header.frame_id.empty() ||
    detection.header.frame_id != array_frame)
  {
    return false;
  }

  int64_t detection_stamp_ns = 0;
  int64_t first_seen_ns = 0;
  int64_t last_seen_ns = 0;
  int64_t ttl_ns = 0;
  if (!stampNanoseconds(detection.header.stamp, detection_stamp_ns) ||
    !stampNanoseconds(hazard.first_seen, first_seen_ns) ||
    !stampNanoseconds(hazard.last_seen, last_seen_ns) ||
    !durationNanoseconds(hazard.ttl, ttl_ns) || first_seen_ns > last_seen_ns)
  {
    return false;
  }

  const int64_t max_age_ns = static_cast<int64_t>(max_observation_age_s_ * kNanosecondsPerSecond);
  if (detection_stamp_ns > now_ns || last_seen_ns > now_ns ||
    now_ns - detection_stamp_ns > max_age_ns || now_ns - last_seen_ns > max_age_ns)
  {
    return false;
  }
  if (ttl_ns == 0) {
    ttl_ns = static_cast<int64_t>(default_ttl_s_ * kNanosecondsPerSecond);
  }
  const int64_t ttl_deadline = saturatingAdd(last_seen_ns, ttl_ns);
  const int64_t age_deadline = saturatingAdd(detection_stamp_ns, max_age_ns);
  track.valid_until_ns = std::min(ttl_deadline, age_deadline);
  if (now_ns > track.valid_until_ns) {
    return false;
  }

  if (!finite(detection.bbox.size.x) || detection.bbox.size.x <= 0.0 ||
    !finite(detection.bbox.size.y) || detection.bbox.size.y <= 0.0 ||
    !finite(detection.bbox.size.z) || detection.bbox.size.z <= 0.0 ||
    !finite(detection.bbox.center.position.x) ||
    !finite(detection.bbox.center.position.y) ||
    !finite(detection.bbox.center.position.z) || detection.results.empty())
  {
    return false;
  }

  const auto & result = detection.results.front();
  const double confidence = result.hypothesis.score;
  if (!finite(confidence) || confidence < min_confidence_ || confidence > 1.0 ||
    !finite(hazard.support_quality) || hazard.support_quality < 0.0 ||
    hazard.support_quality > 1.0)
  {
    return false;
  }

  bool covariance_nonzero = false;
  for (double value : result.pose.covariance) {
    if (!finite(value)) {
      return false;
    }
    covariance_nonzero = covariance_nonzero || value != 0.0;
  }
  for (size_t index : std::array<size_t, 6>{0, 7, 14, 21, 28, 35}) {
    if (result.pose.covariance[index] < 0.0) {
      return false;
    }
  }
  const double xy_variance = std::max(result.pose.covariance[0], result.pose.covariance[7]);
  if (!covariance_nonzero || xy_variance > max_xy_variance_m2_) {
    return false;
  }

  track.cost = stateCost(hazard.state);
  if (track.cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }

  const auto & orientation = detection.bbox.center.orientation;
  if (!finite(orientation.x) || !finite(orientation.y) || !finite(orientation.z) ||
    !finite(orientation.w))
  {
    return false;
  }
  const double quaternion_norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (!finite(quaternion_norm) || quaternion_norm <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  geometry_msgs::msg::PoseStamped source_pose;
  source_pose.header = detection.header;
  source_pose.pose = detection.bbox.center;
  source_pose.pose.orientation.x /= quaternion_norm;
  source_pose.pose.orientation.y /= quaternion_norm;
  source_pose.pose.orientation.z /= quaternion_norm;
  source_pose.pose.orientation.w /= quaternion_norm;

  geometry_msgs::msg::PoseStamped target_pose;
  if (detection.header.frame_id == target_frame_) {
    target_pose = source_pose;
  } else {
    if (tf_ == nullptr) {
      return false;
    }
    try {
      const auto transform = tf_->lookupTransform(
        target_frame_, detection.header.frame_id,
        rclcpp::Time(detection.header.stamp, clock_->get_clock_type()),
        rclcpp::Duration::from_seconds(transform_tolerance_s_));
      tf2::doTransform(source_pose, target_pose, transform);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  if (!finite(target_pose.pose.position.x) || !finite(target_pose.pose.position.y)) {
    return false;
  }
  track.center_x = target_pose.pose.position.x;
  track.center_y = target_pose.pose.position.y;
  track.yaw = tf2::getYaw(target_pose.pose.orientation);
  if (!finite(track.yaw)) {
    return false;
  }

  const double uncertainty = covariance_sigma_scale_ * std::sqrt(xy_variance);
  track.half_size_x = 0.5 * std::max(detection.bbox.size.x, min_footprint_size_m_) + uncertainty;
  track.half_size_y = 0.5 * std::max(detection.bbox.size.y, min_footprint_size_m_) + uncertainty;
  return true;
}

unsigned char AerialSupportLayer::stateCost(uint8_t state) const
{
  using Hazard = lrs_halmstad_interfaces::msg::AerialHazard;
  if (state == Hazard::CONFIRMED) {
    return static_cast<unsigned char>(confirmed_cost_);
  }
  if (state == Hazard::TENTATIVE) {
    return static_cast<unsigned char>(tentative_cost_);
  }
  if (state == Hazard::CONFLICT) {
    return static_cast<unsigned char>(conflict_cost_);
  }
  return nav2_costmap_2d::NO_INFORMATION;
}

void AerialSupportLayer::includeTrackBoundsLocked(const Track & track)
{
  const double cosine = std::abs(std::cos(track.yaw));
  const double sine = std::abs(std::sin(track.yaw));
  const double extent_x = cosine * track.half_size_x + sine * track.half_size_y;
  const double extent_y = sine * track.half_size_x + cosine * track.half_size_y;
  const double cell_margin = 0.5 * getResolution();
  dirty_bounds_.include(
    track.center_x - extent_x - cell_margin, track.center_y - extent_y - cell_margin,
    track.center_x + extent_x + cell_margin, track.center_y + extent_y + cell_margin);
}

void AerialSupportLayer::includeAllTrackBoundsLocked(const TrackStore & tracks)
{
  for (const auto & item : tracks) {
    includeTrackBoundsLocked(item.second);
  }
}

void AerialSupportLayer::rebuildLayerLocked()
{
  if (getCharMap() == nullptr) {
    return;
  }
  std::fill(
    getCharMap(), getCharMap() + getSizeInCellsX() * getSizeInCellsY(),
    nav2_costmap_2d::NO_INFORMATION);
  for (const auto & item : tracks_) {
    rasterizeTrackLocked(item.second);
  }
}

void AerialSupportLayer::rasterizeTrackLocked(const Track & track)
{
  const double cosine = std::cos(track.yaw);
  const double sine = std::sin(track.yaw);
  const double extent_x = std::abs(cosine) * track.half_size_x +
    std::abs(sine) * track.half_size_y;
  const double extent_y = std::abs(sine) * track.half_size_x +
    std::abs(cosine) * track.half_size_y;

  if (track.center_x + extent_x < getOriginX() ||
    track.center_y + extent_y < getOriginY() ||
    track.center_x - extent_x > getOriginX() + getSizeInCellsX() * getResolution() ||
    track.center_y - extent_y > getOriginY() + getSizeInCellsY() * getResolution())
  {
    return;
  }

  int min_mx = 0;
  int min_my = 0;
  int max_mx = 0;
  int max_my = 0;
  worldToMapEnforceBounds(track.center_x - extent_x, track.center_y - extent_y, min_mx, min_my);
  worldToMapEnforceBounds(track.center_x + extent_x, track.center_y + extent_y, max_mx, max_my);

  const double cell_margin = 0.5 * getResolution();
  for (int my = min_my; my <= max_my; ++my) {
    for (int mx = min_mx; mx <= max_mx; ++mx) {
      double world_x = 0.0;
      double world_y = 0.0;
      mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), world_x, world_y);
      const double dx = world_x - track.center_x;
      const double dy = world_y - track.center_y;
      const double local_x = cosine * dx + sine * dy;
      const double local_y = -sine * dx + cosine * dy;
      if (std::abs(local_x) <= track.half_size_x + cell_margin &&
        std::abs(local_y) <= track.half_size_y + cell_margin)
      {
        const unsigned char old_cost = getCost(mx, my);
        setCost(mx, my, old_cost == nav2_costmap_2d::NO_INFORMATION ?
          track.cost : std::max(old_cost, track.cost));
      }
    }
  }
}

void AerialSupportLayer::applyPendingSnapshotLocked(int64_t now_ns)
{
  if (!pending_snapshot_) {
    return;
  }
  includeAllTrackBoundsLocked(tracks_);
  tracks_ = std::move(pending_tracks_);
  pending_tracks_.clear();
  pending_snapshot_ = false;
  for (auto iterator = tracks_.begin(); iterator != tracks_.end(); ) {
    if (now_ns > iterator->second.valid_until_ns) {
      iterator = tracks_.erase(iterator);
    } else {
      ++iterator;
    }
  }
  includeAllTrackBoundsLocked(tracks_);
  rebuildLayerLocked();
}

void AerialSupportLayer::expireTracksLocked(int64_t now_ns)
{
  bool changed = false;
  for (auto iterator = tracks_.begin(); iterator != tracks_.end(); ) {
    if (now_ns > iterator->second.valid_until_ns) {
      includeTrackBoundsLocked(iterator->second);
      iterator = tracks_.erase(iterator);
      changed = true;
    } else {
      ++iterator;
    }
  }
  if (changed) {
    rebuildLayerLocked();
  }
}

void AerialSupportLayer::clearTracksLocked()
{
  includeAllTrackBoundsLocked(tracks_);
  tracks_.clear();
  pending_tracks_.clear();
  pending_snapshot_ = false;
  rebuildLayerLocked();
}

void AerialSupportLayer::updateBounds(
  double, double, double,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const int64_t now_ns = nowNanoseconds();
  if (!enabled_) {
    if (!tracks_.empty() || pending_snapshot_) {
      clearTracksLocked();
    }
  } else {
    applyPendingSnapshotLocked(now_ns);
    expireTracksLocked(now_ns);
  }

  if (dirty_bounds_.valid) {
    touch(dirty_bounds_.min_x, dirty_bounds_.min_y, min_x, min_y, max_x, max_y);
    touch(dirty_bounds_.max_x, dirty_bounds_.max_y, min_x, min_y, max_x, max_y);
  }
  current_ = true;
}

void AerialSupportLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_) {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  dirty_bounds_.clear();
}

rcl_interfaces::msg::SetParametersResult AerialSupportLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    if (parameter.get_name() != getFullName("enabled") &&
      parameter.get_name().rfind(name_ + ".", 0) == 0)
    {
      result.successful = false;
      result.reason = "AerialSupportLayer parameters other than enabled require a costmap restart";
      return result;
    }
  }

  const auto enabled_parameter = std::find_if(
    parameters.begin(), parameters.end(), [this](const rclcpp::Parameter & parameter) {
      return parameter.get_name() == getFullName("enabled");
    });
  if (enabled_parameter == parameters.end()) {
    return result;
  }

  const bool requested_enabled = enabled_parameter->as_bool();
  bool should_subscribe = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled_ && !requested_enabled) {
      clearTracksLocked();
    }
    enabled_ = requested_enabled;
    should_subscribe = active_ && enabled_;
  }
  if (should_subscribe) {
    createSubscription();
  } else {
    subscription_.reset();
  }
  return result;
}

}  // namespace lrs_halmstad_nav_plugins

PLUGINLIB_EXPORT_CLASS(
  lrs_halmstad_nav_plugins::AerialSupportLayer,
  nav2_costmap_2d::Layer)
