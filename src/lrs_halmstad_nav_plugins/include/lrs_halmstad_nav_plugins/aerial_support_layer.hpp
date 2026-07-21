#ifndef LRS_HALMSTAD_NAV_PLUGINS__AERIAL_SUPPORT_LAYER_HPP_
#define LRS_HALMSTAD_NAV_PLUGINS__AERIAL_SUPPORT_LAYER_HPP_

#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "lrs_halmstad_interfaces/msg/aerial_hazard.hpp"
#include "lrs_halmstad_interfaces/msg/aerial_hazard_array.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lrs_halmstad_nav_plugins
{

class AerialSupportLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  AerialSupportLayer() = default;
  ~AerialSupportLayer() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return true;}
  void matchSize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

protected:
  using HazardArray = lrs_halmstad_interfaces::msg::AerialHazardArray;

  virtual int64_t nowNanoseconds() const;
  void incomingHazards(HazardArray::ConstSharedPtr message);

private:
  struct Bounds
  {
    bool valid{false};
    double min_x{std::numeric_limits<double>::max()};
    double min_y{std::numeric_limits<double>::max()};
    double max_x{std::numeric_limits<double>::lowest()};
    double max_y{std::numeric_limits<double>::lowest()};

    void include(double x0, double y0, double x1, double y1);
    void clear();
  };

  struct Track
  {
    double center_x{0.0};
    double center_y{0.0};
    double yaw{0.0};
    double half_size_x{0.0};
    double half_size_y{0.0};
    int64_t valid_until_ns{0};
    unsigned char cost{nav2_costmap_2d::NO_INFORMATION};
  };

  using TrackStore = std::unordered_map<std::string, Track>;

  void declareAndLoadParameters();
  void validateParameters() const;
  void createSubscription();
  void clearTracksLocked();
  void rebuildLayerLocked();
  void applyPendingSnapshotLocked(int64_t now_ns);
  void expireTracksLocked(int64_t now_ns);
  void includeTrackBoundsLocked(const Track & track);
  void includeAllTrackBoundsLocked(const TrackStore & tracks);
  void rasterizeTrackLocked(const Track & track);

  bool buildTrack(
    const lrs_halmstad_interfaces::msg::AerialHazard & hazard,
    const std::string & array_frame,
    int64_t now_ns,
    Track & track) const;
  bool validateArrayHeader(const HazardArray & message, int64_t now_ns) const;
  unsigned char stateCost(uint8_t state) const;
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  mutable std::mutex mutex_;
  TrackStore tracks_;
  TrackStore pending_tracks_;
  bool pending_snapshot_{false};
  Bounds dirty_bounds_;
  bool active_{false};

  std::string topic_;
  std::string target_frame_;
  double transform_tolerance_s_{0.30};
  double max_observation_age_s_{1.00};
  double default_ttl_s_{2.00};
  double min_confidence_{0.35};
  double max_xy_variance_m2_{1.00};
  int confirmed_cost_{254};
  int tentative_cost_{200};
  int conflict_cost_{220};
  double covariance_sigma_scale_{2.0};
  double min_footprint_size_m_{0.30};
  int subscription_depth_{10};

  rclcpp::Subscription<HazardArray>::SharedPtr subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace lrs_halmstad_nav_plugins

#endif  // LRS_HALMSTAD_NAV_PLUGINS__AERIAL_SUPPORT_LAYER_HPP_
