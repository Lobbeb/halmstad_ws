#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lrs_halmstad_interfaces/msg/aerial_hazard.hpp"
#include "lrs_halmstad_interfaces/msg/aerial_hazard_array.hpp"
#include "lrs_halmstad_nav_plugins/aerial_support_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

namespace lrs_halmstad_nav_plugins
{
namespace
{

using Hazard = lrs_halmstad_interfaces::msg::AerialHazard;
using HazardArray = lrs_halmstad_interfaces::msg::AerialHazardArray;
constexpr int64_t kSecond = 1000000000LL;

builtin_interfaces::msg::Time timeMessage(int64_t nanoseconds)
{
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(nanoseconds / kSecond);
  result.nanosec = static_cast<uint32_t>(nanoseconds % kSecond);
  return result;
}

builtin_interfaces::msg::Duration durationMessage(double seconds)
{
  const int64_t nanoseconds = static_cast<int64_t>(seconds * kSecond);
  builtin_interfaces::msg::Duration result;
  result.sec = static_cast<int32_t>(nanoseconds / kSecond);
  result.nanosec = static_cast<uint32_t>(nanoseconds % kSecond);
  return result;
}

Hazard makeHazard(
  int64_t now_ns, const std::string & id = "hazard-1",
  uint8_t state = Hazard::CONFIRMED, double center_x = 0.0,
  double center_y = 0.0, double covariance = 0.01,
  double ttl_s = 2.0, double confidence = 0.9)
{
  Hazard hazard;
  hazard.detection.header.frame_id = "map";
  hazard.detection.header.stamp = timeMessage(now_ns);
  hazard.detection.id = id;
  hazard.detection.bbox.center.position.x = center_x;
  hazard.detection.bbox.center.position.y = center_y;
  hazard.detection.bbox.center.position.z = 0.5;
  hazard.detection.bbox.center.orientation.w = 1.0;
  hazard.detection.bbox.size.x = 0.6;
  hazard.detection.bbox.size.y = 0.4;
  hazard.detection.bbox.size.z = 1.0;

  vision_msgs::msg::ObjectHypothesisWithPose result;
  result.hypothesis.class_id = "hazard";
  result.hypothesis.score = confidence;
  result.pose.covariance[0] = covariance;
  result.pose.covariance[7] = covariance;
  result.pose.covariance[14] = covariance;
  result.pose.covariance[21] = 0.01;
  result.pose.covariance[28] = 0.01;
  result.pose.covariance[35] = 0.01;
  hazard.detection.results.push_back(result);

  hazard.source_uavs = {"dji1"};
  hazard.state = state;
  hazard.first_seen = timeMessage(now_ns - kSecond / 2);
  hazard.last_seen = timeMessage(now_ns);
  hazard.ttl = durationMessage(ttl_s);
  hazard.support_quality = 0.8F;
  hazard.provenance = "test";
  return hazard;
}

HazardArray makeArray(int64_t now_ns, std::vector<Hazard> hazards)
{
  HazardArray message;
  message.header.frame_id = "map";
  message.header.stamp = timeMessage(now_ns);
  message.hazards = std::move(hazards);
  return message;
}

class TestableAerialSupportLayer : public AerialSupportLayer
{
public:
  int64_t test_now_ns{10 * kSecond};

  void ingest(const HazardArray & message)
  {
    incomingHazards(std::make_shared<HazardArray>(message));
  }

protected:
  int64_t nowNanoseconds() const override
  {
    return test_now_ns;
  }
};

class AerialSupportLayerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
          rclcpp::Parameter("aerial_support_layer.enabled", true),
          rclcpp::Parameter("aerial_support_layer.covariance_sigma_scale", 2.0),
    });
    node_ = std::make_shared<nav2_util::LifecycleNode>("aerial_support_layer_test", "", options);
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layered_costmap_->resizeMap(100, 100, 0.1, -5.0, -5.0);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    layer_ = std::make_shared<TestableAerialSupportLayer>();
    layer_->initialize(
      layered_costmap_.get(), "aerial_support_layer", tf_buffer_.get(), node_, callback_group_);
    layer_->activate();
    master_ = layered_costmap_->getCostmap();
    clearMaster();
  }

  void TearDown() override
  {
    layer_->deactivate();
    layer_.reset();
    tf_buffer_.reset();
    layered_costmap_.reset();
    node_.reset();
  }

  void clearMaster()
  {
    master_->resetMapToValue(
      0, 0, master_->getSizeInCellsX(), master_->getSizeInCellsY(),
      nav2_costmap_2d::FREE_SPACE);
  }

  void cycle(bool clear_master = true)
  {
    if (clear_master) {
      clearMaster();
    }
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
    layer_->updateCosts(
      *master_, 0, 0,
      static_cast<int>(master_->getSizeInCellsX()),
      static_cast<int>(master_->getSizeInCellsY()));
  }

  unsigned char costAt(double x, double y) const
  {
    unsigned int mx = 0;
    unsigned int my = 0;
    EXPECT_TRUE(master_->worldToMap(x, y, mx, my));
    return master_->getCost(mx, my);
  }

  size_t markedCellCount() const
  {
    size_t count = 0;
    const auto * map = layer_->getCharMap();
    const size_t cells = layer_->getSizeInCellsX() * layer_->getSizeInCellsY();
    for (size_t index = 0; index < cells; ++index) {
      if (map[index] != nav2_costmap_2d::NO_INFORMATION) {
        ++count;
      }
    }
    return count;
  }

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<TestableAerialSupportLayer> layer_;
  nav2_costmap_2d::Costmap2D * master_{nullptr};
};

TEST_F(AerialSupportLayerTest, ValidConfirmedHazardMarksLethalCost)
{
  layer_->ingest(makeArray(layer_->test_now_ns, {makeHazard(layer_->test_now_ns)}));
  cycle();
  EXPECT_EQ(costAt(0.0, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(AerialSupportLayerTest, TentativeHazardUsesConfiguredHighCost)
{
  layer_->ingest(makeArray(
    layer_->test_now_ns, {makeHazard(layer_->test_now_ns, "tentative", Hazard::TENTATIVE)}));
  cycle();
  EXPECT_EQ(costAt(0.0, 0.0), 200);
}

TEST_F(AerialSupportLayerTest, ConflictHazardUsesConfiguredConservativeCost)
{
  layer_->ingest(makeArray(
    layer_->test_now_ns, {makeHazard(layer_->test_now_ns, "conflict", Hazard::CONFLICT)}));
  cycle();
  EXPECT_EQ(costAt(0.0, 0.0), 220);
}

TEST_F(AerialSupportLayerTest, StaleObservationIsRejected)
{
  auto hazard = makeHazard(layer_->test_now_ns - 2 * kSecond);
  layer_->ingest(makeArray(layer_->test_now_ns - 2 * kSecond, {hazard}));
  cycle();
  EXPECT_EQ(markedCellCount(), 0U);
}

TEST_F(AerialSupportLayerTest, ExpiredTtlIsRejected)
{
  auto hazard = makeHazard(layer_->test_now_ns - kSecond, "expired", Hazard::CONFIRMED,
    0.0, 0.0, 0.01, 0.5);
  layer_->ingest(makeArray(layer_->test_now_ns, {hazard}));
  cycle();
  EXPECT_EQ(markedCellCount(), 0U);
}

TEST_F(AerialSupportLayerTest, ExcessiveCovarianceIsRejected)
{
  layer_->ingest(makeArray(
    layer_->test_now_ns,
      {makeHazard(layer_->test_now_ns, "uncertain", Hazard::CONFIRMED, 0.0, 0.0, 1.1)}));
  cycle();
  EXPECT_EQ(markedCellCount(), 0U);
}

TEST_F(AerialSupportLayerTest, CovarianceEnlargesFootprint)
{
  layer_->ingest(makeArray(
    layer_->test_now_ns,
      {makeHazard(layer_->test_now_ns, "small", Hazard::CONFIRMED, 0.0, 0.0, 0.01)}));
  cycle();
  const size_t small_count = markedCellCount();

  layer_->ingest(makeArray(
    layer_->test_now_ns,
      {makeHazard(layer_->test_now_ns, "large", Hazard::CONFIRMED, 0.0, 0.0, 0.25)}));
  cycle();
  EXPECT_GT(markedCellCount(), small_count);
}

TEST_F(AerialSupportLayerTest, TransformsObservationAtItsTimestampIntoMap)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = timeMessage(layer_->test_now_ns);
  transform.child_frame_id = "sensor_frame";
  transform.transform.translation.x = 1.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer_->setTransform(transform, "test", true));

  auto hazard = makeHazard(layer_->test_now_ns, "transformed");
  hazard.detection.header.frame_id = "sensor_frame";
  auto message = makeArray(layer_->test_now_ns, {hazard});
  message.header.frame_id = "sensor_frame";
  layer_->ingest(message);
  cycle();
  EXPECT_EQ(costAt(1.0, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(AerialSupportLayerTest, TrackMovementClearsOldFootprint)
{
  layer_->ingest(makeArray(
    layer_->test_now_ns,
      {makeHazard(layer_->test_now_ns, "moving", Hazard::CONFIRMED, -2.0, 0.0)}));
  cycle();
  EXPECT_EQ(costAt(-2.0, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);

  layer_->ingest(makeArray(
    layer_->test_now_ns,
      {makeHazard(layer_->test_now_ns, "moving", Hazard::CONFIRMED, 2.0, 0.0)}));
  cycle();
  EXPECT_EQ(costAt(-2.0, 0.0), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(costAt(2.0, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(AerialSupportLayerTest, TrackExpiryClearsFootprint)
{
  layer_->ingest(makeArray(
    layer_->test_now_ns,
      {makeHazard(layer_->test_now_ns, "short", Hazard::CONFIRMED, 0.0, 0.0, 0.01, 0.5)}));
  cycle();
  EXPECT_EQ(costAt(0.0, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);

  layer_->test_now_ns += kSecond;
  cycle();
  EXPECT_EQ(costAt(0.0, 0.0), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(markedCellCount(), 0U);
}

TEST_F(AerialSupportLayerTest, ResetClearsLayer)
{
  layer_->ingest(makeArray(layer_->test_now_ns, {makeHazard(layer_->test_now_ns)}));
  cycle();
  ASSERT_GT(markedCellCount(), 0U);
  layer_->reset();
  cycle();
  EXPECT_EQ(markedCellCount(), 0U);
  EXPECT_EQ(costAt(0.0, 0.0), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(AerialSupportLayerTest, DisablingAndDeactivationClearLayer)
{
  layer_->ingest(makeArray(layer_->test_now_ns, {makeHazard(layer_->test_now_ns)}));
  cycle();
  ASSERT_GT(markedCellCount(), 0U);

  auto result = node_->set_parameter(rclcpp::Parameter("aerial_support_layer.enabled", false));
  ASSERT_TRUE(result.successful);
  cycle();
  EXPECT_EQ(markedCellCount(), 0U);
  EXPECT_EQ(costAt(0.0, 0.0), nav2_costmap_2d::FREE_SPACE);

  result = node_->set_parameter(rclcpp::Parameter("aerial_support_layer.enabled", true));
  ASSERT_TRUE(result.successful);
  layer_->ingest(makeArray(layer_->test_now_ns, {makeHazard(layer_->test_now_ns)}));
  cycle();
  ASSERT_GT(markedCellCount(), 0U);
  layer_->deactivate();
  EXPECT_EQ(markedCellCount(), 0U);
}

TEST_F(AerialSupportLayerTest, MaximumMergePreservesStaticLethalCell)
{
  layer_->ingest(makeArray(
    layer_->test_now_ns,
      {makeHazard(layer_->test_now_ns, "tentative", Hazard::TENTATIVE)}));
  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(master_->worldToMap(0.0, 0.0, mx, my));
  master_->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  layer_->updateCosts(*master_, 0, 0, 100, 100);
  EXPECT_EQ(master_->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(AerialSupportLayerTest, MalformedInputIsRejectedWithoutThrowing)
{
  auto malformed = makeHazard(layer_->test_now_ns);
  malformed.detection.id.clear();
  malformed.detection.bbox.size.x = -1.0;
  malformed.detection.results.front().hypothesis.score =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_NO_THROW(layer_->ingest(makeArray(layer_->test_now_ns, {malformed})));
  cycle();
  EXPECT_EQ(markedCellCount(), 0U);
}

TEST_F(AerialSupportLayerTest, EmptyArrayClearsTracks)
{
  layer_->ingest(makeArray(layer_->test_now_ns, {makeHazard(layer_->test_now_ns)}));
  cycle();
  ASSERT_GT(markedCellCount(), 0U);
  layer_->ingest(makeArray(layer_->test_now_ns, {}));
  cycle();
  EXPECT_EQ(markedCellCount(), 0U);
  EXPECT_EQ(costAt(0.0, 0.0), nav2_costmap_2d::FREE_SPACE);
}

TEST(AerialSupportLayerPluginlibTest, DiscoversAndLoadsPlugin)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  const auto classes = loader.getDeclaredClasses();
  EXPECT_NE(
    std::find(
      classes.begin(), classes.end(),
      "lrs_halmstad_nav_plugins::AerialSupportLayer"),
    classes.end());
  EXPECT_NO_THROW({
        auto plugin = loader.createSharedInstance(
      "lrs_halmstad_nav_plugins::AerialSupportLayer");
        EXPECT_TRUE(plugin->isClearable());
  });
}

}  // namespace
}  // namespace lrs_halmstad_nav_plugins

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
