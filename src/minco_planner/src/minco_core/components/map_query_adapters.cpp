#include "minco_core/components/map_query_adapters.hpp"

#include "tf2/time.h"

#include <Eigen/Geometry>

#include <cmath>
#include <algorithm>

namespace minco_planner {

namespace {

Eigen::Vector3d transformPoint(const geometry_msgs::msg::TransformStamped & tf, const Eigen::Vector3d & p)
{
  const auto & q_msg = tf.transform.rotation;
  const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  const Eigen::Vector3d t(
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
  return q * p + t;
}

Eigen::Vector3d rotateVector(const geometry_msgs::msg::TransformStamped & tf, const Eigen::Vector3d & v)
{
  const auto & q_msg = tf.transform.rotation;
  const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  return q * v;
}

bool isLethalCost(const unsigned char cost)
{
  return cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

}  // namespace

Nav2CostmapQuery::Nav2CostmapQuery(nav2_costmap_2d::Costmap2D * costmap) : costmap_(costmap)
{
}

bool Nav2CostmapQuery::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  return costmap_ && costmap_->worldToMap(wx, wy, mx, my);
}

void Nav2CostmapQuery::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  if (!costmap_) {
    wx = 0.0;
    wy = 0.0;
    return;
  }
  costmap_->mapToWorld(mx, my, wx, wy);
}

unsigned int Nav2CostmapQuery::sizeX() const
{
  return costmap_ ? costmap_->getSizeInCellsX() : 0U;
}
unsigned int Nav2CostmapQuery::sizeY() const
{
  return costmap_ ? costmap_->getSizeInCellsY() : 0U;
}
double Nav2CostmapQuery::resolution() const
{
  return costmap_ ? costmap_->getResolution() : 0.0;
}
double Nav2CostmapQuery::originX() const
{
  return costmap_ ? costmap_->getOriginX() : 0.0;
}
double Nav2CostmapQuery::originY() const
{
  return costmap_ ? costmap_->getOriginY() : 0.0;
}

uint8_t Nav2CostmapQuery::value(unsigned int mx, unsigned int my) const
{
  if (!isValid(mx, my)) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  return costmap_->getCost(mx, my);
}

const unsigned char * Nav2CostmapQuery::values() const
{
  return costmap_ ? costmap_->getCharMap() : nullptr;
}

bool Nav2CostmapQuery::copyValues(std::vector<unsigned char> & out) const
{
  const auto * data = values();
  const size_t count = static_cast<size_t>(sizeX()) * static_cast<size_t>(sizeY());
  if (!data || count == 0U) {
    out.clear();
    return false;
  }
  out.assign(data, data + count);
  return true;
}

bool Nav2CostmapQuery::isValid(unsigned int mx, unsigned int my) const
{
  return costmap_ && mx < sizeX() && my < sizeY();
}

bool Nav2CostmapQuery::isFree(unsigned int mx, unsigned int my) const
{
  if (!isValid(mx, my)) {
    return false;
  }
  const auto cost = costmap_->getCost(mx, my);
  return cost != nav2_costmap_2d::NO_INFORMATION && cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

bool Nav2CostmapQuery::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  const auto result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

rog_map::QueryResult Nav2CostmapQuery::query(const Eigen::Vector3d & pos) const
{
  rog_map::QueryResult result;
  if (!pos.allFinite()) {
    result.status = rog_map::QueryStatus::NONFINITE_INPUT;
    return result;
  }
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!worldToMap(pos.x(), pos.y(), mx, my)) {
    result.status = rog_map::QueryStatus::OUT_OF_MAP;
    return result;
  }
  const auto cost = value(mx, my);
  result.ok = true;
  result.status = rog_map::QueryStatus::OK;
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    result.distance = -1.0;
  } else if (isLethalCost(cost)) {
    result.distance = -resolution();
  } else if (cost == nav2_costmap_2d::FREE_SPACE) {
    result.distance = 6.0;
  } else {
    // rog_map_costmap_layer encodes positive ESDF distance as
    // cost=252*exp(-cost_scaling_factor*distance). Keep this adapter
    // consistent with that layer while retaining a useful far-field cap.
    constexpr double kCostScale = 2.0;
    result.distance = std::clamp(
      -std::log(std::max(1.0, static_cast<double>(cost)) / 252.0) / kCostScale,
      0.0, 6.0);
  }
  result.gradient = Eigen::Vector3d::Zero();
  return result;
}

FrameAwareRogQuery::FrameAwareRogQuery(std::shared_ptr<rog_map::MapQueryInterface> raw,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::string planning_frame,
  std::string rog_frame,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Logger logger)
: raw_(std::move(raw)), tf_(std::move(tf)), planning_frame_(std::move(planning_frame)),
  rog_frame_(std::move(rog_frame)), clock_(std::move(clock)), logger_(logger)
{
}

bool FrameAwareRogQuery::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (!raw_) {
    return false;
  }
  Eigen::Vector3d p(wx, wy, 0.0);
  if (!transformPlanningToRog(p, p)) {
    return false;
  }
  return raw_->worldToMap(p.x(), p.y(), mx, my);
}

void FrameAwareRogQuery::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  wx = 0.0;
  wy = 0.0;
  if (!raw_) {
    return;
  }
  double rwx = 0.0;
  double rwy = 0.0;
  raw_->mapToWorld(mx, my, rwx, rwy);
  Eigen::Vector3d p(rwx, rwy, 0.0);
  if (!transformRogToPlanning(p, p)) {
    return;
  }
  wx = p.x();
  wy = p.y();
}

unsigned int FrameAwareRogQuery::sizeX() const
{
  return raw_ ? raw_->sizeX() : 0U;
}
unsigned int FrameAwareRogQuery::sizeY() const
{
  return raw_ ? raw_->sizeY() : 0U;
}
double FrameAwareRogQuery::resolution() const
{
  return raw_ ? raw_->resolution() : 0.0;
}
double FrameAwareRogQuery::originX() const
{
  return raw_ ? raw_->originX() : 0.0;
}
double FrameAwareRogQuery::originY() const
{
  return raw_ ? raw_->originY() : 0.0;
}

uint8_t FrameAwareRogQuery::value(unsigned int mx, unsigned int my) const
{
  return raw_ ? raw_->value(mx, my) : nav2_costmap_2d::LETHAL_OBSTACLE;
}

const unsigned char * FrameAwareRogQuery::values() const
{
  return raw_ ? raw_->values() : nullptr;
}

bool FrameAwareRogQuery::copyValues(std::vector<unsigned char> & out) const
{
  return raw_ && raw_->copyValues(out);
}

bool FrameAwareRogQuery::isValid(unsigned int mx, unsigned int my) const
{
  return raw_ && raw_->isValid(mx, my);
}

bool FrameAwareRogQuery::isFree(unsigned int mx, unsigned int my) const
{
  return raw_ && raw_->isFree(mx, my);
}

bool FrameAwareRogQuery::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  const auto result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

rog_map::QueryResult FrameAwareRogQuery::query(const Eigen::Vector3d & pos) const
{
  rog_map::QueryResult result;
  if (!pos.allFinite()) {
    result.status = rog_map::QueryStatus::NONFINITE_INPUT;
    return result;
  }
  if (!raw_) {
    result.status = rog_map::QueryStatus::SNAPSHOT_INVALID;
    return result;
  }
  Eigen::Vector3d rog_pos = pos;
  if (!transformPlanningToRog(pos, rog_pos)) {
    result.status = rog_map::QueryStatus::TF_FAILED;
    return result;
  }
  result = raw_->query(rog_pos);
  if (!result.ok) {
    // MINCO needs a usable gradient when a trial trajectory leaves the
    // rolling ROGMap window.  Returning a failed query makes the optimizer
    // use a zero gradient, so L-BFGS can keep moving farther outside and the
    // whole trajectory is rejected afterwards.  Convert the rectangular
    // map boundary into a soft signed-distance constraint instead.  The
    // safety checker still performs its strict worldToMap check, therefore
    // this fallback can only guide optimization and cannot approve an
    // out-of-window command.
    if (result.status == rog_map::QueryStatus::OUT_OF_MAP &&
        raw_->resolution() > 0.0 && raw_->sizeX() > 1U && raw_->sizeY() > 1U)
    {
      const double resolution = raw_->resolution();
      const double min_x = raw_->originX();
      const double min_y = raw_->originY();
      // QueryAdapter requires a one-cell interpolation neighbourhood, so
      // its effective upper bound is the last cell centre rather than the
      // outer edge of the allocated array.
      const double max_x = min_x + (static_cast<double>(raw_->sizeX()) - 1.0) * resolution;
      const double max_y = min_y + (static_cast<double>(raw_->sizeY()) - 1.0) * resolution;
      const double dx = rog_pos.x() < min_x ? (min_x - rog_pos.x()) :
                        (rog_pos.x() > max_x ? (rog_pos.x() - max_x) : 0.0);
      const double dy = rog_pos.y() < min_y ? (min_y - rog_pos.y()) :
                        (rog_pos.y() > max_y ? (rog_pos.y() - max_y) : 0.0);
      const double penetration = std::hypot(dx, dy);
      if (penetration > 0.0) {
        Eigen::Vector3d inward_rog = Eigen::Vector3d::Zero();
        if (dx > 0.0) {
          inward_rog.x() = rog_pos.x() < min_x ? 1.0 : -1.0;
        }
        if (dy > 0.0) {
          inward_rog.y() = rog_pos.y() < min_y ? 1.0 : -1.0;
        }
        inward_rog /= inward_rog.norm();
        Eigen::Vector3d inward_planning = Eigen::Vector3d::Zero();
        if (rotateRogToPlanning(inward_rog, inward_planning)) {
          result.ok = true;
          result.status = rog_map::QueryStatus::OK;
          result.distance = -(penetration + resolution);
          result.gradient = inward_planning;
          return result;
        }
      }
    }
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[MincoPlanner] ROGMap query failed: status=%s planning=(%.3f, %.3f, %.3f) "
      "rog=(%.3f, %.3f, %.3f) snapshot=%lu field=%lu",
      rog_map::queryStatusName(result.status),
      pos.x(), pos.y(), pos.z(),
      rog_pos.x(), rog_pos.y(), rog_pos.z(),
      static_cast<unsigned long>(result.snapshot_sequence),
      static_cast<unsigned long>(result.field_sequence));
    return result;
  }
  Eigen::Vector3d planning_grad = Eigen::Vector3d::Zero();
  if (rotateRogToPlanning(result.gradient, planning_grad)) {
    result.gradient = planning_grad;
  }
  if (!std::isfinite(result.distance) || !result.gradient.allFinite()) {
    result.ok = false;
    result.status = rog_map::QueryStatus::NONFINITE_OUTPUT;
  }
  return result;
}

bool FrameAwareRogQuery::transformPlanningToRog(const Eigen::Vector3d & in, Eigen::Vector3d & out) const
{
  if (planning_frame_ == rog_frame_) {
    out = in;
    return true;
  }
  if (!tf_) {
    return false;
  }
  try {
    const auto tf = tf_->lookupTransform(rog_frame_, planning_frame_, tf2::TimePointZero);
    out = transformPoint(tf, in);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[MincoPlanner] TF %s -> %s failed for ROGMap dynamic query: %s",
      planning_frame_.c_str(),
      rog_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool FrameAwareRogQuery::transformRogToPlanning(const Eigen::Vector3d & in, Eigen::Vector3d & out) const
{
  if (planning_frame_ == rog_frame_) {
    out = in;
    return true;
  }
  if (!tf_) {
    return false;
  }
  try {
    const auto tf = tf_->lookupTransform(planning_frame_, rog_frame_, tf2::TimePointZero);
    out = transformPoint(tf, in);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_,
      *clock_,
      2000,
      "[MincoPlanner] TF %s -> %s failed for ROGMap dynamic query: %s",
      rog_frame_.c_str(),
      planning_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool FrameAwareRogQuery::rotateRogToPlanning(const Eigen::Vector3d & in, Eigen::Vector3d & out) const
{
  if (planning_frame_ == rog_frame_) {
    out = in;
    return true;
  }
  if (!tf_) {
    return false;
  }
  try {
    const auto tf = tf_->lookupTransform(planning_frame_, rog_frame_, tf2::TimePointZero);
    out = rotateVector(tf, in);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

}  // namespace minco_planner
