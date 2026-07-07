#ifndef FLEET_PANEL__FLEET_PANEL_HPP_
#define FLEET_PANEL__FLEET_PANEL_HPP_

#include <map>
#include <string>

#include <QString>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <fleet_interfaces/msg/fleet_status.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

class QLabel;
class QTimer;

namespace fleet_panel
{

// A docked RViz panel showing every PacFleet robot's live telemetry. Clicking a
// robot's ring in the 3D view (Interact tool) focuses/highlights it here.
class FleetPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit FleetPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void refresh();
  void onRobotClicked(const QString & robot_id);

private:
  void onFleet(const fleet_interfaces::msg::FleetStatus::SharedPtr msg);
  void onFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<fleet_interfaces::msg::FleetStatus>::SharedPtr fleet_sub_;
  rclcpp::Subscription<
    visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr fb_sub_;

  fleet_interfaces::msg::FleetStatus latest_;
  std::string selected_;
  QLabel * body_ {nullptr};
  QTimer * timer_ {nullptr};
};

}  // namespace fleet_panel

#endif  // FLEET_PANEL__FLEET_PANEL_HPP_
