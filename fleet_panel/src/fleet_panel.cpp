#include "fleet_panel/fleet_panel.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <QFont>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace fleet_panel
{

static const char * kStateNames[] = {"IDLE", "SEARCH", "CHASE", "RTB", "LOST"};

// per-robot accent colours (match the sim's neon palette)
static std::string robotColor(const std::string & id)
{
  if (id == "chomp") {return "#ff8c1a";}   // orange
  if (id == "dash") {return "#1ae6ff";}    // cyan
  if (id == "nibble") {return "#ff33d9";}  // magenta
  return "#dddddd";
}

FleetPanel::FleetPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);

  auto * title = new QLabel("PACFLEET — FLEET", this);
  QFont tf;
  tf.setBold(true);
  tf.setPointSize(12);
  title->setFont(tf);
  layout->addWidget(title);

  body_ = new QLabel("waiting for fleet…", this);
  QFont bf("Monospace");
  bf.setStyleHint(QFont::TypeWriter);
  bf.setPointSize(10);
  body_->setFont(bf);
  body_->setTextFormat(Qt::RichText);
  body_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  body_->setWordWrap(false);
  body_->setOpenExternalLinks(false);
  body_->setTextInteractionFlags(Qt::TextBrowserInteraction);
  connect(body_, &QLabel::linkActivated, this, &FleetPanel::onRobotClicked);
  layout->addWidget(body_);

  auto * hint = new QLabel("click a robot to focus the camera", this);
  hint->setStyleSheet("color:#6b7d90");
  layout->addWidget(hint);
  layout->addStretch();
}

void FleetPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  fleet_sub_ = node_->create_subscription<fleet_interfaces::msg::FleetStatus>(
    "/hub/fleet_status", 10,
    std::bind(&FleetPanel::onFleet, this, std::placeholders::_1));
  fb_sub_ =
    node_->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
    "/ops/console/feedback", 10,
    std::bind(&FleetPanel::onFeedback, this, std::placeholders::_1));

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &FleetPanel::refresh);
  timer_->start(200);
}

void FleetPanel::onFleet(const fleet_interfaces::msg::FleetStatus::SharedPtr msg)
{
  latest_ = *msg;
}

void FleetPanel::onFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg)
{
  if (msg->event_type ==
    visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK)
  {
    selected_ = msg->marker_name;
  }
}

static std::string batteryBar(double pct)
{
  const int n = 10;
  int f = static_cast<int>(std::lround(std::clamp(pct, 0.0, 100.0) / 100.0 * n));
  std::string s = "[";
  for (int i = 0; i < n; ++i) {s += (i < f) ? "█" : "·";}
  s += "]";
  return s;
}

void FleetPanel::refresh()
{
  if (latest_.robots.empty()) {
    return;
  }
  std::ostringstream h;
  h << "<div style='line-height:150%'>";
  for (const auto & r : latest_.robots) {
    const bool sel = (r.robot_id == selected_);
    const char * st =
      (r.state < 5) ? kStateNames[r.state] : "?";
    const std::string col = robotColor(r.robot_id);

    std::string name = r.robot_id;
    std::transform(name.begin(), name.end(), name.begin(), ::toupper);

    h << "<div style='margin-bottom:8px; padding:4px; ";
    if (sel) {
      h << "background:#20304a; border:1px solid " << col << ";";
    }
    h << "'>";
    h << "<a href='" << r.robot_id << "' style='color:" << col
      << "; font-weight:bold; text-decoration:none'>"
      << (sel ? "▶ " : "&nbsp;&nbsp;") << name << "</a>"
      << "<span style='color:#9fb3c8'> &nbsp;" << st << "</span><br>";

    std::ostringstream batt;
    batt.setf(std::ios::fixed);
    batt.precision(0);
    batt << batteryBar(r.battery) << " " << r.battery << "%";
    std::ostringstream spd;
    spd.setf(std::ios::fixed);
    spd.precision(1);
    spd << r.speed << " m/s";

    std::string bcol = (r.battery < 25.0) ? "#ff6b3d" : "#8fe388";
    h << "<span style='color:" << bcol << "'>" << batt.str() << "</span>"
      << "<span style='color:#9fb3c8'> &nbsp; " << spd.str() << "</span><br>";

    std::ostringstream pos;
    pos.setf(std::ios::fixed);
    pos.precision(0);
    pos << "pos " << r.x << ", " << r.y;
    h << "<span style='color:#6b7d90'>" << pos.str() << "</span>";
    h << "</div>";
  }
  h << "</div>";
  body_->setText(QString::fromStdString(h.str()));
}

void FleetPanel::onRobotClicked(const QString & robot_id)
{
  selected_ = robot_id.toStdString();
  for (const auto & r : latest_.robots) {
    if (r.robot_id != selected_) {
      continue;
    }
    auto * vm = getDisplayContext()->getViewManager();
    if (vm && vm->getCurrent()) {
      auto * vc = vm->getCurrent();          // move the Orbit focal point onto the robot
      vc->subProp("Focal Point")->subProp("X")->setValue(r.x);
      vc->subProp("Focal Point")->subProp("Y")->setValue(r.y);
      vc->subProp("Focal Point")->subProp("Z")->setValue(0.0);
    }
    break;
  }
  refresh();
}

}  // namespace fleet_panel

PLUGINLIB_EXPORT_CLASS(fleet_panel::FleetPanel, rviz_common::Panel)
