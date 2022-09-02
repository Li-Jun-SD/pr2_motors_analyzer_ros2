#ifndef PR2_MOTORS_ANALYZER_H
#define PR2_MOTORS_ANALYZER_H

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_aggregator/analyzer.hpp>
#include <diagnostic_aggregator/status_item.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <string>

namespace diagnostic_aggregator {

class PR2MotorsAnalyzer : public Analyzer
{
public:
  PR2MotorsAnalyzer();
  ~PR2MotorsAnalyzer();

  bool init(const std::string & base_path, const std::string & breadcrumb, const rclcpp::Node::SharedPtr node);

  bool match(const std::string & name);

  bool analyze(const std::shared_ptr<StatusItem> item);

  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report();

  std::string getPath() const { return path_; };

  std::string getName() const { return nice_name_; };

private:

  // Store status item for EtherCAT master
  std::shared_ptr<StatusItem> eth_master_item_;

  std::string path_, nice_name_, power_board_name_;

  bool runstop_hit_, has_initialized_, has_power_data_, has_eth_data_;
};

}

#endif  // PR2_MOTORS_ANALYZER_H