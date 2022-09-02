#include "pr2_motors_analyzer/pr2_motors_analyzer.hpp"

namespace diagnostic_aggregator {

PR2MotorsAnalyzer::PR2MotorsAnalyzer() :
  path_(""),
  nice_name_("Motors"),
  power_board_name_(""),
  runstop_hit_(false),
  has_initialized_(false),
  has_power_data_(false),
  has_eth_data_(false)  
{}

PR2MotorsAnalyzer::~PR2MotorsAnalyzer() {}

bool PR2MotorsAnalyzer::init(const std::string & base_path, const std::string & breadcrumb, const rclcpp::Node::SharedPtr node)
{
  (void)breadcrumb;

  // path_ = BASE_NAME/Motors
  path_ = base_path + "/" + nice_name_;

  if (node->get_parameter("power_board_name", power_board_name_)) {
    // It not work. Maybe someone could fix it. But for me, just comment it. Afterall, me is a noob.
    // RCLCPP_ERROR(node->get_logger(), 
    //             "No power board name was specified in PR2MotorsAnalyzer! Power board must be \"Power board 10XX\". Namespace: %s", 
    //              node->get_namespace()->c_str());
    return false;
  }

  // Make a "missing" item for the EtherCAT Master.
  std::shared_ptr<StatusItem> item = std::make_shared<StatusItem>("EtherCAT Master");
  eth_master_item_ = item;

  has_initialized_ = true;

  return true;
}

bool PR2MotorsAnalyzer::match(const std::string & name)
{
  if (name == "EtherCAT Master")
    return true;

  return name == power_board_name_;
}

bool PR2MotorsAnalyzer::analyze(const std::shared_ptr<StatusItem> item)
{
  if (item->getName() == power_board_name_) {
    has_power_data_ = true;
    runstop_hit_ = item->getValue("Runstop hit") == "True" || 
                   item->getValue("Estop hit") == "True";
    return false;  // Won't report this item.
  }

  // We know out item is "EtherCAT Master"
  eth_master_item_ = item;
  has_eth_data_ = true;

  return true;
}

std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> PR2MotorsAnalyzer::report()
{
  std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus> eth_stat = eth_master_item_->toStatusMsg(path_);

  // If we have power data, and runstop is hit, we'll suppress errors.
  if (has_eth_data_ && has_power_data_ && runstop_hit_) {
    eth_stat->level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> output;
  output.push_back(eth_stat);

  return output;
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::PR2MotorsAnalyzer, diagnostic_aggregator::Analyzer)