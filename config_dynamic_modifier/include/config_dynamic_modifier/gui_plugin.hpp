#ifndef GUI_PLUGIN_HPP_
#define GUI_PLUGIN_HPP_

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QSpinBox>
#include <QTextEdit>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace config_dynamic_modifier
{

class ConfigModifierGUI : public QWidget
{
  Q_OBJECT

public:
  ConfigModifierGUI(QWidget* parent = nullptr);
  ~ConfigModifierGUI();

private slots:
  void onAreaChanged(int index);
  void onPointCountChanged(int value);
  void onApplyClicked();
  void onResetClicked();
  void onReloadClicked();
  void onStatusReceived(const QString& status);

private:
  void setupUI();
  void sendCommand(const QString& command);
  void updateStatus(const QString& status);
  void updatePointCountSpin();

  // ROS相关
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  
  // UI组件
  QComboBox* area_combo_;
  QSpinBox* point_count_spin_;
  QPushButton* apply_btn_;
  QPushButton* reset_btn_;
  QPushButton* reload_btn_;
  QTextEdit* status_text_;
  
  // 状态
  int current_area_;
  std::vector<int> area_point_counts_;
};

}  // namespace config_dynamic_modifier

#endif  // GUI_PLUGIN_HPP_ 