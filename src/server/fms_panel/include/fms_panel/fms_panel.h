// RViz panel declarations for the operator-facing fleet dashboard.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "fms_msgs/msg/robot_info.hpp" 
#include "fms_msgs/msg/robots.hpp" 
#include "fms_msgs/msg/robot_state.hpp"
#include "fms_msgs/msg/tasks.hpp"
#include "fms_msgs/msg/task_state.hpp"
#include "fms_msgs/msg/task_info.hpp"

#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QTableWidget>
#include <QScrollArea>
#include <QHeaderView> 
#include <QTimer>  // QTimer 

#include "rviz_common/panel.hpp"
#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace fms_panel
{
    class FmsPanel : public rviz_common::Panel
    {
        Q_OBJECT
        public:
            explicit FmsPanel(QWidget *parent = nullptr);    
            void onInitialize() override; 

        private:
            //  
            void getRobots(const fms_msgs::msg::Robots::SharedPtr msg);
            void getTasks(const fms_msgs::msg::Tasks::SharedPtr msg);
            void updateStateTable();

            QString checkMode(int i);
            QString checkType(int i);
            QString checkState(int i);
            QString checkTaskType(int i);

            // rclcpp  
            std::shared_ptr<rclcpp::Node> node_;
            rclcpp::Subscription<fms_msgs::msg::Robots>::SharedPtr robots_subscriber;
            rclcpp::Subscription<fms_msgs::msg::Tasks>::SharedPtr tasks_subscriber;


            fms_msgs::msg::Robots::SharedPtr subRobots;
            fms_msgs::msg::Tasks::SharedPtr subTasks;

            // QTimer   
            QTimer *timer_;  //  updateRobotState  

            //   
            QTableWidget *table_1;
            QTableWidget *table_2;
    };
} // namespace fms_panel
