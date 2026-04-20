// RViz panel implementation for monitoring robot and task tables during operation.

#include "fms_panel/fms_panel.h"

namespace fms_panel
{
    FmsPanel::FmsPanel(QWidget *parent)
            : rviz_common::Panel(parent),
              node_(std::make_shared<rclcpp::Node>("fms_panel_node"))
    {
        auto layout = new QVBoxLayout();
        layout->setContentsMargins(20, 0, 0, 0);

        //  
        auto blank = new QLabel("");
        auto text = new QLabel("Robot State");
        auto text_2 = new QLabel("Task State");

        //  
        table_1 = new QTableWidget(0, 8, this);
        table_2 = new QTableWidget(0, 8, this);

        //   
        QStringList RobotHeaders;
        RobotHeaders << "Robot ID" << "Type" << "Mode" << "Task ID" << "Progress" << "Battery" << "Goal Node" << "Time Delay";
        table_1->setHorizontalHeaderLabels(RobotHeaders);

        QStringList TaskHeaders;
        TaskHeaders << "Task ID" << "Type" << "State" << "Priority" << "Robot ID" << "Target_0" << "Target_1" << "Target_2";
        table_2->setHorizontalHeaderLabels(TaskHeaders);

        //     (   )
        table_1->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        table_2->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

        //      QScrollArea 
        auto scroll_area = new QScrollArea(this);
        scroll_area->setWidget(table_1);
        scroll_area->setWidgetResizable(true);

        auto scroll_area_2 = new QScrollArea(this);
        scroll_area_2->setWidget(table_2);
        scroll_area_2->setWidgetResizable(true);

        //   
        layout->addWidget(blank);
        layout->addWidget(text);
        layout->addWidget(scroll_area);

        layout->addWidget(blank);
        layout->addWidget(text_2);
        layout->addWidget(scroll_area_2);

        setLayout(layout);

        robots_subscriber = node_->create_subscription<fms_msgs::msg::Robots>(
            "/fms/robots", 5,
            std::bind(&FmsPanel::getRobots, this, std::placeholders::_1));


        tasks_subscriber = node_->create_subscription<fms_msgs::msg::Tasks>(
            "/fms/tasks", 5,
            std::bind(&FmsPanel::getTasks, this, std::placeholders::_1));


        //     - 1 updateRobotState 
        timer_ = new QTimer(this);
        connect(timer_, &QTimer::timeout, this, &FmsPanel::updateStateTable);
        timer_->start(200);  // 1000ms (1)
    }

    void FmsPanel::getRobots(const fms_msgs::msg::Robots::SharedPtr msg)
    {
       subRobots = msg;
    }

    void FmsPanel::getTasks(const fms_msgs::msg::Tasks::SharedPtr msg)
    {
       subTasks = msg;
    }

    void FmsPanel::updateStateTable()
    {
        rclcpp::spin_some(node_);

        if (subRobots && !subRobots->robots.empty()) {
            table_1->setRowCount(subRobots->robots.size());
            for (int r = 0; r < static_cast<int>(subRobots->robots.size()); ++r) {
                table_1->setItem(r, 0, new QTableWidgetItem(QString::fromStdString(subRobots->robots[r].robot_id)));
                table_1->setItem(r, 1, new QTableWidgetItem(checkType(r)));
                table_1->setItem(r, 2, new QTableWidgetItem(checkMode(r)));
                table_1->setItem(r, 3, new QTableWidgetItem(QString::fromStdString(subRobots->robots[r].task_id)));
                table_1->setItem(r, 4, new QTableWidgetItem(QString::number(subRobots->robots[r].progress)));
                table_1->setItem(r, 5, new QTableWidgetItem(QString::number(subRobots->robots[r].battery)));
                table_1->setItem(r, 6, new QTableWidgetItem(QString::fromStdString(subRobots->robots[r].goal_node.name)));
                table_1->setItem(r, 7, new QTableWidgetItem(QString::number(subRobots->robots[r].time_delay)));
            }
        } else {
            table_1->setRowCount(0);
        }

        if (subTasks && !subTasks->tasks.empty()) {
            table_2->setRowCount(subTasks->tasks.size());
            for (int t = 0; t < static_cast<int>(subTasks->tasks.size()); ++t) {
                table_2->setItem(t, 0, new QTableWidgetItem(QString::fromStdString(subTasks->tasks[t].task_id)));
                table_2->setItem(t, 1, new QTableWidgetItem(checkTaskType(t)));
                table_2->setItem(t, 2, new QTableWidgetItem(checkState(t)));
                table_2->setItem(t, 3, new QTableWidgetItem(QString::number(subTasks->tasks[t].info.priority)));
                table_2->setItem(t, 4, new QTableWidgetItem(QString::fromStdString(subTasks->tasks[t].robot_id)));
                // task_nodes  3  
                for (int n = 0; n < 3; ++n) {
                    QString nodeName = (subTasks->tasks[t].info.task_nodes.size() > n)
                        ? QString::fromStdString(subTasks->tasks[t].info.task_nodes[n].name)
                        : QString("-");
                    table_2->setItem(t, 5 + n, new QTableWidgetItem(nodeName));
                }
            }
        } else {
            table_2->setRowCount(0);
        }
    }

    QString FmsPanel::checkMode(int i)
    {
        if (!subRobots || i < 0 || i >= static_cast<int>(subRobots->robots.size()))
            return QString("UNKNOWN");
        switch(subRobots->robots[i].mode)
        {
            case fms_msgs::msg::RobotState::MODE_IDLE: return QString("IDLE");
            case fms_msgs::msg::RobotState::MODE_MOVING_TASK: return QString("MOVING_TASK");
            case fms_msgs::msg::RobotState::MODE_TASK_PROCESSING: return QString("TASK_PROCESSING");
            case fms_msgs::msg::RobotState::MODE_MOVING: return QString("MOVING");
            case fms_msgs::msg::RobotState::MODE_MOVING_CHARGING: return QString("CHARGING");
            case fms_msgs::msg::RobotState::MODE_MOVING_WAITING: return QString("MOVING_WAITING");
            case fms_msgs::msg::RobotState::MODE_WAITING: return QString("WAITING");
            case fms_msgs::msg::RobotState::MODE_PAUSED: return QString("PAUSED");
            case fms_msgs::msg::RobotState::MODE_ERROR: return QString("ERROR");
            case fms_msgs::msg::RobotState::MODE_ISOLATED: return QString("ISOLATED");
            default: return QString("UNKNOWN");
        }
    }

    QString FmsPanel::checkType(int i)
    {
        if (!subRobots || i < 0 || i >= static_cast<int>(subRobots->robots.size()))
            return QString("UNKNOWN");
        switch(subRobots->robots[i].type)
        {
            case fms_msgs::msg::RobotState::TYPE_AGF: return QString("AGF");
            case fms_msgs::msg::RobotState::TYPE_AGV: return QString("AGV");
            case fms_msgs::msg::RobotState::TYPE_AMR: return QString("AMR");
            default: return QString("UNKNOWN");
        }
    }

    QString FmsPanel::checkState(int i)
    {
        if (!subTasks || i < 0 || i >= static_cast<int>(subTasks->tasks.size()))
            return QString("UNKNOWN");
        switch(subTasks->tasks[i].state)
        {
            case fms_msgs::msg::TaskState::STATE_QUEUED: return QString("QUEUED");
            case fms_msgs::msg::TaskState::STATE_ACTIVE: return QString("ACTIVE");
            case fms_msgs::msg::TaskState::STATE_COMPLETED: return QString("COMPLETED");
            case fms_msgs::msg::TaskState::STATE_FAILED: return QString("FAILED");
            case fms_msgs::msg::TaskState::STATE_CANCELED: return QString("CANCELED");
            case fms_msgs::msg::TaskState::STATE_PENDING: return QString("PENDING");
            default: return QString("UNKNOWN");
        }
    }

    QString FmsPanel::checkTaskType(int i)
    {
        if (!subTasks || i < 0 || i >= static_cast<int>(subTasks->tasks.size()))
            return QString("UNKNOWN");
        switch(subTasks->tasks[i].info.type)
        {
            case fms_msgs::msg::TaskInfo::TYPE_ORDERPICK: return QString("ORDERPICK");
            case fms_msgs::msg::TaskInfo::TYPE_STACKING: return QString("STACKING");
            default: return QString("UNKNOWN");
        }
    }

    void FmsPanel::onInitialize()
    {
        // tree_widget_->setModel(getDisplayContext()->getSelectionManager()->getPropertyModel());
    }
}  // namespace fms_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fms_panel::FmsPanel, rviz_common::Panel)
