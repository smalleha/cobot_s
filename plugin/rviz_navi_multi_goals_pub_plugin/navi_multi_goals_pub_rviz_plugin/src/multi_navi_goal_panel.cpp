#include <cstdio>

#include <ros/console.h>

#include <fstream>
#include <sstream>

#include <QPainter>
#include <QLineEdit>
#include <QDialog>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>
#include <QDir>
#include <tf2_msgs/TFMessage.h>
#include <QFileDialog>
#include <ros/package.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

//#include <QGridLayout>

#include "multi_navi_goal_panel.h"

namespace navi_multi_goals_pub_rviz_plugin {


    MultiNaviGoalsPanel::MultiNaviGoalsPanel(QWidget *parent)
            : rviz::Panel(parent), nh_(), maxNumGoal_(1) {

        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal_temp", 1,
                                                              boost::bind(&MultiNaviGoalsPanel::goalCntCB, this, _1));

        status_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("move_base/status", 1,
                                                                     boost::bind(&MultiNaviGoalsPanel::statusCB, this,
                                                                                 _1));

        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

        cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);


        work_dir = getDirPath();
        std::cout << work_dir << std::endl;
        /*********init************/
        mainlayout = new QHBoxLayout;
        tabWg_ = new QTabWidget;
        tab0 = new QWidget;
        tab0_horizontalLayout = new QHBoxLayout;
        tab1 = new QWidget;
        tab1_horizontalLayout= new QHBoxLayout;
        map_table = new QComboBox;
        tab0_verticalLayout = new QVBoxLayout;
        start_mapping_btn = new QPushButton("star_mapping");
        stop_mapping_btn = new QPushButton("stopmapping");
        record_map_btn = new QPushButton("record");
        file_line = new QLineEdit();
        /*********mapping************/


        /*********navigation************/
        QVBoxLayout *root_layout = new QVBoxLayout;
        // create a panel about "maxNumGoal"
        QHBoxLayout *maxNumGoal_layout = new QHBoxLayout;
        maxNumGoal_layout->addWidget(new QLabel("目标最大数量"));
        output_maxNumGoal_editor_ = new QLineEdit;
        maxNumGoal_layout->addWidget(output_maxNumGoal_editor_);
        output_maxNumGoal_button_ = new QPushButton("确定");
        maxNumGoal_layout->addWidget(output_maxNumGoal_button_);
        root_layout->addLayout(maxNumGoal_layout);

        cycle_checkbox_ = new QCheckBox("循环");
        root_layout->addWidget(cycle_checkbox_);
        // creat a QTable to contain the poseArray
        poseArray_table_ = new QTableWidget;
        initPoseTable();
        root_layout->addWidget(poseArray_table_);
        //creat a manipulate layout
        QHBoxLayout *manipulate_layout = new QHBoxLayout;
        output_reset_button_ = new QPushButton("重置");
        manipulate_layout->addWidget(output_reset_button_);
        output_cancel_button_ = new QPushButton("取消");
        manipulate_layout->addWidget(output_cancel_button_);
        output_startNavi_button_ = new QPushButton("开始导航!");
        manipulate_layout->addWidget(output_startNavi_button_);
        root_layout->addLayout(manipulate_layout);

        /**********add object***********/
        QGridLayout *qglayout = new QGridLayout;
        record_map_btn->setFixedSize(120,28);

        qglayout->addWidget(map_table,0,0,Qt::AlignTop);
        qglayout->addWidget(record_map_btn,0,2,Qt::AlignTop);
        tab0_horizontalLayout->addLayout(qglayout);
        tab0->setLayout(tab0_horizontalLayout);
        tab1->setLayout(root_layout);
        tabWg_->addTab(tab0,"mapping");
        tabWg_->addTab(tab1,"navigation");
        mainlayout->addWidget(tabWg_);
        initDialog();


        setLayout(mainlayout);
        // set a Qtimer to start a spin for subscriptions
        QTimer *output_timer = new QTimer(this);
        output_timer->start(200);

//        QString dirpath = QFileDialog::getExistingDirectory(this,"选择目录","./",QFileDialog::ShowDirsOnly);
//        qDebug() << dirpath << endl;


         retrieveMap();
         //std::cout << work_dir <<std::endl;
        // 设置信号与槽的连接
        connect(save_file_btn , SIGNAL(clicked()) , this ,  SLOT(saveFileBtnSlot()));
        connect(record_map_btn , SIGNAL(clicked()) , this ,  SLOT(recordMapping()));
        connect(map_table,SIGNAL(activated(int)),this,SLOT(mapTableSlot(int)));
        connect(output_maxNumGoal_button_, SIGNAL(clicked()), this,
                SLOT(updateMaxNumGoal()));
        connect(output_maxNumGoal_button_, SIGNAL(clicked()), this,
                SLOT(updatePoseTable()));
        connect(output_reset_button_, SIGNAL(clicked()), this, SLOT(initPoseTable()));
        connect(output_cancel_button_, SIGNAL(clicked()), this, SLOT(cancelNavi()));
        connect(output_startNavi_button_, SIGNAL(clicked()), this, SLOT(startNavi()));
        connect(cycle_checkbox_, SIGNAL(clicked(bool)), this, SLOT(checkCycle()));
        connect(output_timer, SIGNAL(timeout()), this, SLOT(startSpin()));
        connect(poseArray_table_, SIGNAL(itemChanged(QTableWidgetItem*)), this, SLOT(UpdateLine(QTableWidgetItem*)));

        //connect();
        //retrieveMap();
    }
//start mapping
    void MultiNaviGoalsPanel::startMapping()
    {

    }
//stop mapping
    void MultiNaviGoalsPanel::stopMapping()
    {

    }

//map_table's slot
    void MultiNaviGoalsPanel::mapTableSlot(int index)
    {

      if(gmapping_switch == 1)
      {
        std::string str1 = "bash -c 'killall map_server'&";
        gmapping_switch = 0;
        system(str1.c_str());
      }

      if(lidar_switch == 0)
      {
        lidar_switch = 1;
        std::string str1 = "bash -c 'source /opt/ros/kinetic/setup.bash;source "+work_dir+"devel/setup.bash; roslaunch agilexpro open_lidar.launch  '&";
        system(str1.c_str());
      }

      QString text = map_table->itemText(index);
      std::string str1 = "bash -c 'source /opt/ros/kinetic/setup.bash; rosrun map_server map_server "+work_dir+"src/agilexpro/maps/"+text.toStdString()+".yaml'&";
      system(str1.c_str());
      gmapping_switch = 1;
      str1 = "bash -c 'source /opt/ros/kinetic/setup.bash;source "+work_dir+"devel/setup.bash; roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch  '&";
      system(str1.c_str());
      rf20_switch = 1;
      str1 = "bash -c 'source /opt/ros/kinetic/setup.bash;source "+work_dir+"devel/setup.bash; roslaunch agilexpro navigation_4wd.launch '&";
      system(str1.c_str());
      move_base_switch = 1;
      amcl_switch = 1;
      bunker_base_switch = 1;

    }



    void MultiNaviGoalsPanel::clearMap()
    {
      if(gmapping_switch == 1)
      {
        std::string str1 = "bash -c 'killall map_server'&";
        system(str1.c_str());
        gmapping_switch = 0;
      }
      std::string str1 = "bash -c 'source /opt/ros/kinetic/setup.bash; rosrun map_server map_server "+work_dir+"src/agilexpro/maps/empty.yaml'&";
      system(str1.c_str());
      ros::Rate r(0.5);
      r.sleep();
      str1 = "bash -c 'killall map_server'&";
      system(str1.c_str());
      gmapping_switch = 0;
    }

//svae file
    void MultiNaviGoalsPanel::saveFileBtnSlot()
    {
      dialog->setVisible(false);
      ros::Rate rate(1);
      QString str = file_line->text();
      file_name_str = str.toStdString();
      std::string str1 = "bash -c 'source /opt/ros/kinetic/setup.bash; rosrun map_server map_saver -f "+work_dir+"src/agilexpro/maps/" +file_name_str+ "'&";
      //std::string str1 = "bash -c 'touch ~/arm/src/agilexpro/maps/" +file_name_str+ ".yaml'&";
      system(str1.c_str());
      //ROS_INFO(str1.c_str());
      rate.sleep();
      retrieveMap();
      file_line->setText("");      
      retrieveMap();
      system("bash -c 'killall slam_gmapping;killall rf2o_laser_odometry_node'&");
      rf20_switch = 0;
      gmapping_switch = 0;
    }
//stop mapping
    void MultiNaviGoalsPanel::recordMapping()
    {
      record_switch++;
      std::string str1;
      if(record_switch % 2 == 0)//stop reord map
      {
        record_map_btn->setText("record");
        svaeNameFile();
      }
      else//save map
      {
        clearMap();
        record_map_btn->setText("stop mapping!");
        if(amcl_switch == 1)
        {
          system("bash -c 'killall move_base;killall amcl;killall bunker_base_node'&");
          amcl_switch = 0; move_base_switch = 0;bunker_base_switch = 0;
        }
        if(rf20_switch == 1)
        {
          system("bash -c 'killall rf2o_laser_odometry_node'&");
          str1 = "bash -c 'source /opt/ros/kinetic/setup.bash;source "+work_dir+"devel/setup.bash; roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch  '&";
          system(str1.c_str());
          rf20_switch = 1;
        }
        else
        {
          str1 = "bash -c 'source /opt/ros/kinetic/setup.bash;source "+work_dir+"devel/setup.bash ; roslaunch agilexpro open_lidar.launch '&";
          system(str1.c_str());
          rf20_switch = 1;
        }
        str1 = " bash -c 'source /opt/ros/kinetic/setup.bash;source "+work_dir+"devel/setup.bash ; roslaunch agilexpro gmapping.launch '&";
        system(str1.c_str());
        gmapping_switch = 1;
      }
    }

//init dialog
    void MultiNaviGoalsPanel::initDialog() {
      dialog = new  QDialog();
      save_file_btn = new QPushButton();
      QVBoxLayout *dhlayout = new QVBoxLayout();
      QHBoxLayout *dvlayout = new QHBoxLayout();
      QHBoxLayout *dvlayout2 = new QHBoxLayout();
      dvlayout->addWidget(new QLabel("filename"));
      dvlayout->addWidget(file_line);
      save_file_btn->setText("save");
      dvlayout2->setAlignment(Qt::AlignRight);
      dvlayout2->addWidget(save_file_btn);
      dhlayout->addLayout(dvlayout);
      dhlayout->addLayout(dvlayout2);
      dialog->setLayout(dhlayout);

    }

    void MultiNaviGoalsPanel::svaeNameFile()
    {
      dialog->setVisible(true);
    }

// 更新maxNumGoal命名
    void MultiNaviGoalsPanel::updateMaxNumGoal() {
        setMaxNumGoal(output_maxNumGoal_editor_->text());
        ROS_INFO("maxgoal:%d",maxNumGoal_);
        InitTableItem(output_maxNumGoal_editor_->text().toInt()*3);
    }

// set up the maximum number of goals
    void MultiNaviGoalsPanel::setMaxNumGoal(const QString &new_maxNumGoal) {
        // 检查maxNumGoal是否发生改变.
        if (new_maxNumGoal != output_maxNumGoal_) {
            output_maxNumGoal_ = new_maxNumGoal;

            // 如果命名为空，不发布任何信息
            if (output_maxNumGoal_ == "") {
                nh_.setParam("maxNumGoal_", 1);
                maxNumGoal_ = 1;
            } else {
//                velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>(output_maxNumGoal_.toStdString(), 1);
                nh_.setParam("maxNumGoal_", output_maxNumGoal_.toInt());
                maxNumGoal_ = output_maxNumGoal_.toInt();
            }
            Q_EMIT configChanged();
        }
    }

    // initialize the table of pose
    void MultiNaviGoalsPanel::initPoseTable() {
        ROS_INFO("Initialize");
        curGoalIdx_ = 0, cycleCnt_ = 0;
        permit_ = false, cycle_ = false;
        count = 0;
        if(poseArray_table_->rowCount()>0){
          delete [] table_item;
        }
        poseArray_table_->clear();
        InitTableItem(maxNumGoal_*3);
        pose_array_.poses.clear();
        deleteMark();
        poseArray_table_->setRowCount(maxNumGoal_);
        poseArray_table_->setColumnCount(3);
        poseArray_table_->setEditTriggers(QAbstractItemView::CurrentChanged);
        poseArray_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        QStringList pose_header;
        pose_header << "x" << "y" << "yaw";
        poseArray_table_->setHorizontalHeaderLabels(pose_header);
        cycle_checkbox_->setCheckState(Qt::Unchecked);

    }

    // delete marks in the map
    void MultiNaviGoalsPanel::deleteMark() {
        visualization_msgs::Marker marker_delete;
        marker_delete.action = visualization_msgs::Marker::DELETEALL;
        marker_pub_.publish(marker_delete);
    }

    //update the table of pose
    void MultiNaviGoalsPanel::updatePoseTable() {
        poseArray_table_->setRowCount(maxNumGoal_);
//        pose_array_.poses.resize(maxNumGoal_);
        QStringList pose_header;
        pose_header << "x" << "y" << "yaw";
        poseArray_table_->setHorizontalHeaderLabels(pose_header);
        poseArray_table_->show();
    }

    // call back function for counting goals
    void MultiNaviGoalsPanel::goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose) {
        if (pose_array_.poses.size() < maxNumGoal_) {
            write_flag = true;
            pose_array_.poses.push_back(pose->pose);
            pose_array_.header.frame_id = pose->header.frame_id;
            writePose(pose->pose);
            markPose(pose,pose_array_.poses.size());
            write_flag = false;
        } else {
            ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
        }
    }

    // write the poses into the table
    void MultiNaviGoalsPanel::writePose(geometry_msgs::Pose pose) {

//        poseArray_table_->setItem(pose_array_.poses.size() - 1, 0,
//                                  new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
//        poseArray_table_->setItem(pose_array_.poses.size() - 1, 1,
//                                  new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
//        poseArray_table_->setItem(pose_array_.poses.size() - 1, 2,
//                                  new QTableWidgetItem(
//                                          QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));

//      table_item[0].setText(QString::number(pose.position.x, 'f', 2));
//      table_item[1].setText(QString::number(pose.position.y, 'f', 2));
//      table_item[2].setText(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2));
//      poseArray_table_->setItem(pose_array_.poses.size() - 1, 0,&table_item[0]);
//      poseArray_table_->setItem(pose_array_.poses.size() - 1, 1,&table_item[1]);
//      poseArray_table_->setItem(pose_array_.poses.size() - 1, 2,&table_item[2]);
      ROS_INFO("writepose");
      if(count == maxNumGoal_)count=0;
      int row = pose_array_.poses.size() - 1;
      ROS_INFO("count%d",count);
      for(int i=3*count;i<3*count+3;i++)
      {
        if(i%3 == 0){
          table_item[i].setText(QString::number(pose.position.x, 'f', 2));
        }
        else if (i%3 == 1) {
          table_item[i].setText(QString::number(pose.position.y, 'f', 2));
        }
        else{
          table_item[i].setText(QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2));
        }
        poseArray_table_->setItem(row,i%3,&table_item[i]);
        std::cout<<"line:"<<i%3<<std::endl;
        //ROS_INFO("row:%d",pose_array_.poses.size());
      };
      count++;

      std::cout<<"row:"<<row<<std::endl;

      std::cout<<"count:"<<count<<std::endl;

    }

    // when setting a Navi Goal, it will set a mark on the map
    void MultiNaviGoalsPanel::markPose(const geometry_msgs::PoseStamped::ConstPtr &pose,int id) {
      ROS_INFO("markpose");
        if (ros::ok()) {
            visualization_msgs::Marker arrow;
            visualization_msgs::Marker number;
            visualization_msgs::Marker circular;
            circular.header.frame_id = arrow.header.frame_id = number.header.frame_id = pose->header.frame_id;
            arrow.ns = "navi_point_arrow";
            number.ns = "navi_point_number";
            circular.ns = "navi_point_circular";
            circular.action = arrow.action = number.action = visualization_msgs::Marker::ADD;
            arrow.type = visualization_msgs::Marker::ARROW;
            number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            circular.type = visualization_msgs::Marker::SPHERE;
            circular.pose = arrow.pose = number.pose = pose->pose;
            number.pose.position.z += 0.5;
            arrow.scale.x = 0.3;
            arrow.scale.y = 0.04;
            circular.scale.x = 0.1;
            circular.scale.y = 0.1;
            circular.scale.z = 0;
            number.scale.z = 0.5;

            number.color.r = arrow.color.r = circular.color.r = 0.1f;
            number.color.g = arrow.color.g = circular.color.g = 0.1f;
            number.color.b = arrow.color.b = circular.color.b = 1.0f;
            number.color.a = arrow.color.a = circular.color.a = 1;

            circular.id = arrow.id = number.id = id;
            number.text = std::to_string(pose_array_.poses.size());
            marker_pub_.publish(arrow);
            marker_pub_.publish(number);
            marker_pub_.publish(circular);
        }
    }

    void MultiNaviGoalsPanel::RemarkPose(geometry_msgs::PoseStamped &pose,int id) {
      ROS_INFO("markpose");
        if (ros::ok()) {
//            visualization_msgs::Marker arrow;
//            visualization_msgs::Marker number;
//            arrow.header.frame_id = number.header.frame_id = "map";
//            arrow.ns = "navi_point_arrow";
//            number.ns = "navi_point_number";
//            arrow.action = number.action = visualization_msgs::Marker::ADD;
//            arrow.type = visualization_msgs::Marker::ARROW;
//            number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//            arrow.pose = number.pose = pose.pose;
//            number.pose.position.z += 1.0;
//            arrow.scale.x = 1.0;
//            arrow.scale.y = 0.2;
//            number.scale.z = 1.0;
//            arrow.color.r = number.color.r = 1.0f;
//            arrow.color.g = number.color.g = 0.98f;
//            arrow.color.b = number.color.b = 0.80f;
//            arrow.color.a = number.color.a = 1.0;
//            arrow.id = number.id = id;
//            number.text = std::to_string(id);
//            marker_pub_.publish(arrow);
//            marker_pub_.publish(number);
          visualization_msgs::Marker arrow;
          visualization_msgs::Marker number;
          visualization_msgs::Marker circular;
          circular.header.frame_id = arrow.header.frame_id = number.header.frame_id = pose.header.frame_id;
          arrow.ns = "navi_point_arrow";
          number.ns = "navi_point_number";
          circular.ns = "navi_point_circular";
          circular.action = arrow.action = number.action = visualization_msgs::Marker::ADD;
          arrow.type = visualization_msgs::Marker::ARROW;
          number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          circular.type = visualization_msgs::Marker::SPHERE;
          circular.pose = arrow.pose = number.pose = pose.pose;
          number.pose.position.z += 0.5;
          arrow.scale.x = 0.3;
          arrow.scale.y = 0.04;
          circular.scale.x = 0.1;
          circular.scale.y = 0.1;
          circular.scale.z = 0;
          number.scale.z = 0.5;

          number.color.r = arrow.color.r = circular.color.r = 0.1f;
          number.color.g = arrow.color.g = circular.color.g = 0.1f;
          number.color.b = arrow.color.b = circular.color.b = 1.0f;
          number.color.a = arrow.color.a = circular.color.a = 1;

          circular.id = arrow.id = number.id = id;
          number.text = std::to_string(id);
          marker_pub_.publish(arrow);
          marker_pub_.publish(number);
          marker_pub_.publish(circular);
        }
    }

    // check whether it is in the cycling situation
    void MultiNaviGoalsPanel::checkCycle() {
        cycle_ = cycle_checkbox_->isChecked();
    }

    // start to navigate, and only command the first goal
    void MultiNaviGoalsPanel::startNavi() {
        curGoalIdx_ = curGoalIdx_ % pose_array_.poses.size();
        if (!pose_array_.poses.empty() && curGoalIdx_ < maxNumGoal_) {
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_.header;
            goal.pose = pose_array_.poses.at(curGoalIdx_);
            goal_pub_.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);
            poseArray_table_->item(curGoalIdx_, 0)->setBackgroundColor(QColor(255, 69, 0));
            poseArray_table_->item(curGoalIdx_, 1)->setBackgroundColor(QColor(255, 69, 0));
            poseArray_table_->item(curGoalIdx_, 2)->setBackgroundColor(QColor(255, 69, 0));
            curGoalIdx_ += 1;
            permit_ = true;
        } else {
            ROS_ERROR("Something Wrong");
        }
    }

    // complete the remaining goals
    void MultiNaviGoalsPanel::completeNavi() {
        if (curGoalIdx_ < pose_array_.poses.size()) {
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_.header;
            goal.pose = pose_array_.poses.at(curGoalIdx_);
            goal_pub_.publish(goal);
            ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);
            poseArray_table_->item(curGoalIdx_, 0)->setBackgroundColor(QColor(255, 69, 0));
            poseArray_table_->item(curGoalIdx_, 1)->setBackgroundColor(QColor(255, 69, 0));
            poseArray_table_->item(curGoalIdx_, 2)->setBackgroundColor(QColor(255, 69, 0));
            curGoalIdx_ += 1;
            permit_ = true;
        } else {
            ROS_ERROR("All goals are completed");
            permit_ = false;
        }
    }

    // command the goals cyclically
    void MultiNaviGoalsPanel::cycleNavi() {
        if (permit_) {
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_.header;
            goal.pose = pose_array_.poses.at(curGoalIdx_ % pose_array_.poses.size());
            goal_pub_.publish(goal);
            ROS_INFO("Navi to the Goal%lu, in the %dth cycle", curGoalIdx_ % pose_array_.poses.size() + 1,
                     cycleCnt_ + 1);
            bool even = ((cycleCnt_ + 1) % 2 != 0);
            QColor color_table;
            if (even) color_table = QColor(255, 69, 0); else color_table = QColor(100, 149, 237);
            poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 0)->setBackgroundColor(color_table);
            poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 1)->setBackgroundColor(color_table);
            poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 2)->setBackgroundColor(color_table);
            curGoalIdx_ += 1;
            cycleCnt_ = curGoalIdx_ / pose_array_.poses.size();
        }
    }

    // cancel the current command
    void MultiNaviGoalsPanel::cancelNavi() {
        if (!cur_goalid_.id.empty()) {
            cancel_pub_.publish(cur_goalid_);
            ROS_ERROR("Navigation have been canceled");
        }
    }

    // call back for listening current state
    void MultiNaviGoalsPanel::statusCB(const actionlib_msgs::GoalStatusArray::ConstPtr &statuses) {
        bool arrived_pre = arrived_;
        arrived_ = checkGoal(statuses->status_list);
//        if (arrived_) {
//          ROS_ERROR("%d,%d", int(arrived_), int(arrived_pre));
//        }
        if (arrived_ && arrived_ != arrived_pre && ros::ok() && permit_) {
            ros::Rate r(1);
            r.sleep();
            if (cycle_) cycleNavi();
            else completeNavi();
        }
    }

    //check the current state of goal
    bool MultiNaviGoalsPanel::checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list) {
        bool done;
        if (!status_list.empty()) {
            for (auto &i : status_list) {
                if (i.status == 3) {
                    done = true;
                    ROS_INFO("completed Goal%d", curGoalIdx_);
                } else if (i.status == 4) {
                    ROS_ERROR("Goal%d is Invalid, Navi to Next Goal%d", curGoalIdx_, curGoalIdx_ + 1);
                    return true;
                } else if (i.status == 0) {
                    done = true;
                } else if (i.status == 1) {
                    cur_goalid_ = i.goal_id;
                    done = false;
                } else done = false;
            }
        } else {
            ROS_INFO("Please input the Navi Goal");
            done = false;
        }
        return done;
    }

// spin for subscribing
    void MultiNaviGoalsPanel::startSpin() {
        if (ros::ok()) {
            ros::spinOnce();
        }
    }

    void MultiNaviGoalsPanel::InitTableItem(int num)
    {
      table_item = new QTableWidgetItem[num]();
     // table_item = (QTableWidgetItem*) malloc( sizeof(QTableWidgetItem) * num );
      ROS_INFO("num:%d",num);

    }

    void MultiNaviGoalsPanel::UpdateLine(QTableWidgetItem *item)
    {
      if(write_flag == false)
      {
        ROS_INFO("**%d**",item->row());
        QTableWidgetItem *text;
        geometry_msgs::PoseArray modify_item;
        text = poseArray_table_->item(item->row(),item->column());
        int index = item->column();
        int row = item->row();
        if(text->text() != "")
        {

          switch (index) {
            case 0:{
                pose_array_.poses.at(row).position.x=text->text().toDouble();
            }break;
            case 1:{
                pose_array_.poses.at(row).position.y=text->text().toDouble();
            }break;
            case 2:{
                tf::Quaternion q;
                q.setRPY(0,0,(2*text->text().toDouble()*3.14)/360.0);
                pose_array_.poses.at(row).orientation.x=q.x();
                pose_array_.poses.at(row).orientation.y=q.y();
                pose_array_.poses.at(row).orientation.z=q.z();
                pose_array_.poses.at(row).orientation.w=q.w();
            }break;
          default:break;
          }
          DeleteCubeMark(row+1);
          geometry_msgs::PoseStamped pose_temp;
          pose_temp.header.frame_id = pose_array_.header.frame_id;
          pose_temp.pose = pose_array_.poses.at(row);
          ROS_WARN("id:%d",row+1);
          RemarkPose(pose_temp,row+1);
        }
        else
        {
          //poseArray_table_->setItem(item->row(),item->column(),item);
          ROS_WARN("cannot input null!!!");
          write_flag = true;
          if(pose_array_.poses.size()>0)
          {
            switch (index) {
              case 0:{
                  item->setText(QString::number(pose_array_.poses.at(row).position.x, 'f', 2));
              }break;
              case 1:{
                  item->setText(QString::number(pose_array_.poses.at(row).position.y, 'f', 2));
              }break;
              case 2:{
                  item->setText(QString::number(tf::getYaw(pose_array_.poses.at(row).orientation) * 180.0 / 3.14, 'f', 2));
              }break;
            default:break;
            }

            }
          }
          write_flag = false;
      }

    }

    void MultiNaviGoalsPanel::DeleteCubeMark(int id)
    {
      visualization_msgs::Marker arrow;
      visualization_msgs::Marker number;
      //arrow.header.frame_id = number.header.frame_id = pose->header.frame_id;
      arrow.ns = "navi_point_arrow";
      number.ns = "navi_point_number";
      arrow.action = number.action = visualization_msgs::Marker::DELETE;
      arrow.type = visualization_msgs::Marker::ARROW;
      number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      arrow.id = id;
      number.id = id;
      number.text = id;
      marker_pub_.publish(arrow);
      marker_pub_.publish(number);
    }

    void MultiNaviGoalsPanel::retrieveMap()
    {
      if(map_table->count() > 0)
      {
        map_table->clear();
        map_table->setCurrentIndex(2);
      }
      QString qstr =  QString(QString::fromLocal8Bit(work_dir.c_str()))+"src/agilexpro/maps"/**/;
      //std::cout << "stringtoqstring:" << qstr.toStdString() << std::endl;
      QDir dir(qstr);
      if(dir.exists())
      {
        //加载目录下所有文件，可以添加过滤
        ROS_INFO("files");
        QFileInfoList subFileList = dir.entryInfoList(QDir::Files | QDir::CaseSensitive);//过滤条件为只限文件并区分大小写

        //遍历并输出指定类型的文件名
        for (int i = 0;i < subFileList.size(); i++)
        {
            QString suffix = subFileList[i].suffix();//获取后缀名
            if (suffix.compare("yaml") == 0)//如果后缀为"yaml"
            {
                   map_table->addItem(subFileList[i].baseName());
               // cout << qPrintable(subFileList[i].baseName()) << endl;//输出文件名称
            }
         }
      }
    }

    std::string MultiNaviGoalsPanel::getDirPath()
    {
//      if((tmp = getcwd(NULL, 0)) == NULL)
//        {
//          perror("getcwd error");
//        }
//       work_dir = tmp;
//         std::cout << ros::package::getPath("agilexpro") <<std::endl;
          std::string dir;
          std::string s = ros::package::getPath("agilexpro");
          std::vector<std::string> vStr;
          boost::split( vStr, s, boost::is_any_of( "/" ), boost::token_compress_on );
          for( std::vector<std::string>::iterator it = vStr.begin(); it != vStr.end(); ++ it )
          {
            //cout << *it << endl;
            std::string temp = *it;
            if(std::strcmp(temp.c_str(),"src") == 0)break;
            //std::cout << typeid(dir).name() << std::endl;
            dir += *it + "/";
          }
          return dir;
    }

} // end namespace navi-multi-goals-pub-rviz-plugin

// 声明此类是一个rviz的插件

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(navi_multi_goals_pub_rviz_plugin::MultiNaviGoalsPanel, rviz::Panel)

