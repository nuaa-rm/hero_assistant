/**
 * @file laserMapping.cpp
 * @brief Point-LIO 激光雷达建图节点的实现文件
 * 
 * 本文件包含 LaserMappingNode 类的实现，负责激光雷达点云处理、
 * IMU 数据处理、状态估计、地图构建和位姿发布等功能。
 * 
 * 主要功能包括：
 * - 激光雷达点云预处理和特征提取
 * - IMU 数据融合和状态预测
 * - 基于卡尔曼滤波的状态估计
 * - 增量式地图构建和更新
 * - 位姿估计和 TF 发布
 * - 多激光雷达数据同步
 * 
 * @author Robot Team
 * @date 2024
 */

#include "laserMapping.h"

/**
 * @brief LaserMappingNode 构造函数
 * 
 * 初始化激光雷达建图节点，包括：
 * - 读取配置参数
 * - 初始化点云数据结构和滤波器
 * - 设置卡尔曼滤波器参数
 * - 初始化 ROS 发布者和订阅者
 * - 设置 TF 广播器和监听器
 * - 配置多激光雷达同步
 * 
 * @param options ROS 节点选项
 */
LaserMappingNode::LaserMappingNode(const rclcpp::NodeOptions &options) : Node("laser_mapping", options)
{
    RCLCPP_INFO(this->get_logger(), "laser_mapping node created");
    readParameters();

    // 初始化点云数据结构
    feats_undistort.reset(new PointCloudXYZI());
    feats_down_body_space.reset(new PointCloudXYZI());
    init_feats_world.reset(new PointCloudXYZI());
    cloud.reset(new PointCloudXYZI());
    pcl_wait_pub.reset(new PointCloudXYZI(500000, 1));
    pcl_wait_save.reset(new PointCloudXYZI());
    p_imu = std::make_shared<ImuProcess>();
    ptr_con.reset(new PointCloudXYZI());
    if_already_read__is_we_are_blue = true;

    // 初始化路径消息
    path.header.stamp = get_ros_time(lidar_end_time);
    path.header.frame_id = map_frame;
    
    // 初始化点云选择数组和降采样滤波器
    memset(point_selected_surf, true, sizeof(point_selected_surf)); // 将point_selected_surf数组的所有元素都设置为1，即将它们全部设置为true
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min); // 降采样
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min); // 降采样
    
    // 设置激光雷达到IMU的外参
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    if (extrinsic_est_en) // false
    {
        if (!use_imu_as_input) {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        } else {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    // 初始化动力学模型相关的参数
    kf_input.init_dyn_share_modified(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_2h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    
    // 初始化卡尔曼滤波器协方差矩阵
    Eigen::Matrix<double, 24, 24> P_init = MD(24, 24)::Identity() * 0.01;
    P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
    P_init.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    kf_input.change_P(P_init); // 更新卡尔曼滤波器的协方差矩阵
    
    Eigen::Matrix<double, 30, 30> P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init_output.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;
    kf_input.change_P(P_init); // 更新卡尔曼滤波器的协方差矩阵
    kf_output.change_P(P_init_output); // 更新卡尔曼滤波器的协方差矩阵
    
    // 计算过程噪声协方差矩阵
    Q_input = process_noise_cov_input(); // 计算输入和输出的过程噪声协方差矩阵
    Q_output = process_noise_cov_output();

    // 程序结束时根据当前时间来命名保存的pcd文件
    time_t now = time(0);
    tm *ltm = localtime(&now);
    sprintf(pcd_file_name, "%d_%02d_%02d_%02d_%02d_%02d", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday,
            ltm->tm_hour, ltm->tm_min, ltm->tm_sec);

    // 初始化发布者订阅者
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());
    sub_option.callback_group = callback_group_;

    // 根据激光雷达类型创建订阅者
    if (p_pre->lidar_type == AVIA) {
        sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 200000,
            std::bind(&LaserMappingNode::livox_pcl_cbk, this, std::placeholders::_1));
    } else {
        sub_pcl_pc = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 200000,
            std::bind(&LaserMappingNode::standard_pcl_cbk, this, std::placeholders::_1));
    }
    
    // 创建IMU订阅者
    sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000,
        std::bind(&LaserMappingNode::imu_cbk, this, std::placeholders::_1));

    // 创建其他订阅者
    sub_is_we_are_blue_= this->create_subscription<std_msgs::msg::Bool>("/is_we_are_blue", 1000,
       std::bind(&LaserMappingNode::is_we_are_blue_cbk, this, std::placeholders::_1), sub_option);

    // 创建发布者
    pubLaserCloudFull = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100000);
    pubLaserCloudFull_body = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 100000);
    pubLaserCloudObstacle = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_obstacle_new", 100000);
    //pubLaserCloudEffect = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100000);
    pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100000);
    pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 100000);
    pubPath = this->create_publisher<nav_msgs::msg::Path>("/path", 100000);
    plane_pub = this->create_publisher<visualization_msgs::msg::Marker>("planner_normal", 1000);
    
    // 创建TF广播器
    odom_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // test_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // 创建TF监听器
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 创建定时器
    auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(500.0 / 100.0)); // 1ms
    timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms,
                                  std::bind(&LaserMappingNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "laser_mapping node init finished.");
    // auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));       // 1s
    // map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));//发布Lasermap

    // 初始化TF变换
    base_to_lidar_affine = Eigen::Affine3d::Identity();
    base_to_lidar = tf_buffer_->lookupTransform(point_frame_id_, odom_frame, rclcpp::Time(),
                                        rclcpp::Duration::from_seconds(0.5));

    // 声明多激光雷达同步参数
    this->declare_parameter("lidar_topic_1", "/livox/lidar_192_168_1_187");
    this->declare_parameter("lidar_topic_2", "/livox/lidar_192_168_1_104");
    this->declare_parameter("OutPut_lidar_topic", "/livox/lidar");
    this->declare_parameter("main_livox_frame_id", "livox_192_168_1_104");

    lidar_flag = 0;
    this->get_parameter("lidar_topic_1", lidar_topic_1_);
    this->get_parameter("lidar_topic_2", lidar_topic_2_);
    this->get_parameter("OutPut_lidar_topic", OutPut_lidar_topic_);
    this->get_parameter("main_livox_frame_id", main_livox_frame_id_);

    // 精确时间同步设置
    // lidar_subscriber_1.subscribe(this, lidar_topic_1_, rmw_qos_profile_default);
    // lidar_subscriber_2.subscribe(this, lidar_topic_2_, rmw_qos_profile_default);

    // lidar_sync_ = std::make_shared<exact_synchronizer_lidar>(exact_policy_lidar(10),
    //                                                          lidar_subscriber_1, lidar_subscriber_2);
    // lidar_sync_->registerCallback(std::bind(&LaserMappingNode::lidarCallback, this, _1, _2));
    // lidar_sync_->setMaxIntervalDuration(std::chrono::milliseconds(5)); // 减少到5ms
    
    // // 订阅lidar_flag
    // lidar_flag_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("lidar_flag", 10,
    //                                                                          std::bind(
    //                                                                              &LaserMappingNode::lidarFlagCallback,
    //                                                                              this, _1));

}

/**
 * @brief LaserMappingNode 析构函数
 * 
 * 在程序结束时保存当前扫描的点云数据到PCD文件
 */
LaserMappingNode::~LaserMappingNode()
{
    if (pcl_wait_save->size() > 0)
    {
        pcd_index++;
        string all_points_dir(string(string(ROOT_DIR) + "PCD/" + pcd_file_name + "_") + to_string(pcd_index) + string(".pcd"));
        pcl::PCDWriter pcd_writer;
        RCLCPP_INFO(this->get_logger(), "current scan saved to /PCD/%s", all_points_dir.c_str());
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
}

/**
 * @brief 读取配置参数
 * 
 * 从ROS参数服务器读取所有配置参数，包括：
 * - IMU和激光雷达相关参数
 * - 滤波和降采样参数
 * - 地图构建参数
 * - 坐标系和话题名称
 * - 初始位姿参数
 */
void LaserMappingNode::readParameters()
{
    p_pre.reset(new Preprocess());
    
    // 声明和读取各种参数
    this->declare_parameter<bool>("prop_at_freq_of_imu", true); // true
    this->declare_parameter<bool>("use_imu_as_input", false); // false
    this->declare_parameter<bool>("check_satu", true); // true
    this->declare_parameter<int>("init_map_size", 10); // 10
    this->declare_parameter<bool>("space_down_sample", true); // true
    this->declare_parameter<double>("mapping.satu_acc", 3.); // 3.0
    this->declare_parameter<double>("mapping.satu_gyro", 35.); // 35
    this->declare_parameter<double>("mapping.acc_norm", 1.); // 1.0
    this->declare_parameter<float>("mapping.plane_thr", 0.1); // 0.1
    this->declare_parameter<int>("point_filter_num", 1); // 1
    this->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
    this->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
    this->declare_parameter<std::string>("common.odom_topic", "/Odometry");
    this->declare_parameter<std::string>("common.map_frame", "map");
    this->declare_parameter<std::string>("common.odom_frame", "base_link");
    this->declare_parameter<std::string>("gcl.task", "C");
    this->declare_parameter<float>("gcl.angle_threshold", 0.785);
    this->declare_parameter<bool>("common.con_frame", false); // false
    this->declare_parameter<int>("common.con_frame_num", 1); // 1
    this->declare_parameter<bool>("common.cut_frame", false); // false
    this->declare_parameter<double>("common.cut_frame_time_interval", 0.1); // 0.1
    this->declare_parameter<double>("common.time_lag_imu_to_lidar", 0.); // 0.0
    this->declare_parameter<double>("filter_size_surf", 0.3); // 0.3
    this->declare_parameter<double>("filter_size_map", 0.2); // 0.2
    this->declare_parameter<double>("cube_side_length", 2000.); // 2000
    this->declare_parameter<float>("mapping.det_range", 100.f); // 100
    this->declare_parameter<double>("mapping.fov_degree", 360.); // 360
    this->declare_parameter<bool>("mapping.imu_en", true); // true
    this->declare_parameter<bool>("mapping.start_in_aggressive_motion", false); // false
    this->declare_parameter<bool>("mapping.extrinsic_est_en", false); // false
    this->declare_parameter<double>("mapping.imu_time_inte", 0.005); // 0.005
    this->declare_parameter<double>("mapping.lidar_meas_cov", 0.001); // 0.001
    this->declare_parameter<double>("mapping.acc_cov_input", 0.1); // 0.1
    this->declare_parameter<double>("mapping.vel_cov", 20.); //
    this->declare_parameter<double>("mapping.gyr_cov_input", 0.01); // 0.01
    this->declare_parameter<double>("mapping.gyr_cov_output", 1000.); // 1000
    this->declare_parameter<double>("mapping.acc_cov_output", 500.); // 500
    this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001); // 4.94982e-05 dyn
    this->declare_parameter<double>("mapping.b_acc_cov", 0.0001); // 2.30444e-05 dyn
    this->declare_parameter<double>("mapping.imu_meas_acc_cov", 0.1); // 0.0008957688 dyn
    this->declare_parameter<double>("mapping.imu_meas_omg_cov", 0.1); // 0.0011397957 dyn
    this->declare_parameter<int>("mapping.initialize_time_threshold", 1000);
    this->declare_parameter<double>("preprocess.blind", 0.5); // 0.5
    this->declare_parameter<int>("preprocess.lidar_type", 1); // 1
    this->declare_parameter<int>("preprocess.scan_line", 4); // 4
    this->declare_parameter<int>("preprocess.scan_rate", 10); //
    this->declare_parameter<int>("preprocess.timestamp_unit", 3); // 1
    this->declare_parameter<double>("mapping.match_s", 81.0); // 81
    this->declare_parameter<bool>("mapping.gravity_align", true); // true
    this->declare_parameter<vector<double> >("mapping.gravity", vector<double>()); // 0.0 0.0 -0.981
    this->declare_parameter<vector<double> >("mapping.gravity_init", vector<double>()); // 0.0 0.0 -0.981
    this->declare_parameter<vector<double> >("mapping.extrinsic_T", vector<double>()); // [ 0.04165, 0.02326, -0.0284 ]
    this->declare_parameter<vector<double> >("mapping.extrinsic_R", vector<double>());
    this->declare_parameter<bool>("odometry.publish_odometry_without_downsample", false); // false
    this->declare_parameter<bool>("publish.path_en", true); // true
    this->declare_parameter<bool>("publish.scan_publish_en", true); // true
    this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", false); // false
    this->declare_parameter<bool>("runtime_pos_log_enable", 0); // false
    this->declare_parameter<bool>("pcd_save.pcd_save_en", false); // false
    this->declare_parameter<int>("pcd_save.interval", -1); // -1
    this->declare_parameter<bool>("pcd_save.use_pcd_map_", true);
    this->declare_parameter<std::string>("pcd_save.prior_PCD_map_path_red", "");
    this->declare_parameter<std::string>("pcd_save.prior_PCD_map_path_blue", "");
    this->declare_parameter<vector<double> >("init_pos.init_trans_red", vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<vector<double> >("init_pos.initial_rot_red", vector<double>{0.0, 0.0, 0.0, 1.0});
    this->declare_parameter<vector<double> >("init_pos.init_trans_blue", vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<vector<double> >("init_pos.initial_rot_blue", vector<double>{0.0, 0.0, 0.0, 1.0});
    this->declare_parameter<bool>("init_pos.is_we_are_blue", true);
    this->declare_parameter<std::string>("point_frame_id", "livox_192_168_1_104");

    this->get_parameter_or<bool>("prop_at_freq_of_imu", prop_at_freq_of_imu, true);
    this->get_parameter_or<bool>("use_imu_as_input", use_imu_as_input, false);
    this->get_parameter_or<bool>("check_satu", check_satu, true);
    this->get_parameter_or<int>("init_map_size", init_map_size, 10);
    this->get_parameter_or<bool>("space_down_sample", space_down_sample, true);
    this->get_parameter_or<double>("mapping.satu_acc", satu_acc, 3.0);
    this->get_parameter_or<double>("mapping.satu_gyro", satu_gyro, 35.0);
    this->get_parameter_or<double>("mapping.acc_norm", acc_norm, 1.0);
    this->get_parameter_or<float>("mapping.plane_thr", plane_thr, 0.1f);
    this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 1);
    this->get_parameter_or<std::string>("common.lid_topic", lid_topic, "/livox/lidar");
    this->get_parameter_or<std::string>("common.imu_topic", imu_topic, "/livox/imu_192_168_1_104");
    this->get_parameter_or<std::string>("common.odom_topic", odom_topic, "/Odometry");
    this->get_parameter_or<bool>("common.con_frame", con_frame, false);
    this->get_parameter_or<int>("common.con_frame_num", con_frame_num, 1);
    this->get_parameter_or<bool>("common.cut_frame", cut_frame, false);
    this->get_parameter_or<double>("common.cut_frame_time_interval", cut_frame_time_interval, 0.1);
    this->get_parameter_or<double>("common.time_lag_imu_to_lidar", time_lag_imu_to_lidar, 0.0);
    this->get_parameter_or<std::string>("common.map_frame", map_frame, "map");
    this->get_parameter_or<std::string>("common.odom_frame", odom_frame, "base_link");
    this->get_parameter_or<std::string>("gcl.task", task, "C");
    RCLCPP_INFO(this->get_logger(), "gcl.task:%s", task.c_str());
    this->get_parameter_or<float>("gcl.angle_threshold", angle_threshold_, 0.785f);
    this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min, 0.3);
    this->get_parameter_or<double>("filter_size_map", filter_size_map_min, 0.2);
    this->get_parameter_or<double>("cube_side_length", cube_len, 2000.);
    this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 100.f);
    this->get_parameter_or<double>("mapping.fov_degree", fov_deg, 360.);
    this->get_parameter_or<bool>("mapping.imu_en", imu_en, true);
    this->get_parameter_or<bool>("mapping.start_in_aggressive_motion", non_station_start, false);
    this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, false);
    this->get_parameter_or<double>("mapping.imu_time_inte", imu_time_inte, 0.005);
    this->get_parameter_or<double>("mapping.lidar_meas_cov", laser_point_cov, 0.001);
    this->get_parameter_or<double>("mapping.acc_cov_input", acc_cov_input, 0.1);
    this->get_parameter_or<double>("mapping.vel_cov", vel_cov, 20.);
    this->get_parameter_or<double>("mapping.gyr_cov_input", gyr_cov_input, 0.01);
    this->get_parameter_or<double>("mapping.gyr_cov_output", gyr_cov_output, 1000.);
    this->get_parameter_or<double>("mapping.acc_cov_output", acc_cov_output, 500.);
    this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
    this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
    this->get_parameter_or<double>("mapping.imu_meas_acc_cov", imu_meas_acc_cov, 0.1);
    this->get_parameter_or<double>("mapping.imu_meas_omg_cov", imu_meas_omg_cov, 0.1);
    this->get_parameter_or<int>("mapping.initialize_time_threshold", initialize_time_threshold, 1000);
    this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 1.0);
    this->get_parameter_or<int>("preprocess.lidar_type", lidar_type, 1);
    this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 4);
    this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
    this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, 3);
    this->get_parameter_or<double>("mapping.match_s", match_s, 81.0);
    this->get_parameter_or<bool>("mapping.gravity_align", gravity_align, true);
    this->get_parameter_or<vector<double> >("mapping.gravity", gravity, vector<double>());
    this->get_parameter_or<vector<double> >("mapping.gravity_init", gravity_init, vector<double>());
    this->get_parameter_or<vector<double> >("mapping.extrinsic_T", extrinT, vector<double>());
    this->get_parameter_or<vector<double> >("mapping.extrinsic_R", extrinR, vector<double>());
    this->get_parameter_or<bool>("odometry.publish_odometry_without_downsample", publish_odometry_without_downsample,
                                 false);
    this->get_parameter_or<bool>("publish.path_en", path_en, true);
    this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
    this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, false);
    this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, false);
    this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
    RCLCPP_INFO(this->get_logger(), "***************************8当前是否在建图模式: %d", pcd_save_en);
    this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
    this->get_parameter_or<bool>("pcd_save.use_pcd_map_", use_pcd_map_, true);
    
    std::string priorPCDMapPath_red, priorPCDMapPath_blue;
    this->get_parameter_or<std::string>("pcd_save.prior_PCD_map_path_red", priorPCDMapPath_red, "");
    this->get_parameter_or<std::string>("pcd_save.prior_PCD_map_path_blue", priorPCDMapPath_blue, "");
    this->get_parameter_or<bool>("init_pos.is_we_are_blue", is_we_are_blue_, false);
    
    priorPCDMapPath = is_we_are_blue_ ? priorPCDMapPath_blue : priorPCDMapPath_red;
    RCLCPP_INFO(this->get_logger(), "priorPCDMapPath: %s (is_we_are_blue: %d)", priorPCDMapPath.c_str(), is_we_are_blue_);
    
    this->get_parameter_or<vector<double> >("init_pos.init_trans_red", initial_trans_red_,
                                            vector<double>{0.0, 0.0, 0.0});
    this->get_parameter_or<vector<double> >("init_pos.initial_rot_red", initial_rot_red_,
                                            vector<double>{0.0, 0.0, 0.0, 1.0});
    this->get_parameter_or<vector<double> >("init_pos.init_trans_blue", initial_trans_blue_,
                                            vector<double>{0.0, 0.0, 0.0});
    this->get_parameter_or<vector<double> >("init_pos.initial_rot_blue", initial_rot_blue_,
                                            vector<double>{0.0, 0.0, 0.0, 1.0});
    this->get_parameter_or<std::string>("point_frame_id", point_frame_id_, "livox_192_168_1_104");

}

void LaserMappingNode::map_publish_callback()
{
    publish_map(pubLaserCloudMap);
}

PointCloudXYZI::Ptr LaserMappingNode::loadPointcloudFromPcd(const std::string &filename){
    pcl::io::loadPCDFile(filename, cloudBlob);
    RCLCPP_INFO(this->get_logger(), "load pcd from %s", filename.c_str());
    pcl::fromPCLPointCloud2(cloudBlob, *cloud);
    pcl::toROSMsg(*cloud,pcd_map_);
    return cloud;
}
double LaserMappingNode::get_time_sec(const builtin_interfaces::msg::Time &time)
{
    return rclcpp::Time(time).seconds();
}
rclcpp::Time LaserMappingNode::get_ros_time(double timestamp)
{
    int32_t sec = std::floor(timestamp);
    auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
    uint32_t nanosec = nanosec_d;
    return rclcpp::Time(sec, nanosec);
}
void LaserMappingNode::SigHandle(int sig)
{
    flg_exit = true;
    RCLCPP_ERROR(this->get_logger(), "catch sig %d", sig);
    sig_buffer.notify_all();
}
/**
 * @brief 将点从激光雷达坐标系转换到IMU坐标系
 * 
 * 根据外参估计状态，将点从激光雷达坐标系转换到IMU坐标系。
 * 如果启用外参估计，使用卡尔曼滤波器估计的外参；否则使用预设的外参。
 * 
 * @param pi 输入点（激光雷达坐标系）
 * @param po 输出点（IMU坐标系）
 */
void LaserMappingNode::pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    if (extrinsic_est_en)
    {
        if (!use_imu_as_input)
        {
            p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
        }
        else
        {
            p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    }
    else
    {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

/**
 * @brief 收集点云缓存
 * 
 * 从增量KD树中获取被移除的点，用于内存管理
 */
void LaserMappingNode::points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}

/**
 * @brief 激光地图视场分割
 * 
 * 根据当前激光雷达位置，动态调整局部地图的范围。
 * 当机器人移动到地图边缘时，会移动局部地图窗口并删除远处的点云。
 */
void LaserMappingNode::lasermap_fov_segment()
{
    cub_needrm.shrink_to_fit();
    V3D pos_LiD;
    if (use_imu_as_input)
    {
        pos_LiD = kf_input.x_.pos + kf_input.x_.rot * Lidar_T_wrt_IMU;
    }
    else
    {
        pos_LiD = kf_output.x_.pos + kf_output.x_.rot * Lidar_T_wrt_IMU;
    }
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;
    points_cache_collect();
    if(cub_needrm.size() > 0) int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

/**
 * @brief 红蓝方标识回调函数
 * 
 * 接收红蓝方标识信息，用于确定当前队伍
 * 
 * @param msg 红蓝方标识消息
 */
void LaserMappingNode::is_we_are_blue_cbk(const std_msgs::msg::Bool::UniquePtr &msg)
{
    is_we_are_blue_ = msg->data;
    RCLCPP_INFO(get_logger(), "当前红蓝方为: %d", is_we_are_blue_);
    if_already_read__is_we_are_blue=true;
}

/**
 * @brief 标准点云回调函数
 * 
 * 处理标准格式的激光雷达点云数据，包括：
 * - 时间戳检查
 * - 点云预处理
 * - 帧分割或合并
 * - 数据缓存
 * 
 * @param msg 点云消息
 */
void LaserMappingNode::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar)
    {
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }
    last_timestamp_lidar = get_time_sec(msg->header.stamp);

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr  ptr_div(new PointCloudXYZI());
    double time_div = rclcpp::Time(msg->header.stamp).seconds();
    p_pre->process(msg, ptr);
    if (cut_frame)
    {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++)
        {
            ptr_div->push_back(ptr->points[i]);
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div > cut_frame_time_interval)
            {
                if(ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr  ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty())
        {
            lidar_buffer.push_back(ptr_div);
            time_buffer.push_back(time_div);
        }
    }
    else if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar;
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}
void LaserMappingNode::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg)
{
    // auto curr_time = std::chrono::high_resolution_clock::now();
    // auto msg_time_chrono = std::chrono::time_point<std::chrono::high_resolution_clock>(
    // std::chrono::seconds(msg->header.stamp.sec) +
    // std::chrono::nanoseconds(msg->header.stamp.nanosec));
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar)
    {
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }
    last_timestamp_lidar = get_time_sec(msg->header.stamp);
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr  ptr_div(new PointCloudXYZI());
    p_pre->process(msg, ptr); //预处理，将点云存入ptr中
    double time_div = get_time_sec(msg->header.stamp);
    std::string frame_id = msg->header.frame_id;
    if (cut_frame)//根据每个点的时间戳将一帧点云切成若干片，curvature表示每个点相对这一帧第一个点的时间戳，cut_frame_time_interval单位:s
    {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++)
        {
            ptr_div->push_back(ptr->points[i]);
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div > cut_frame_time_interval)//?为什么要先加后减一个相同的值+get_time_sec(msg->header.stamp) - time_div
            {
                if(ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr  ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                lidar_frame_id_buffer.push_back(frame_id);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty())
        {
            lidar_buffer.push_back(ptr_div);
            time_buffer.push_back(time_div);
            lidar_frame_id_buffer.push_back(frame_id);
        }
    }
    else if (con_frame)//将连续帧点云合并成一帧，curvature表示每个点相对第一帧第一个点的时间戳
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar;
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            lidar_frame_id_buffer.push_back(frame_id);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else//将每帧点云存入lidar_buffer中
    {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
        lidar_frame_id_buffer.push_back(frame_id);
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}
void LaserMappingNode::imu_cbk(const sensor_msgs::msg::Imu::UniquePtr &msg_in)
{
    rclcpp::Time time_;
    stamp_ = (float)msg_in->header.stamp.sec;
    time_ = msg_in->header.stamp;
    publish_count ++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_lag_imu_to_lidar);
    double timestamp = get_time_sec(msg->header.stamp);
    mtx_buffer.lock();
    if (timestamp < last_timestamp_imu)
    {
        RCLCPP_WARN(this->get_logger(), "imu loop back, clear deque");
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }
    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}
bool LaserMappingNode::sync_packages(MeasureGroup &meas)
{
    if (!imu_en) //不启用imu
    {
        if (!lidar_buffer.empty())
        {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            meas.lidar_frame_id = lidar_frame_id_buffer.front();
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            lidar_frame_id_buffer.pop_front();
            if(meas.lidar->points.size() < 1)
            {
                RCLCPP_ERROR(this->get_logger(), "meas.lidar->points.size() < 1: lose lidar");
                return false;
            }
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points)
            {
                if (pt.curvature > end_time)
                {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
            return true;
        }
        // RCLCPP_ERROR(get_logger(), "lidar_buffer.empty()");
        return false;
    }

    if(lidar_buffer.empty())
    {
        // RCLCPP_ERROR(get_logger(), "lidar_buffer.empty()");
        return false;
    }
    if(lidar_frame_id_buffer.empty())
    {
        RCLCPP_ERROR(get_logger(), "lidar_frame_id_buffer.empty()");
        return false;
    }
    if (imu_deque.empty())
    {
        // RCLCPP_ERROR(get_logger(), "imu_deque.empty()");
        return false;
    }
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front(); //取最新值
        if(meas.lidar->points.size() < 1)
        {
            RCLCPP_ERROR(get_logger(), "meas.lidar->points.size() < 1: lose lidar");
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            lidar_frame_id_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        meas.lidar_frame_id = lidar_frame_id_buffer.front();
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt: meas.lidar->points)
        {
            if (pt.curvature > end_time)
            {
                end_time = pt.curvature; //确定这一帧雷达数据结束时间
            }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);

        meas.lidar_last_time = lidar_end_time;
        lidar_pushed = true; //最新数据已经传入meas中
    }
    if (last_timestamp_imu < lidar_end_time)//imu时间早于lidar时间
    {
        RCLCPP_ERROR(get_logger(), "lidar_end_time < last_timestamp_imu");
        return false;
    }
    if (p_imu->imu_need_init_) //imu需要初始化
    {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        while (!imu_deque.empty() && imu_time < lidar_end_time)
        {
            imu_time = get_time_sec(imu_deque.front()->header.stamp);
            if(imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front()); //meas获取imu队列中的数据
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }
    else if(!init_map) //初始地图还未初始化
    {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        meas.imu.emplace_back(imu_last_ptr);
        while (!imu_deque.empty() && imu_time < lidar_end_time)
        {
            imu_time = get_time_sec(imu_deque.front()->header.stamp);
            if(imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front()); //操作同上
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_frame_id_buffer.pop_front();
    lidar_pushed = false;
    return true;
}
void LaserMappingNode::map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

        for(int i = 0; i < feats_down_size; i++)
        {
            if (!Nearest_Points[i].empty())
            {
                const PointVector &points_near = Nearest_Points[i];
                bool need_add = true;
                PointType downsample_result, mid_point;
                mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                    if (fabs(points_near[0].x - mid_point.x) > 0.866 * filter_size_map_min || fabs(points_near[0].y - mid_point.y) > 0.866 * filter_size_map_min || fabs(points_near[0].z - mid_point.z) > 0.866 * filter_size_map_min)
                    {
                        PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
                        continue;
                    }
                    float dist  = calc_dist<float>(feats_down_world->points[i],mid_point);
                    for (int readd_i = 0; readd_i < points_near.size(); readd_i ++)
                    {
                            if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min)
                            {
                                need_add = false;
                                break;
                            }
                    }
                    if (need_add) PointToAdd.emplace_back(feats_down_world->points[i]);
            }
            else
            {
                    PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
            }
        }
    int add_point_size = ikdtree.Add_Points(PointToAdd, true);
    // std::cout << "add_point_size: " << add_point_size << std::endl;
    int no_need_down_num_ = ikdtree.Add_Points(PointNoNeedDownsample, false);
    // std::cout << "no_need_down_num_: " << no_need_down_num_ << std::endl;
}
void LaserMappingNode::publish_init_kdtree(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes)
{
    int size_init_ikdtree = ikdtree.size();
    PointCloudXYZI::Ptr   laserCloudInit(new PointCloudXYZI(size_init_ikdtree, 1));
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    PointVector ().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
    laserCloudInit->points = ikdtree.PCL_Storage;
    pcl::toROSMsg(*laserCloudInit, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = map_frame;
    pubLaserCloudFullRes->publish(laserCloudmsg);
}
void LaserMappingNode::publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr  laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = map_frame;
        pubLaserCloudFullRes->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }
    if (pcd_save_en)
    {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }
        // std::cout << "laserCloudWorld: " << laserCloudWorld->header.frame_id << std::endl;
        // std::cout << "laserCloudWorld: " << laserCloudWorld->header.frame_id << std::endl;
        *pcl_wait_save += *laserCloudWorld;
        static int scan_wait_num = 0;
        scan_wait_num ++;
        RCLCPP_INFO(this->get_logger(), "scan_wait_num: %d", scan_wait_num);
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/" + pcd_file_name + "_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            RCLCPP_INFO(this->get_logger(), "current scan saved to /PCD/%s", all_points_dir.c_str());
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}
void LaserMappingNode::publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudIMUBody(new pcl::PointCloud<pcl::PointXYZINormal>(size, 1));
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]); //转到imu坐标系下
    }
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = point_frame_id_;
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}
void LaserMappingNode::publish_obstacle_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudObstacle_body)
{
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_down_obstacle, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = point_frame_id_;
    pubLaserCloudObstacle_body->publish(laserCloudmsg);
    //publish_count -= PUBFRAME_PERIOD;
}
void LaserMappingNode::publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    bool dense_pub_en = false;
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);//feats_down_body特征点
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    if(is_first_frame__)
    {
        *pcl_wait_pub += *cloud;
        is_first_frame__ = false;
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = map_frame;
    int point_num = laserCloudmsg.row_step / laserCloudmsg.point_step;
    // std::cout << "point_num: " << point_num << std::endl;
    double stamp_ = (float)laserCloudmsg.header.stamp.sec;
    pcd_map_.header.stamp = get_ros_time(lidar_end_time);
    pcd_map_.header.frame_id = map_frame;
    // pubLaserCloudMap->publish(pcd_map_);
    pubLaserCloudMap->publish(laserCloudmsg);
}
template<typename T>
void LaserMappingNode::set_posestamp(T& out) {
    base_to_lidar =
        tf_buffer_->lookupTransform(point_frame_id_, odom_frame, rclcpp::Time(),
                                    rclcpp::Duration::from_seconds(0.5));
    base_to_lidar_affine = tf2::transformToEigen(base_to_lidar); //云台->lidar
    Eigen::Affine3d lidar_to_map_affine = Eigen::Affine3d::Identity(); //map->主雷达
    if (!use_imu_as_input)
    {
        if (is_first_kf_)
        {
            RCLCPP_INFO(this->get_logger(), "----------first_kf----------");
            if (task == "A") //使用任意点启动
            {
                kf_output.x_.pos(0) = init_x_;
                kf_output.x_.pos(1) = init_y_;
                kf_output.x_.pos(2) = init_z_;
                RCLCPP_INFO(this->get_logger(), "use initial pos: %f, %f, %f", kf_output.x_.pos(0), kf_output.x_.pos(1),
                            kf_output.x_.pos(2));
                // 将四元数转换为旋转矩阵
                Eigen::Quaterniond q(initial_rot[3], initial_rot[0], initial_rot[1], initial_rot[2]);
                RCLCPP_INFO(this->get_logger(), "use initial rot: %f, %f, %f, %f", initial_rot[3], initial_rot[0],
                            initial_rot[1], initial_rot[2]);
                Eigen::Matrix3d rot_matrix = q.normalized().toRotationMatrix();
                // 将旋转矩阵转换为 MTK::SO3<double> 类型
                MTK::SO3<double> so3_rot(rot_matrix);
                // 将转换后的旋转矩阵赋值给 kf_output.x_.rot
                kf_output.x_.rot = so3_rot;
                Eigen::Quaterniond q_updated(so3_rot); // 使用更新后的 kf_output.x_.rot 构造四元数
                lidar_to_map_affine.translation() << kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2);
                // 应用平移
                lidar_to_map_affine.rotate(q_updated); // 应用旋转
                is_first_kf_ = false;
            }
            else if (task == "B")
            {
                kf_output.x_.pos(0) = init_x_;
                kf_output.x_.pos(1) = init_y_;
                kf_output.x_.pos(2) = init_z_;
                Eigen::Quaterniond q(kf_output.x_.rot);
                RCLCPP_INFO(this->get_logger(), "use initial pos: %f, %f, %f", kf_output.x_.pos(0), kf_output.x_.pos(1),
                            kf_output.x_.pos(2));
                RCLCPP_INFO(this->get_logger(), "use initial rot: %f, %f, %f, %f", initial_rot[3], initial_rot[0],
                            initial_rot[1], initial_rot[2]);
                lidar_to_map_affine.translation() << kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2);
                // 应用平移
                lidar_to_map_affine.rotate(q); // 应用旋转
                is_first_kf_ = false;
            }
        }
        else //不是第一次启动
        {
            Eigen::Quaterniond q(kf_output.x_.rot);
            lidar_to_map_affine.translation() << kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2); // 应用平移
            lidar_to_map_affine.rotate(q); // 应用旋转
            //保存当前位姿
            init_x_ = kf_output.x_.pos(0);
            init_y_ = kf_output.x_.pos(1);
            init_z_ = kf_output.x_.pos(2);
            initial_rot[0] = q.coeffs()[0];
            initial_rot[1] = q.coeffs()[1];
            initial_rot[2] = q.coeffs()[2];
            initial_rot[3] = q.coeffs()[3];
            // std::cout<<"init_x_: "<<init_x_<<std::endl;
            // std::cout<<"init_y_: "<<init_y_<<std::endl;
            // std::cout<<"init_z_: "<<init_z_<<std::endl;
        }
    }
    else
    {
        Eigen::Quaterniond q(kf_input.x_.rot);
        lidar_to_map_affine.translation() << kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2); // 应用平移
        lidar_to_map_affine.rotate(q); // 应用旋转
        //保存当前位姿
        init_x_ = kf_input.x_.pos(0);
        init_y_ = kf_input.x_.pos(1);
        init_z_ = kf_input.x_.pos(2);
        initial_rot[0] = q.coeffs()[0];
        initial_rot[1] = q.coeffs()[1];
        initial_rot[2] = q.coeffs()[2];
        initial_rot[3] = q.coeffs()[3];
    }
    Eigen::Affine3d base_to_map = lidar_to_map_affine * base_to_lidar_affine; //map->云台
    // std::cout << "lidar_to_map_affine: " << std::endl << lidar_to_map_affine.matrix() << std::endl;
    // std::cout << "base_to_lidar_affine: " << std::endl << base_to_lidar_affine.matrix() << std::endl;
    // std::cout << "base_to_map: " << std::endl << base_to_map.matrix() << std::endl;
    out.translation.x = base_to_map.translation().x();
    out.translation.y = base_to_map.translation().y();
    out.translation.z = base_to_map.translation().z();
    Eigen::Matrix3d rotation_matrix = base_to_map.rotation();
    Eigen::Quaterniond base_quat_result(rotation_matrix);
    // 获取四元数的各个值
    out.rotation.x = base_quat_result.x();
    out.rotation.y = base_quat_result.y();
    out.rotation.z = base_quat_result.z();
    out.rotation.w = base_quat_result.w();
}
/**
 * @brief 发布里程计信息
 * 
 * 发布机器人的位姿信息，包括：
 * - 发布里程计消息到指定话题
 * - 发布TF变换（map到odom，map到base_link）
 * - 处理云台与底盘的角度误差
 * - 发布小yaw坐标系变换
 * 
 * @param pubOdomAftMapped 里程计发布者
 * @param tf_br TF广播器
 */
void LaserMappingNode::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = odom_frame; //云台
    if (publish_odometry_without_downsample)
    {
        odomAftMapped.header.stamp = get_ros_time(time_current); //每个点都发一个tf
    }
    else
    {
        odomAftMapped.header.stamp = get_ros_time(lidar_end_time); //合并到lidar_end_time发一个tf
    }

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = rclcpp::Time(odomAftMapped.header.stamp);
    transformStamped.header.frame_id = map_frame;
    transformStamped.child_frame_id = odom_frame; //云台
    set_posestamp(transformStamped.transform); //获取map_frame->odom_frame的坐标系变换
    tf_br->sendTransform(transformStamped);

    odomAftMapped.pose.pose.position.x = transformStamped.transform.translation.x;
    odomAftMapped.pose.pose.position.y = transformStamped.transform.translation.y;
    odomAftMapped.pose.pose.position.z = transformStamped.transform.translation.z;
    odomAftMapped.pose.pose.orientation.w = transformStamped.transform.rotation.w;
    odomAftMapped.pose.pose.orientation.x = transformStamped.transform.rotation.x;
    odomAftMapped.pose.pose.orientation.y = transformStamped.transform.rotation.y;
    odomAftMapped.pose.pose.orientation.z = transformStamped.transform.rotation.z;
    pubOdomAftMapped->publish(odomAftMapped); //发布到/Odometry话题，但并没有用到

}

/**
 * @brief 发布路径信息
 * 
 * 发布机器人的运动路径（当前未实现）
 * 
 * @param pubPath 路径发布者
 */
void LaserMappingNode::publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    // set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    // msg_body_pose.header.frame_id = map_frame;
    // {
    //     path.poses.emplace_back(msg_body_pose);
    //     pubPath->publish(path);
    // }
}

/**
 * @brief 定时器回调函数
 * 
 * 主要的处理循环，包括：
 * - 同步激光雷达和IMU数据
 * - 处理第一帧激光雷达数据
 * - IMU数据预处理和状态预测
 * - 激光雷达点云处理和特征提取
 * - 状态估计和地图更新
 * - 发布位姿和点云数据
 */
void LaserMappingNode::timer_callback()
{
    // while(!if_already_read__is_we_are_blue){
    //     callback_group_executor_.spin_some();
    // }
    if (sync_packages(Measures)) //获取新一组待处理Measures
    {
        if (flg_first_scan) { //雷达第一帧
            RCLCPP_INFO(this->get_logger(), "first scan frame_id: %s", Measures.lidar_frame_id.c_str());
            //读取红蓝方位姿
            if (is_we_are_blue_) {
                init_x_ = initial_trans_blue_[0];
                init_y_ = initial_trans_blue_[1];
                init_z_ = initial_trans_blue_[2];
                initial_rot = initial_rot_blue_;
            } else {
                init_x_ = initial_trans_red_[0];
                init_y_ = initial_trans_red_[1];
                init_z_ = initial_trans_red_[2];
                initial_rot = initial_rot_red_;
            }
            first_lidar_time = Measures.lidar_beg_time;
            flg_first_scan = false;
            RCLCPP_INFO(get_logger(), "first lidar time: %lf", first_lidar_time);
        }
        if (flg_reset) {
            RCLCPP_WARN(get_logger(), "reset when rosbag play back");
            p_imu->Reset();
            flg_reset = false;
            return;
        }
        double t0, t1, t2, t3, t4, t5, match_start, solve_start;
        match_time = 0;
        solve_time = 0;
        propag_time = 0;
        update_time = 0;
        t0 = omp_get_wtime();
        if (Measures.lidar_frame_id != point_frame_id_){
            RCLCPP_WARN(this->get_logger(), "frame_id change to: %s", Measures.lidar_frame_id.c_str());

            geometry_msgs::msg::TransformStamped lidar_to_lidar = tf_buffer_->lookupTransform(
                point_frame_id_, Measures.lidar_frame_id, rclcpp::Time(), rclcpp::Duration::from_seconds(0.5));
            Eigen::Affine3d lidar_to_lidar_affine = tf2::transformToEigen(lidar_to_lidar);
            Eigen::Affine3d  lidar_to_map_affine = Eigen::Affine3d::Identity();
            lidar_to_map_affine.translation() << init_x_, init_y_, init_z_;
            lidar_to_map_affine.rotate(Eigen::Quaterniond(initial_rot[3], initial_rot[0], initial_rot[1], initial_rot[2]));

            Eigen::Affine3d new_lidar_to_map =  lidar_to_map_affine * lidar_to_lidar_affine;
            init_x_ = new_lidar_to_map.translation().x();
            init_y_ = new_lidar_to_map.translation().y();
            init_z_ = new_lidar_to_map.translation().z();
            Eigen::Matrix3d rotation_matrix = new_lidar_to_map.rotation();
            Eigen::Quaterniond new_lidar_quat_result(rotation_matrix);

            // 获取四元数的各个值
            initial_rot[0] = new_lidar_quat_result.x();
            initial_rot[1] = new_lidar_quat_result.y();
            initial_rot[2] = new_lidar_quat_result.z();
            initial_rot[3] = new_lidar_quat_result.w();

            base_to_lidar = tf_buffer_->lookupTransform(point_frame_id_, odom_frame, rclcpp::Time(),
                                          rclcpp::Duration::from_seconds(0.5));

            is_first_kf_ = true;
            last_timestamp_imu = 0.0;
            point_frame_id_ = Measures.lidar_frame_id;
            publish_odometry(pubOdomAftMapped, odom_tf_broadcaster);
        }
        p_imu->Process(Measures, feats_undistort); //确保含有imu数据，Measures.lidar-->>feats_undistort
        // std::cout << "feats_undistort: " << feats_undistort->header.frame_id << std::endl;
        if (p_imu->imu_need_init_) {
            RCLCPP_INFO(this->get_logger(), "IMU need init.");
            return;
        }
        if (imu_en) {
            if (!p_imu->gravity_align_) //初始化，重力对齐
            {
                while (Measures.lidar_beg_time > get_time_sec(imu_next.header.stamp)) { //时间戳对齐
                    imu_last = imu_next;
                    imu_next = *(imu_deque.front());
                    imu_deque.pop_front();
                }
                if (non_station_start) //是否在激烈运动中
                {
                    state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc *= -1;
                } else {
                    state_in.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
                    state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
                    state_out.acc = p_imu->mean_acc * G_m_s2 / acc_norm;
                }
                if (gravity_align) {
                    Eigen::Matrix3d rot_init;
                    p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
                    p_imu->Set_init(state_in.gravity, rot_init);
                    state_in.gravity = p_imu->gravity_;
                    state_out.gravity = p_imu->gravity_;
                    state_in.rot = rot_init;
                    state_out.rot = rot_init;
                    state_out.acc = -rot_init.transpose() * state_out.gravity;
                }
                kf_input.change_x(state_in); //TODO: 看明白
                kf_output.change_x(state_out); //输入和输出状态传递给卡尔曼滤波器
                p_imu->gravity_align_ = true;
            }
        } else {
            if (!p_imu->gravity_align_) {
                state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                if (gravity_align) {
                    Eigen::Matrix3d rot_init;
                    p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
                    p_imu->Set_init(state_in.gravity, rot_init);
                    state_out.gravity = p_imu->gravity_;
                    state_out.rot = rot_init;
                    state_out.acc = -rot_init.transpose() * state_out.gravity;
                } else {
                    state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc *= -1;
                }
                kf_output.change_x(state_out);
                p_imu->gravity_align_ = true;
            }
        }
        lasermap_fov_segment();
        // RCLCPP_INFO(this->get_logger(), "lasermap_fov_segment finished.");
        t1 = omp_get_wtime();
        if (space_down_sample) //降采样feats_undistort-->>feats_down_body
        {
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
        } else {
            feats_down_body = Measures.lidar;
            sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
        }
        // RCLCPP_INFO(this->get_logger(), "downSizeFilterSurf finished.");
        time_seq = time_compressing<int>(feats_down_body); //feats_down_body每个时间点含有的点数
        feats_down_size = feats_down_body->points.size();
        if (!init_map) //pcl2型map是否初始化成功
        {
            if (ikdtree.Root_Node == nullptr) {
                ikdtree.set_downsample_param(filter_size_map_min);
            }
            if (use_pcd_map_) { //用先验地图初始化
                PointCloudXYZI::Ptr map_cloud_(new PointCloudXYZI());
                map_cloud_ = loadPointcloudFromPcd(priorPCDMapPath);
                ikdtree.Build(map_cloud_->points);
            } else { //收集初始采集到并转至世界坐标系下的点进行初始化
                feats_down_world->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++) {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    // std::cout << "feats_down_body: " << feats_down_body->header.frame_id << std::endl;
                    // std::cout << "feats_down_body: " << feats_down_body->header.frame_id << std::endl;
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                if (init_feats_world->size() < init_map_size){
                    RCLCPP_INFO(this->get_logger(), "init_feats_world->size() < init_map_size");
                    return;//点数不够多就继续初始化
                }

                ikdtree.Build(init_feats_world->points);
            }
            init_map = true;
            publish_init_kdtree(pubLaserCloudMap);
            RCLCPP_INFO(this->get_logger(), "publish_init_kdtree(pubLaserCloudMap)");
            return;
        }
        normvec->resize(feats_down_size); //点的法向量
        feats_down_obstacle->resize(feats_down_size);
        feats_down_obstacle->clear();
        feats_down_world->resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);
        t2 = omp_get_wtime();
        crossmat_list.reserve(feats_down_size);
        pbody_list.reserve(feats_down_size);
        for (size_t i = 0; i < feats_down_body->size(); i++) //将点使用kf_output进行坐标变换,顺便计算交叉矩阵（？）crossmat。
        {
            V3D point_this(feats_down_body->points[i].x,
                           feats_down_body->points[i].y,
                           feats_down_body->points[i].z);
            pbody_list[i] = point_this;
            if (extrinsic_est_en) {
                if (!use_imu_as_input) {
                    point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
                } else {
                    point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
                }
            } else {
                point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
            }
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);
            crossmat_list[i] = point_crossmat;
        }
        if (!use_imu_as_input) //use_imu_as_input=false
        {
            effct_feat_num = 0;
            double pcl_beg_time = Measures.lidar_beg_time;
            idx = -1;
            for (k = 0; k < time_seq.size(); k++) //更新卡尔曼滤波器状态
            {
                PointType &point_body = feats_down_body->points[idx + time_seq[k]];
                time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                if (is_first_frame) {
                    if (imu_en) {
                        while (time_current > get_time_sec(imu_next.header.stamp)) {
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                        }

                        angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.
                                angular_velocity.z;
                        acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.
                                linear_acceleration.z;
                    }
                    is_first_frame = false;
                    time_update_last = time_current;
                    time_predict_last_const = time_current; //为什么不是get_time_sec(imu_last.header.stamp);
                }
                if (imu_en) {
                    bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                    while (imu_comes) //是否处理完点所在时间点之前的Imu数据
                    {
                        angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.
                                angular_velocity.z;
                        acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.
                                linear_acceleration.z;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        double dt = get_time_sec(imu_last.header.stamp) - time_predict_last_const;
                        kf_output.predict(dt, Q_output, input_in, true, false);
                        time_predict_last_const = get_time_sec(imu_last.header.stamp);
                        imu_comes = time_current > get_time_sec(imu_next.header.stamp); {
                            double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
                            if (dt_cov > 0.0) {
                                time_update_last = get_time_sec(imu_last.header.stamp);
                                double propag_imu_start = omp_get_wtime();
                                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                propag_time += omp_get_wtime() - propag_imu_start;
                                double solve_imu_start = omp_get_wtime();
                                kf_output.update_iterated_dyn_share_IMU();
                                solve_time += omp_get_wtime() - solve_imu_start;
                            }
                        }
                    }
                }
                double dt = time_current - time_predict_last_const;
                double propag_state_start = omp_get_wtime();
                if (!prop_at_freq_of_imu) {
                    double dt_cov = time_current - time_update_last;
                    if (dt_cov > 0.0) {
                        kf_output.predict(dt_cov, Q_output, input_in, false, true);
                        time_update_last = time_current;
                    }
                }
                kf_output.predict(dt, Q_output, input_in, true, false);
                propag_time += omp_get_wtime() - propag_state_start;
                time_predict_last_const = time_current;
                double t_update_start = omp_get_wtime();

                if (feats_down_size < 1) {
                    RCLCPP_INFO(this->get_logger(), "No point, skip this scan!");
                    idx += time_seq[k];
                    continue;
                }
                if (!kf_output.update_iterated_dyn_share_modified()) {
                    idx = idx + time_seq[k];
                    continue;
                }
                solve_start = omp_get_wtime();

                if (publish_odometry_without_downsample) //若为true，每个在相同时间的点，约等于每个点发一个tf
                {
                    publish_odometry(pubOdomAftMapped, odom_tf_broadcaster);
                    if (runtime_pos_log) {
                        state_out = kf_output.x_;
                        euler_cur = SO3ToEuler(state_out.rot);
                    }
                }
                for (int j = 0; j < time_seq[k]; j++) //转坐标系：feats_down_body-->>feats_down_world
                {
                    PointType &point_body_j = feats_down_body->points[idx + j + 1];
                    PointType &point_world_j = feats_down_world->points[idx + j + 1];
                    pointBodyToWorld(&point_body_j, &point_world_j);
                }
                solve_time += omp_get_wtime() - solve_start;
                update_time += omp_get_wtime() - t_update_start;
                idx += time_seq[k];
            }
        }
        /*else 用不到所以全注释了
        {
            bool imu_prop_cov = false;
            effct_feat_num = 0;

            double pcl_beg_time = Measures.lidar_beg_time;
            idx = -1;
            for (k = 0; k < time_seq.size(); k++)
            {
                PointType &point_body  = feats_down_body->points[idx+time_seq[k]];
                time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                if (is_first_frame)
                {
                    while (time_current > get_time_sec(imu_next.header.stamp))
                    {
                        imu_last = imu_next;//
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                    }
                    imu_prop_cov = true;
                    is_first_frame = false;
                    t_last = time_current;
                    time_update_last = time_current;
                    {
                        input_in.gyro<<imu_last.angular_velocity.x,
                                    imu_last.angular_velocity.y,
                                    imu_last.angular_velocity.z;
                        input_in.acc<<imu_last.linear_acceleration.x,
                                    imu_last.linear_acceleration.y,
                                    imu_last.linear_acceleration.z;
                        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                    }
                }
                while (time_current > get_time_sec(imu_next.header.stamp))
                {
                    imu_last = imu_next;
                    imu_next = *(imu_deque.front());
                    imu_deque.pop_front();
                    input_in.gyro<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                    input_in.acc <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                    input_in.acc    = input_in.acc * G_m_s2 / acc_norm;
                    double dt = get_time_sec(imu_last.header.stamp) - t_last;
                    double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
                    if (dt_cov > 0.0)
                    {
                        kf_input.predict(dt_cov, Q_input, input_in, false, true);
                        time_update_last = get_time_sec(imu_last.header.stamp);
                    }
                        kf_input.predict(dt, Q_input, input_in, true, false);
                    t_last = get_time_sec(imu_last.header.stamp);
                    imu_prop_cov = true;
                }
                double dt = time_current - t_last;
                t_last = time_current;
                double propag_start = omp_get_wtime();
                if(!prop_at_freq_of_imu)
                {
                    double dt_cov = time_current - time_update_last;
                    if (dt_cov > 0.0)
                    {
                        kf_input.predict(dt_cov, Q_input, input_in, false, true);
                        time_update_last = time_current;
                    }
                }
                kf_input.predict(dt, Q_input, input_in, true, false);
                propag_time += omp_get_wtime() - propag_start;
                double t_update_start = omp_get_wtime();
                if (feats_down_size < 1)
                {
                    std::cout<<"No point, skip this scan!\n"<<std::endl;
                    idx += time_seq[k];
                    continue;
                }
                if (!kf_input.update_iterated_dyn_share_modified())
                {
                    idx = idx+time_seq[k];
                    continue;
                }
                solve_start = omp_get_wtime();
                if (publish_odometry_without_downsample)
                {
                    publish_odometry(pubOdomAftMapped, odom_tf_broadcaster);
                    if (runtime_pos_log)
                    {
                        state_in = kf_input.x_;
                        euler_cur = SO3ToEuler(state_in.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_in.pos.transpose() << " " << state_in.vel.transpose() \
                        <<" "<<state_in.bg.transpose()<<" "<<state_in.ba.transpose()<<" "<<state_in.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;
                    }
                }
                for (int j = 0; j < time_seq[k]; j++)
                {
                    PointType &point_body_j  = feats_down_body->points[idx+j+1];
                    PointType &point_world_j = feats_down_world->points[idx+j+1];
                    pointBodyToWorld(&point_body_j, &point_world_j);
                }
                solve_time += omp_get_wtime() - solve_start;
                update_time += omp_get_wtime() - t_update_start;
                idx = idx + time_seq[k];
            }
        }*/

        if (!publish_odometry_without_downsample) //每一次处理发一次tf
        {
            publish_odometry(pubOdomAftMapped, odom_tf_broadcaster);
        }
        publish_obstacle_frame_body(pubLaserCloudObstacle);
        t3 = omp_get_wtime();
        if (use_pcd_map_) {
            if (feats_down_size > 4) {
                if (initialize_time > initialize_time_threshold) { //启动时有延迟时间
                    // std::cout << "start" <<std::endl;
                    map_incremental();
                }
                else
                {
                    initialize_time = initialize_time + 1;
                }
            }
        } else {
            if (feats_down_size > 4) {
                map_incremental();
            }
        }
        t5 = omp_get_wtime();
        if (path_en) publish_path(pubPath);
        if (scan_pub_en || pcd_save_en) publish_frame_world(pubLaserCloudFull);
        if (scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
        if (runtime_pos_log) {
            frame_num++;
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num; {
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num;
            }
            aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
            aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
            aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
            T1[time_log_counter] = Measures.lidar_beg_time;
            s_plot[time_log_counter] = t5 - t0;
            s_plot2[time_log_counter] = feats_undistort->points.size();
            s_plot3[time_log_counter] = aver_time_consu;
            time_log_counter++;
            printf(
                "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f total time: %0.6f current time: %0.6f \n",
                t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp,
                aver_time_propag,t5-t0, t5); //打印调试信息。
            if (!publish_odometry_without_downsample) {
                if (!use_imu_as_input) {
                    state_out = kf_output.x_;
                    euler_cur = SO3ToEuler(state_out.rot);
                } else {
                    state_in = kf_input.x_;
                    euler_cur = SO3ToEuler(state_in.rot);
                }
            }
        }
    }
}

/**
 * @brief 点云合并函数
 * 
 * 将两个激光雷达的点云数据合并成一个点云消息
 * 
 * @param lidar_in 输入激光雷达点云
 * @param lidar_out 输出合并后的点云
 */
void LaserMappingNode::point_merge(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_in,
                                       livox_ros_driver2::msg::CustomMsg::UniquePtr& lidar_out)
{
    for (auto& point : lidar_in->points)
    {
        //注释内容都为测试计算时间用
        // auto begin=rclcpp::Clock().now();
        // int32_t begin_second=begin.nanoseconds();

        livox_ros_driver2::msg::CustomPoint point_msg;
        // auto tf=rclcpp::Clock().now();
        // int32_t tf_second=tf.nanoseconds();
        // RCLCPP_INFO(this->get_logger(), "mat use %d",tf_second-begin_second,"s");

        point_msg.x = point.x;
        point_msg.y = point.y;
        point_msg.z = point.z;
        point_msg.reflectivity = point.reflectivity;
        point_msg.tag = point.tag;
        point_msg.line = point.line;
        point_msg.offset_time = point.offset_time;
        // auto filter=rclcpp::Clock().now();
        // int32_t filter_second=filter.nanoseconds();
        // RCLCPP_INFO(this->get_logger(), "give value use %d",filter_second-tf_second,"s");

        lidar_out->points.push_back(point_msg);
        // auto end=rclcpp::Clock().now();
        // int32_t end_second=end.nanoseconds();
        // RCLCPP_INFO(this->get_logger(), "pushback use %d",end_second-filter_second,"s");
    }
}

/**
 * @brief 多激光雷达同步回调函数
 * 
 * 处理两个激光雷达的同步数据，包括：
 * - 合并两个激光雷达的点云数据
 * - 设置输出消息的头部信息
 * - 进行点云预处理和缓存
 * 
 * @param lidar_1_pc 第一个激光雷达的点云数据
 * @param lidar_2_pc 第二个激光雷达的点云数据
 */
void LaserMappingNode::lidarCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_1_pc,
                                   const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_2_pc)
{
    if (lidar_flag == 0)
    {
        // RCLCPP_INFO(this->get_logger(), "lidar_flag is 0");
        // TODO 优化，这里是否可以只进行一次merge
        livox_ros_driver2::msg::CustomMsg::UniquePtr output_msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
        output_msg->points.reserve(lidar_1_pc->point_num + lidar_2_pc->point_num);
        point_merge(lidar_1_pc, output_msg);
        point_merge(lidar_2_pc, output_msg);

        // auto tf=rclcpp::Clock().now();
        // double tf_second=tf.seconds();
        // RCLCPP_INFO(this->get_logger(), "transform finsh use %f",tf_second-begin_second,"s");
        output_msg->header.stamp = lidar_2_pc->header.stamp;
        output_msg->header.frame_id = main_livox_frame_id_;
        output_msg->timebase = lidar_2_pc->timebase;
        output_msg->point_num = output_msg->points.size();
        output_msg->lidar_id = lidar_2_pc->lidar_id;
        output_msg->rsvd = lidar_2_pc->rsvd;

        // auto curr_time = std::chrono::high_resolution_clock::now();
        // auto msg_time_chrono = std::chrono::time_point<std::chrono::high_resolution_clock>(
        // std::chrono::seconds(output_msg->header.stamp.sec) +
        // std::chrono::nanoseconds(output_msg->header.stamp.nanosec));
        mtx_buffer.lock();
        double preprocess_start_time = omp_get_wtime();
        scan_count ++;
        if (get_time_sec(output_msg->header.stamp) < last_timestamp_lidar)
        {
            mtx_buffer.unlock();
            sig_buffer.notify_all();
            return;
        }
        last_timestamp_lidar = get_time_sec(output_msg->header.stamp);
        PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
        PointCloudXYZI::Ptr  ptr_div(new PointCloudXYZI());
        p_pre->process(output_msg, ptr); //预处理，将点云存入ptr中
        double time_div = get_time_sec(output_msg->header.stamp);
        std::string frame_id = output_msg->header.frame_id;
        if (cut_frame)//根据每个点的时间戳将一帧点云切成若干片，curvature表示每个点相对这一帧第一个点的时间戳，cut_frame_time_interval单位:s
        {
            sort(ptr->points.begin(), ptr->points.end(), time_list);

            for (int i = 0; i < ptr->size(); i++)
            {
                ptr_div->push_back(ptr->points[i]);
                if (ptr->points[i].curvature / double(1000) + get_time_sec(output_msg->header.stamp) - time_div > cut_frame_time_interval)//?为什么要先加后减一个相同的值+get_time_sec(output_msg->header.stamp) - time_div
                {
                    if(ptr_div->size() < 1) continue;
                    PointCloudXYZI::Ptr  ptr_div_i(new PointCloudXYZI());
                    *ptr_div_i = *ptr_div;
                    lidar_buffer.push_back(ptr_div_i);
                    time_buffer.push_back(time_div);
                    lidar_frame_id_buffer.push_back(frame_id);
                    time_div += ptr->points[i].curvature / double(1000);
                    ptr_div->clear();
                }
            }
            if (!ptr_div->empty())
            {
                lidar_buffer.push_back(ptr_div);
                time_buffer.push_back(time_div);
                lidar_frame_id_buffer.push_back(frame_id);
            }
        }
        else if (con_frame)//将连续帧点云合并成一帧，curvature表示每个点相对第一帧第一个点的时间戳
        {
            if (frame_ct == 0)
            {
                time_con = last_timestamp_lidar;
            }
            if (frame_ct < con_frame_num)
            {
                for (int i = 0; i < ptr->size(); i++)
                {
                    ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                    ptr_con->push_back(ptr->points[i]);
                }
                frame_ct ++;
            }
            else
            {
                PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI());
                *ptr_con_i = *ptr_con;
                double time_con_i = time_con;
                lidar_buffer.push_back(ptr_con_i);
                time_buffer.push_back(time_con_i);
                lidar_frame_id_buffer.push_back(frame_id);
                ptr_con->clear();
                frame_ct = 0;
            }
        }
        else//将每帧点云存入lidar_buffer中
        {
            lidar_buffer.emplace_back(ptr);
            time_buffer.emplace_back(get_time_sec(output_msg->header.stamp));
            lidar_frame_id_buffer.push_back(frame_id);
        }
        s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }
}

/**
 * @brief 激光雷达标志回调函数
 * 
 * 接收激光雷达标志信息，用于控制多激光雷达同步
 * 
 * @param msg 激光雷达标志消息
 */
void LaserMappingNode::lidarFlagCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "receive lidar_flag: %d", msg->data);
    lidar_flag = msg->data;
}

/**
 * @brief 主函数
 * 
 * 程序入口点，初始化ROS节点并启动激光雷达建图节点
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出码
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserMappingNode>());
    if (rclcpp::ok())
        rclcpp::shutdown();
    return 0;
}