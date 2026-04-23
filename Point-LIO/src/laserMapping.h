#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <iostream>

#include"rclcpp/rclcpp.hpp"

#include "IMU_Processing.hpp"
#include "parameters.h"
#include "Estimator.h"

#include "visualization_msgs/msg/marker.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_eigen/tf2_eigen.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "livox_ros_driver2/msg/custom_msg.h"

#include <std_msgs/msg/bool.hpp>

#include "rm_interfaces/msg/self_position.hpp"
// #include "rm_interfaces/msg/angle_error.hpp"

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include"livox_ros_driver2/msg/custom_msg.hpp"
#include "std_msgs/msg/int32.hpp"



#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

using std::placeholders::_1;
using std::placeholders::_2;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

class LaserMappingNode : public rclcpp::Node
{
public:
    /**
    * @brief 构造函数
    */
    explicit LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
    * @brief 析构函数
    */
    ~LaserMappingNode();

    /**
    * @brief 抛出异常
    */
    void SigHandle(int sig);

private:
    /**
    * @brief 从yaml文件中读取要用到的ros参数
    */
    void readParameters();

    /**
    * @brief 加载先验地图
    * @param filename 先验地图.pcd文件的路径
    */
    PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string &filename);

    /**
    * @brief 从ros时间戳中获取秒
    */
    double get_time_sec(const builtin_interfaces::msg::Time &time);

    /**
    * @brief 将当前时间转换为ros时间戳
    */
    rclcpp::Time get_ros_time(double timestamp);

    /**
    * @brief 将点云从雷达坐标系根据外参Lidar_T/R_wrt_IMU转换到imu坐标系
    */
    void pointBodyLidarToIMU(PointType const * const pi, PointType * const po);

    /**
    * @brief 获取ikdtree中已经被删除的点
    */
    void points_cache_collect();

    /**
    * @brief 将距离当前雷达远的之前录入的点云从ikdtree中删除
    *
    */
    void lasermap_fov_segment();

    void is_we_are_blue_cbk(const std_msgs::msg::Bool::UniquePtr &msg);

    /**
    * @brief 根据选项，将点云存入lidar_buffer中。可选1：将一帧点云按时间分开
    * @brief 2：将连续帧点云存入lidar_buffer中
    * @brief 3：将每帧点云存入lidar_buffer中
    *
    * @param msg
    */
    void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);

    /**
    * @brief 根据选项，将点云存入lidar_buffer中。可选1：将一帧点云按时间分开
    * @brief 2：将连续帧点云存入lidar_buffer中
    * @brief 3：将每帧点云存入lidar_buffer中
    *
    * @param msg
    */
    void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg);

    /**
    * @brief 将imu数据加入队列
    *
    * @param msg_in
    */
    void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr &msg_in);

    /**
    * @brief 将lidar_buffer中的最早一份点云传给Measures，并根据条件初始化Measures中的imu
    *
    * @param meas
    * @return true 更新成功
    * @return false 更新失败,可能是lidar_buffer为空,可能imu数据还没更新
    */
    bool sync_packages(MeasureGroup &meas);

    /**
     * @brief ikdtree地图增量更新（feats_down_world点云）
     *
    */
    void map_incremental();

    /**
     * @brief 发布初始ikdtree地图
     *
     * @param pubLaserCloudFullRes Publisher
    */
    void publish_init_kdtree(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes);

    /**
     * @brief 发布这一帧下采样后的世界坐标系点云
     *
     * @param pubLaserCloudFullRes Publisher
    */
    void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes);

    /**
    * @brief 发布这一帧未处理过的原始车体坐标系点云
    *
    * @param pubLaserCloudFull_body Publisher
    */
    void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body);

    /**
    * @brief pubLaserCloudObstacle_body发布这一帧的障碍物点云
    *
    * @param pubLaserCloudObstacle_body Publisher
    */
    void publish_obstacle_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudObstacle_body);

    /**
    * @brief 将这一帧的车体坐标系点云加到之前的所有点云中，发布所有点云的叠加
    *
    * @param pubLaserCloudMap Publisher
    */
    void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap);

    /**
    * @brief 根据当前的Kf_output更新当前位姿
    * @brief 具体过程为先查询当前云台系(odom_frame)到雷达坐标系(point_frame_id_)的静态变换；
    * @brief 如果是启动后第一次处理就根据task不同选择是否任意位置启动，是用A，将yaml文件中设定好的tf导入并赋给kf_output，不是用B；
    * @brief 如果不是第一次启动，直接从kf_output中获得map到lidar的坐标系变换；
    * @brief 最后计算map到云台的坐标系变换，赋给out
    *
    * @tparam T
    * @param out
    */
    template<typename T>
    void set_posestamp(T & out);

    /**
    * @brief 发布set_posestamp函数更新后的map到robot_frame坐标系变换，可选择是否跟头
    *
    * @param pubOdomAftMapped
    * @param tf_br
    */
    void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
        std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br);
    /**
    * @brief pubPath
    *
    *
    */
    void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);
    /**
    * @brief pubLaserCloudObstacle_body发布这一帧的障碍物点云
    *
    */
    void map_publish_callback();
    /**
    * @brief 计时器回调函数，控制程序以一定的频率运行
    */
    void timer_callback();



    /**
     * @brief 将输入点云融合到输出点云中
     * @param lidar_in 输入点云
     * @param lidar_out 输出点云
     */
    void point_merge(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_in,
                     livox_ros_driver2::msg::CustomMsg::UniquePtr& lidar_out);

    /**
     * @brief 只负责在两个雷达都正常工作时将副雷达的点云融合到主雷达点云中
     *
     */
    void lidarCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_1_pc,
                       const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& lidar_2_pc);

    /**
     * @brief 只负责在两个雷达都正常工作时将点云数据进行精确时间同步和融合
     *
     */
    void lidarFlagCallback(const std_msgs::msg::Int32::SharedPtr msg);

    /**
     * @brief 角度误差回调函数
     *
     */
    // void angle_error_callback(const rm_interfaces::msg::AngleError::ConstPtr &msg);

    //初始化发布者和订阅者
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::SubscriptionOptions sub_option;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc; //订阅PointCloud2类型的点云消息
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_; //订阅CustomMsg类型的点云消息
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu; //订阅imu消息
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_is_we_are_blue_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull; //发布下采样后世界坐标系下的点云 /cloud_registered
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body; //发布这一帧机体坐标系下的点云 /cloud_registered_body
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudObstacle; //发布这一帧的障碍物点云 /cloud_obstacle_new
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap; //发布从ikdtree中获取的初始地图点云 /Laser_map
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped; //发布odomAftMapped到odom_topic
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_pub;
    rclcpp::Publisher<rm_interfaces::msg::SelfPosition>::SharedPtr selfposition_pub;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    // std::unique_ptr<tf2_ros::TransformBroadcaster> test_tf_broadcaster;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster;
    
    // std::unique_ptr<tf2_ros::TransformBroadcaster> static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;
    std::time_t startTime, endTime;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::Matrix<double, 24, 24> Q_input;
    Eigen::Matrix<double, 30, 30> Q_output;
    

    bool is_we_are_blue_;
    const float MOV_THRESHOLD = 1.5f;
    mutex mtx_buffer;
    condition_variable sig_buffer;
    int feats_down_size = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
    int frame_ct = 0;
    double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;
    shared_ptr<ImuProcess> p_imu;
    bool init_map = false, flg_first_scan = true;
    PointCloudXYZI::Ptr  ptr_con;
    double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
    double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;
    bool   lidar_pushed = false, flg_reset = false, flg_exit = false;
    vector<BoxPointType> cub_needrm;

    deque<std::string>          lidar_frame_id_buffer;
    deque<PointCloudXYZI::Ptr>  lidar_buffer;
    deque<double>               time_buffer;
    deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;
    PointCloudXYZI::Ptr feats_undistort; //从meas中直接获取的最原始点云NO.1
    PointCloudXYZI::Ptr feats_down_body_space; //
    PointCloudXYZI::Ptr init_feats_world; //不断收集feats_down_world用于初始化地图（ikdtree）的点，世界系 NO.2
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;
    V3D euler_cur;
    MeasureGroup Measures;  //雷达传回的各种最原始的数据
    sensor_msgs::msg::Imu imu_last, imu_next;
    sensor_msgs::msg::Imu::SharedPtr imu_last_ptr;
    nav_msgs::msg::Path path;
    nav_msgs::msg::Odometry odomAftMapped;
    geometry_msgs::msg::PoseStamped msg_body_pose;
    double stamp_;

    std::string point_frame_id_;
    sensor_msgs::msg::PointCloud2 pcd_map_;
    PointCloudXYZI::Ptr cloud;
    pcl::PCLPointCloud2 cloudBlob;
    int points_cache_size = 0;
    BoxPointType LocalMap_Points;
    bool Localmap_Initialized = false;
    int process_increments = 0;
    PointCloudXYZI::Ptr pcl_wait_pub; //所有点云相加（可选是否下采样）
    PointCloudXYZI::Ptr pcl_wait_save;
    bool is_first_frame__ = true;
    bool is_first_kf_ = true;
    int initialize_time = 0;
    bool if_already_read__is_we_are_blue;
    bool if_divide_gimbal_and_chassic;
    int initialize_time_threshold;

    geometry_msgs::msg::TransformStamped base_to_lidar;
    Eigen::Affine3d base_to_lidar_affine;


    int lidar_flag;
    std::vector<livox_ros_driver2::msg::CustomMsg::ConstPtr> livox_data;
    std::string lidar_topic_1_;
    std::string lidar_topic_2_;
    std::string OutPut_lidar_topic_;
    std::string main_livox_frame_id_;

    typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver2::msg::CustomMsg,
                                                            livox_ros_driver2::msg::CustomMsg> exact_policy_lidar;
    typedef message_filters::Synchronizer<exact_policy_lidar> exact_synchronizer_lidar;

    message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar_subscriber_1;
    message_filters::Subscriber<livox_ros_driver2::msg::CustomMsg> lidar_subscriber_2;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lidar_flag_subscriber_;

    std::shared_ptr<exact_synchronizer_lidar> lidar_sync_;
};