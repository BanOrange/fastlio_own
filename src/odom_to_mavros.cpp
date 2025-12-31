#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>  // 新增：包含Odometry消息头文件
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

// 全局发布器（发布转换后的位姿给MAVROS）
ros::Publisher pub_mavros_pose;

// 自定义的坐标系旋转矩阵和平移向量（全局变量，初始化后不变）
Eigen::Matrix3d R_custom; // 3×3旋转矩阵
Eigen::Vector3d T_custom; // 3×1平移向量

//是否启用转发
bool if_use = false;

/**
 * @brief 位姿转换核心函数：将导航系位姿转换为飞控MAVROS所需位姿
 * @param nav_pose 导航系统输出的位姿
 * @return 转换后的位姿
 */
geometry_msgs::PoseStamped convertPose(const geometry_msgs::PoseStamped& nav_pose)
{
    geometry_msgs::PoseStamped mavros_pose = nav_pose; // 复制头部信息（时间戳、frame_id）

    // 1. 提取导航位姿的位置和姿态（转换为Eigen便于计算）
    Eigen::Vector3d P_nav(
        nav_pose.pose.position.x,
        nav_pose.pose.position.y,
        nav_pose.pose.position.z
    );
    Eigen::Quaterniond Q_nav(
        nav_pose.pose.orientation.w,
        nav_pose.pose.orientation.x,
        nav_pose.pose.orientation.y,
        nav_pose.pose.orientation.z
    );

    // 2. 位置转换（使用平移向量和旋转矩阵）
    Eigen::Vector3d P_mavros = R_custom * P_nav + T_custom;

    // 3. 姿态转换：旋转矩阵转四元数，左乘表示世界系旋转（与位置转换一致）
    Eigen::Quaterniond Q_custom(R_custom); 
    Eigen::Quaterniond Q_mavros = Q_custom * Q_nav; // 激光雷达到机体imu
    Q_mavros.normalize(); // 归一化四元数，避免计算误差

    // 4. 将转换后的位姿赋值给输出消息
    mavros_pose.pose.position.x = P_mavros.x();
    mavros_pose.pose.position.y = P_mavros.y();
    mavros_pose.pose.position.z = P_mavros.z();
    mavros_pose.pose.orientation.x = Q_mavros.x();
    mavros_pose.pose.orientation.y = Q_mavros.y();
    mavros_pose.pose.orientation.z = Q_mavros.z();
    mavros_pose.pose.orientation.w = Q_mavros.w();

    // 可选：修改frame_id为MAVROS默认的"map"或"world"（与飞控匹配）
    mavros_pose.header.frame_id = "world";

    return mavros_pose;
}

/**
 * @brief 回调函数：处理Odometry消息，完成「Odometry→PoseStamped→转换函数」
 * @param odom_msg 导航系统的Odometry消息（智能指针）
 */
void navOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    // 未启用转发则直接返回
    if (!if_use) {
        return;
    }

    // 步骤1：Odometry → PoseStamped（核心转换逻辑）
    geometry_msgs::PoseStamped nav_pose;
    nav_pose.header = odom_msg->header; // 复制时间戳和frame_id
    nav_pose.pose = odom_msg->pose.pose; // 复制位姿（位置+姿态）

    // 步骤2：转换PoseStamped的坐标系（调用核心函数）
    geometry_msgs::PoseStamped mavros_pose = convertPose(nav_pose);

    // 步骤3：发布给MAVROS
    pub_mavros_pose.publish(mavros_pose);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "pose_converter_node");
    ros::NodeHandle nh("~"); // 私有命名空间，便于参数配置

    // 从参数服务器读取配置（可在launch文件中设置）
    nh.param<bool>("if_use", if_use, false);

    // 订阅导航系统的位姿话题（需根据实际导航系统修改话题名，如"/fastlio/pose"）
    std::string nav_pose_topic;
    nh.param<std::string>("nav_pose_topic", nav_pose_topic, "/Odometry");
    ros::Subscriber sub_nav_pose = nh.subscribe(nav_pose_topic, 1000, navOdomCallback);


    // 读取旋转矩阵（默认值：单位矩阵）
    std::vector<double> rotation_vec(9, 0.0);
    std::vector<double> translation_vec(3, 0.0);
    nh.param("translation", translation_vec, std::vector<double>({0.0,0.0,0.0}));
    nh.param("rotation", rotation_vec, std::vector<double>({1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0}));
    T_custom << translation_vec[0], translation_vec[1], translation_vec[2];
    // 将一维数组转换为3×3矩阵（行优先）
    R_custom << rotation_vec[0], rotation_vec[1], rotation_vec[2],
                rotation_vec[3], rotation_vec[4], rotation_vec[5],
                rotation_vec[6], rotation_vec[7], rotation_vec[8];
    ROS_INFO("pose_converter_node/Loaded custom translation: [%.2f, %.2f, %.2f]", T_custom.x(), T_custom.y(), T_custom.z());
    ROS_INFO("pose_converter_node/Loaded custom rotation matrix:\n%.2f %.2f %.2f\n%.2f %.2f %.2f\n%.2f %.2f %.2f",
             R_custom(0,0), R_custom(0,1), R_custom(0,2),
             R_custom(1,0), R_custom(1,1), R_custom(1,2),
             R_custom(2,0), R_custom(2,1), R_custom(2,2));

    // 发布转换后的位姿到MAVROS的视觉位姿话题（与launch文件remap对应）
    pub_mavros_pose = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1000);

    ROS_INFO("Pose converter node started!");
    ROS_INFO("Subscribed to: %s", nav_pose_topic.c_str());
    ROS_INFO("Publishing to: /mavros/vision_pose/pose");

    // 自旋
    ros::spin();

    return 0;
}