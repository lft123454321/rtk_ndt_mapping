#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <rtk_ndt_mapping/save_map.h>
#include <geometry_msgs/PoseArray.h>
#include <map>
#include <tuple>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

class RTKNDTMapping
{
public:
    bool offline_bag_mode_ = false;
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // 订阅器
    ros::Subscriber points_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber imu2_sub_;

    // 发布器
    ros::Publisher map_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher rtk_pose_array_pub_;
    ros::Publisher hybrid_pose_array_pub_;
    ros::Publisher ndt_pose_array_pub_;

    // 服务
    ros::ServiceServer save_map_srv_;
    ros::ServiceServer pub_map_srv_;

    // TF广播器
    tf::TransformBroadcaster tf_broadcaster_;

    // 点云相关
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan_;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;

    // NDT相关
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
    bool ndt_initialized_;

    // 位姿相关
    Eigen::Matrix4f current_pose_;
    Eigen::Matrix4f previous_pose_;
    bool first_scan_;
    bool rtk_fixed_;

    // 坐标系原点（第一次RTK固定解的位置）
    double origin_lat_;
    double origin_lon_;
    double origin_alt_;
    double origin_yaw_;
    bool origin_set_;
    bool imu2_get_ = false;

    // 参数
    double voxel_leaf_size_;
    double ndt_resolution_;
    double ndt_step_size_;
    double ndt_trans_epsilon_;
    int ndt_max_iterations_;
    double min_scan_range_;
    double max_scan_range_;
    double min_add_scan_shift_;
    double map_block_size_ = 100.0;
    double ndt_search_radius_ = 50.0;
    double vis_downsample_res_ = 0.2;
    double block_downsample_res_ = 0.1;

    // ROI参数
    double roi_min_x_ = -50.0, roi_max_x_ = 50.0;
    double roi_min_y_ = -50.0, roi_max_y_ = 50.0;
    double roi_min_z_ = -5.0,  roi_max_z_ = 15.0;

    // 时间相关
    ros::Time last_map_publish_time_;
    ros::Time last_scan_time_;
    double map_publish_interval_ = 0.1; // 地图发布间隔


    // IMU里程计相关
    ros::Time last_imu_time_;
    bool imu_initialized_ = false;
    double imu_roll_ = 0, imu_pitch_ = 0, imu_yaw_ = 0;
    double imu_vel_x_ = 0, imu_vel_y_ = 0, imu_vel_z_ = 0;
    double nav_roll_ = 0, nav_pitch_ = 0, nav_yaw_ = 0;

    double current_pose_imu_roll_ = 0, current_pose_imu_pitch_ = 0, current_pose_imu_yaw_ = 0;
    double current_velocity_imu_x_ = 0, current_velocity_imu_y_ = 0, current_velocity_imu_z_ = 0;
    double offset_imu_x_ = 0, offset_imu_y_ = 0, offset_imu_z_ = 0;
    double offset_imu_roll_ = 0, offset_imu_pitch_ = 0, offset_imu_yaw_ = 0;

    // 地图发布相关
    bool do_map_publish_;
    bool disable_rtk_ = false;

    geometry_msgs::PoseArray rtk_pose_array_;
    geometry_msgs::PoseArray hybrid_pose_array_;
    geometry_msgs::PoseArray ndt_pose_array_;

    std::string offline_bag_path_;

    struct BlockIndex {
        int x, y;
        bool operator<(const BlockIndex& other) const {
            return std::tie(x, y) < std::tie(other.x, other.y);
        }
    };
    std::map<BlockIndex, pcl::PointCloud<pcl::PointXYZI>::Ptr> map_blocks_;

public:
    RTKNDTMapping() : 
        nh_(),
        private_nh_("~"),
        map_cloud_(new pcl::PointCloud<pcl::PointXYZI>),
        current_scan_(new pcl::PointCloud<pcl::PointXYZI>),
        ndt_initialized_(false),
        first_scan_(true),
        rtk_fixed_(false),
        origin_set_(false)
    {
        // 初始化参数
        initializeParameters();
        
        // 初始化订阅器和发布器
        if(!offline_bag_mode_) {
            initializeSubscribers();
        }
        initializePublishers();
        initializeServices();
        
        // 初始化NDT
        initializeNDT();
        
        // 初始化点云滤波器
        initializeFilters();
        
        ROS_INFO("RTK-NDT Mapping initialized");
    }

    ~RTKNDTMapping()
    {

    }

    void runOfflineBag() {
        if (offline_bag_path_.empty()) {
            ROS_ERROR("offline_bag_path is empty!");
            return;
        }
        rosbag::Bag bag;
        bag.open(offline_bag_path_, rosbag::bagmode::Read);
        std::vector<std::string> topics = {
            "/os_cloud_node/points",
            "/imu/data",
            "/gnss",
            "/imu2"
        };
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        ros::Time bag_begin = view.getBeginTime();
        ros::Time bag_end = view.getEndTime();
        double bag_total = (bag_end - bag_begin).toSec();
        ROS_INFO("Bag total duration: %.2f seconds", bag_total);

        for (const rosbag::MessageInstance& m : view) {
            double cur_time = (m.getTime() - bag_begin).toSec();
            if (m.isType<sensor_msgs::PointCloud2>()) {
                pointsCallback(m.instantiate<sensor_msgs::PointCloud2>());
            } else if (m.isType<sensor_msgs::Imu>()) {
                if (m.getTopic() == "/imu/data")
                    imuCallback(m.instantiate<sensor_msgs::Imu>());
                else if (m.getTopic() == "/imu2")
                    imu2Callback(m.instantiate<sensor_msgs::Imu>());
            } else if (m.isType<sensor_msgs::NavSatFix>()) {
                gnssCallback(m.instantiate<sensor_msgs::NavSatFix>());
            }
            ROS_INFO_THROTTLE(1.0, "Bag progress: %.2f / %.2f seconds (%.1f%%)", cur_time, bag_total, 100.0 * cur_time / bag_total);
            ros::spinOnce();
        }
        bag.close();
        ROS_INFO("Bag parsing finished.");
    }

private:
    void initializeParameters()
    {
        private_nh_.param("voxel_leaf_size", voxel_leaf_size_, 2.0);
        private_nh_.param("ndt_resolution", ndt_resolution_, 1.0);
        private_nh_.param("ndt_step_size", ndt_step_size_, 0.1);
        private_nh_.param("ndt_trans_epsilon", ndt_trans_epsilon_, 0.01);
        private_nh_.param("ndt_max_iterations", ndt_max_iterations_, 30);
        private_nh_.param("min_scan_range", min_scan_range_, 5.0);
        private_nh_.param("max_scan_range", max_scan_range_, 200.0);
        private_nh_.param("min_add_scan_shift", min_add_scan_shift_, 1.0);
        // ROI参数
        private_nh_.param("roi_min_x", roi_min_x_, roi_min_x_);
        private_nh_.param("roi_max_x", roi_max_x_, roi_max_x_);
        private_nh_.param("roi_min_y", roi_min_y_, roi_min_y_);
        private_nh_.param("roi_max_y", roi_max_y_, roi_max_y_);
        private_nh_.param("roi_min_z", roi_min_z_, roi_min_z_);
        private_nh_.param("roi_max_z", roi_max_z_, roi_max_z_);
        private_nh_.param("disable_rtk", disable_rtk_, false);
        private_nh_.param("offline_bag_mode", offline_bag_mode_, false);
        private_nh_.param("offline_bag_path", offline_bag_path_, std::string(""));
        private_nh_.param("map_block_size", map_block_size_, 100.0);
        private_nh_.param("ndt_search_radius", ndt_search_radius_, 50.0);
        private_nh_.param("vis_downsample_res", vis_downsample_res_, 0.2);
        private_nh_.param("block_downsample_res", block_downsample_res_, 0.1);

        ROS_INFO("Parameters loaded:");
        ROS_INFO("  voxel_leaf_size: %f", voxel_leaf_size_);
        ROS_INFO("  ndt_resolution: %f", ndt_resolution_);
        ROS_INFO("  ndt_step_size: %f", ndt_step_size_);
        ROS_INFO("  ndt_trans_epsilon: %f", ndt_trans_epsilon_);
        ROS_INFO("  ndt_max_iterations: %d", ndt_max_iterations_);
        ROS_INFO("  min_scan_range: %f", min_scan_range_);
        ROS_INFO("  max_scan_range: %f", max_scan_range_);
        ROS_INFO("  min_add_scan_shift: %f", min_add_scan_shift_);
        ROS_INFO("  roi_min_x: %f, roi_max_x: %f", roi_min_x_, roi_max_x_);
        ROS_INFO("  roi_min_y: %f, roi_max_y: %f", roi_min_y_, roi_max_y_);
        ROS_INFO("  roi_min_z: %f, roi_max_z: %f", roi_min_z_, roi_max_z_);
        ROS_INFO("  disable_rtk: %d", disable_rtk_);
        ROS_INFO("  offline_bag_mode: %d", offline_bag_mode_);
        ROS_INFO("  offline_bag_path: %s", offline_bag_path_.c_str());
        ROS_INFO("  map_block_size: %.1f, ndt_search_radius: %.1f, vis_downsample_res: %.2f", map_block_size_, ndt_search_radius_, vis_downsample_res_);
        ROS_INFO("  block_downsample_res: %.2f", block_downsample_res_);
    }

    void initializeSubscribers()
    {
        points_sub_ = nh_.subscribe("/os_cloud_node/points", 1, &RTKNDTMapping::pointsCallback, this);
        imu_sub_ = nh_.subscribe("/imu/data", 100, &RTKNDTMapping::imuCallback, this);
        gnss_sub_ = nh_.subscribe("/gnss", 100, &RTKNDTMapping::gnssCallback, this);
        imu2_sub_ = nh_.subscribe("/imu2", 100, &RTKNDTMapping::imu2Callback, this);
    }

    void initializePublishers()
    {
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_map", 1);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 1);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/mapping_odom", 1);
        rtk_pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("rtk_pose_array", 1, true);
        hybrid_pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("hybrid_pose_array", 1, true);
        ndt_pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("ndt_pose_array", 1, true);
    }

    void initializeServices()
    {
        save_map_srv_ = nh_.advertiseService("/save_map", &RTKNDTMapping::saveMapCallback, this);
        pub_map_srv_ = nh_.advertiseService("/pub_map", &RTKNDTMapping::pubMapCallback, this);
    }

    void initializeNDT()
    {
        ndt_.setTransformationEpsilon(ndt_trans_epsilon_);
        ndt_.setStepSize(ndt_step_size_);
        ndt_.setResolution(ndt_resolution_);
        ndt_.setMaximumIterations(ndt_max_iterations_);
    }

    void initializeFilters()
    {
        voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    }



    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // 检查自上次添加到地图以来的位移是否超过阈值，未超过则不添加，节省CPU时间
        if(!disable_rtk_ && !first_scan_) {
            static Eigen::Vector3f last_added_translation = current_pose_.block<3, 1>(0, 3);
            Eigen::Vector3f current_translation = current_pose_.block<3, 1>(0, 3);
            double shift = (current_translation - last_added_translation).norm();
            if (shift < min_add_scan_shift_)
            {
                // ROS_INFO("distance less than min_add_scan_shift_ (%.3f < %.3f), not add to map", shift, min_add_scan_shift_);
                return;
            }
            last_added_translation = current_translation;
        }
        // 转换点云
        pcl::PointCloud<pcl::PointXYZI> input_cloud;
        pcl::fromROSMsg(*msg, input_cloud);

        // 过滤点云
        filterPointCloud(input_cloud);

        // 检查是否应该开始建图
        if (!shouldStartMapping())
        {
            return;
        }

        // 处理点云
        processPointCloud(input_cloud, msg->header.stamp);

        // 发布地图
        // publishMap();
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        if (!rtk_fixed_ && origin_set_)
        {
            static ros::Time previous_time = msg->header.stamp;
            double diff_time = (msg->header.stamp - previous_time).toSec();

            double diff_imu_roll = msg->angular_velocity.x * diff_time;
            double diff_imu_pitch = msg->angular_velocity.y * diff_time;
            double diff_imu_yaw = msg->angular_velocity.z * diff_time;

            current_pose_imu_roll_ += diff_imu_roll;
            current_pose_imu_pitch_ += diff_imu_pitch;
            current_pose_imu_yaw_ += diff_imu_yaw;

            double accX1 = msg->linear_acceleration.x;
            double accY1 = std::cos(current_pose_imu_roll_) * msg->linear_acceleration.y -
                           std::sin(current_pose_imu_roll_) * msg->linear_acceleration.z;
            double accZ1 = std::sin(current_pose_imu_roll_) * msg->linear_acceleration.y +
                           std::cos(current_pose_imu_roll_) * msg->linear_acceleration.z;

            double accX2 = std::sin(current_pose_imu_pitch_) * accZ1 + std::cos(current_pose_imu_pitch_) * accX1;
            double accY2 = accY1;
            double accZ2 = std::cos(current_pose_imu_pitch_) * accZ1 - std::sin(current_pose_imu_pitch_) * accX1;

            double accX = std::cos(current_pose_imu_yaw_) * accX2 - std::sin(current_pose_imu_yaw_) * accY2;
            double accY = std::sin(current_pose_imu_yaw_) * accX2 + std::cos(current_pose_imu_yaw_) * accY2;
            double accZ = accZ2;

            offset_imu_x_ += current_velocity_imu_x_ * diff_time + accX * diff_time * diff_time / 2.0;
            offset_imu_y_ += current_velocity_imu_y_ * diff_time + accY * diff_time * diff_time / 2.0;
            offset_imu_z_ += current_velocity_imu_z_ * diff_time + accZ * diff_time * diff_time / 2.0;

            current_velocity_imu_x_ += accX * diff_time;
            current_velocity_imu_y_ += accY * diff_time;
            current_velocity_imu_z_ += accZ * diff_time;

            offset_imu_roll_ += diff_imu_roll;
            offset_imu_pitch_ += diff_imu_pitch;
            offset_imu_yaw_ += diff_imu_yaw;

            // 更新current_pose_的平移和旋转
            current_pose_(0,3) += offset_imu_x_;
            current_pose_(1,3) += offset_imu_y_;
            current_pose_(2,3) += offset_imu_z_;

            Eigen::Matrix3f rot;
            rot = Eigen::AngleAxisf(current_pose_imu_yaw_, Eigen::Vector3f::UnitZ()) *
                  Eigen::AngleAxisf(current_pose_imu_pitch_, Eigen::Vector3f::UnitY()) *
                  Eigen::AngleAxisf(current_pose_imu_roll_, Eigen::Vector3f::UnitX());
            current_pose_.block<3,3>(0,0) = rot;

            previous_time = msg->header.stamp;

            // printPose(current_pose_, "imuCallback, current_pose_");
        }
    }

    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        bool was_fixed = rtk_fixed_;
        rtk_fixed_ = (msg->status.status == 3);

        if (rtk_fixed_) {
            if (!origin_set_) {
                setOrigin(msg);
            } else if (origin_set_) {
                // RTK固定解时，origin已设置，更新current_pose_的xyz为ENU
                geographic_msgs::GeoPoint geo_origin, geo_now;
                geo_origin.latitude = origin_lat_;
                geo_origin.longitude = origin_lon_;
                geo_origin.altitude = origin_alt_;
                geo_now.latitude = msg->latitude;
                geo_now.longitude = msg->longitude;
                geo_now.altitude = msg->altitude;

                geodesy::UTMPoint utm_origin(geo_origin);
                geodesy::UTMPoint utm_now(geo_now);

                double enu_x = utm_now.easting - utm_origin.easting;
                double enu_y = utm_now.northing - utm_origin.northing;
                double enu_z = utm_now.altitude - utm_origin.altitude;

                if(abs(enu_x) > 10000 || abs(enu_y) > 10000 || abs(enu_z) > 100) {
                    ROS_WARN("RTK out of range, enu_x=%.3f, enu_y=%.3f, enu_z=%.3f", enu_x, enu_y, enu_z);
                    return;
                }

                current_pose_(0,3) = enu_x;
                current_pose_(1,3) = enu_y;
                current_pose_(2,3) = enu_z;
                // printPose(current_pose_, "gnssCallback, current_pose_");
                // 记录RTK pose
                geometry_msgs::Pose p;
                p.position.x = enu_x;
                p.position.y = enu_y;
                p.position.z = enu_z;
                Eigen::Quaternionf q(current_pose_.block<3,3>(0,0));
                p.orientation.x = q.x();
                p.orientation.y = q.y();
                p.orientation.z = q.z();
                p.orientation.w = q.w();
                rtk_pose_array_.header.stamp = msg->header.stamp;
                rtk_pose_array_.header.frame_id = "map";
                rtk_pose_array_.poses.push_back(p);
                rtk_pose_array_pub_.publish(rtk_pose_array_);
            }
        }

        // 记录状态变化
        if (rtk_fixed_ != was_fixed)
        {
            ROS_INFO("RTK status changed: %s", rtk_fixed_ ? "FIXED" : "NOT FIXED");
        }
    }

    void imu2Callback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // 使用IMU2的姿态信息
        nav_roll_ = msg->orientation_covariance[0];
        nav_pitch_ = msg->orientation_covariance[1];
        nav_yaw_ = fmod(360 - msg->orientation_covariance[2], 360);
        imu2_get_ = true;
        // 组合导航IMU数据处理
        if (rtk_fixed_)// && origin_set_)
        {
            // 更新当前位姿的姿态部分
            updatePoseFromIMU2(nav_roll_, nav_pitch_, nav_yaw_);
        }
    }

    void filterPointCloud(pcl::PointCloud<pcl::PointXYZI>& cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        // 距离过滤
        for (const auto& point : cloud.points)
        {
            double distance = sqrt(point.x * point.x + point.y * point.y);
            if (distance >= min_scan_range_ && distance <= max_scan_range_)
            {
                if (point.x >= roi_min_x_ && point.x <= roi_max_x_ &&
                    point.y >= roi_min_y_ && point.y <= roi_max_y_ &&
                    point.z >= roi_min_z_ && point.z <= roi_max_z_)
                {
                    filtered_cloud->points.push_back(point);
                }
            }
        }
        
        // 体素滤波
        voxel_filter_.setInputCloud(filtered_cloud);
        voxel_filter_.filter(cloud);
    }

    bool shouldStartMapping()
    {
        // 原点设置完成才开始建图
        return origin_set_;
    }

    void setOrigin(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        if(!imu2_get_) {
            ROS_WARN("IMU2 not get, skip setOrigin");
            return;
        }
        origin_lat_ = msg->latitude;
        origin_lon_ = msg->longitude;
        origin_alt_ = msg->altitude;
        origin_set_ = true;
        ROS_INFO("Origin set: lat=%.6f, lon=%.6f, alt=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f",
             origin_lat_, origin_lon_, origin_alt_, nav_roll_, nav_pitch_, nav_yaw_);

        // 初始化位姿矩阵，旋转部分用imu2的roll pitch yaw(输入单位为deg)
        Eigen::AngleAxisf rollAngle(nav_roll_/180*M_PI, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(nav_pitch_/180*M_PI, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(nav_yaw_/180*M_PI, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rotation = (yawAngle * pitchAngle * rollAngle).matrix();

        current_pose_ = Eigen::Matrix4f::Identity();
        previous_pose_ = Eigen::Matrix4f::Identity();
        current_pose_.block<3, 3>(0, 0) = rotation;
        previous_pose_.block<3, 3>(0, 0) = rotation;

        // 重置IMU积分状态
        imu_initialized_ = false;
        imu_roll_ = imu_pitch_ = imu_yaw_ = 0;
        imu_vel_x_ = imu_vel_y_ = imu_vel_z_ = 0;

        printPose(current_pose_, "setOrigin, current_pose_");

    }

    void updatePoseFromIMU2(double roll, double pitch, double yaw)
    {
        // ROS_INFO("updatePoseFromIMU2, roll=%.3f, pitch=%.3f, yaw=%.3f", roll, pitch, yaw);

        // 更新当前位姿的姿态部分
        Eigen::AngleAxisf roll_angle(roll/180*M_PI, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitch_angle(pitch/180*M_PI, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yaw_angle(yaw/180*M_PI, Eigen::Vector3f::UnitZ());

        Eigen::Matrix3f rotation = (yaw_angle * pitch_angle * roll_angle).matrix();
        current_pose_.block<3, 3>(0, 0) = rotation;
        // printPose(current_pose_, "updatePoseFromIMU2, current_pose_");
    }

    void processPointCloud(const pcl::PointCloud<pcl::PointXYZI>& input_cloud, const ros::Time& timestamp)
    {
        if (disable_rtk_) rtk_fixed_ = false;
        if (first_scan_)
        {
            // 第一帧，先将input_cloud根据current_pose_变换到map坐标系再添加到地图
            pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
            pcl::transformPointCloud(input_cloud, transformed_cloud, current_pose_);
            addCloudToBlock(transformed_cloud, current_pose_);  
            do_map_publish_ = true;
            first_scan_ = false;
            ndt_initialized_ = true;
        }
        else
        {
            // 后续帧，根据RTK状态选择处理方式
            if (rtk_fixed_)
            {
                // RTK固定解，强制使用RTK位姿
                processWithRTK(input_cloud, timestamp);
            }
            else
            {
                // RTK非固定解，使用NDT
                processWithNDT(input_cloud, timestamp);
            }
        }

        // 更新位姿
        previous_pose_ = current_pose_;
        last_scan_time_ = timestamp;
        // 记录混合pose
        geometry_msgs::Pose p;
        p.position.x = current_pose_(0,3);
        p.position.y = current_pose_(1,3);
        p.position.z = current_pose_(2,3);
        Eigen::Quaternionf q(current_pose_.block<3,3>(0,0));
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();
        hybrid_pose_array_.header.stamp = timestamp;
        hybrid_pose_array_.header.frame_id = "map";
        hybrid_pose_array_.poses.push_back(p);
        hybrid_pose_array_pub_.publish(hybrid_pose_array_);
        // 发布位姿
        publishPose(timestamp);
    }

    void processWithRTK(const pcl::PointCloud<pcl::PointXYZI>& input_cloud, const ros::Time& timestamp)
    {
        printPose(current_pose_, "processWithRTK, origin current_pose_");
        // 使用RTK强制锚定位姿
        // 这里需要根据实际的RTK数据更新current_pose_的平移部分

        if (!map_blocks_.empty()) {
            Eigen::Vector3f center = current_pose_.block<3,1>(0,3);
            pcl::PointCloud<pcl::PointXYZI>::Ptr ndt_map = getNearbyBlocksCloud(center, ndt_search_radius_);
            if (ndt_map->empty()) {
                ROS_WARN("NDT target map is empty, skip NDT alignment.");
                return;
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr input_ptr(new pcl::PointCloud<pcl::PointXYZI>(input_cloud));
            ndt_.setInputSource(input_ptr);
            ndt_.setInputTarget(ndt_map);
            ROS_INFO("ndt_map size: %ld, input_cloud size: %ld", ndt_map->size(), input_cloud.size());
            pcl::PointCloud<pcl::PointXYZI> ndt_aligned_cloud;
            ndt_.align(ndt_aligned_cloud, current_pose_);
            double transform_probability = ndt_.getTransformationProbability();
            if(transform_probability > 0.7) {
                current_pose_ = ndt_.getFinalTransformation();
                ROS_INFO("NDT transform probability > 0.7, update current_pose_ using NDT");
            } else {
                ROS_INFO("NDT transform probability < 0.7, using RTK current_pose_");
            }
        }
        // 转换点云并添加到地图
        pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
        pcl::transformPointCloud(input_cloud, transformed_cloud, current_pose_);
        addCloudToBlock(transformed_cloud, current_pose_);        
    }

    void processWithNDT(const pcl::PointCloud<pcl::PointXYZI>& input_cloud, const ros::Time& timestamp)
    {
        if (!ndt_initialized_)
        {
            return;
        }

        if (!map_cloud_ || map_cloud_->empty()) {
            ROS_WARN("NDT target map_cloud_ is empty, skip NDT alignment.");
            return;
        }
        if (input_cloud.empty()) {
            ROS_WARN("Input cloud is empty, skip NDT alignment.");
            return;
        }

        // 设置NDT输入点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_ptr(new pcl::PointCloud<pcl::PointXYZI>(input_cloud));
        ndt_.setInputSource(input_ptr);

        // 执行NDT配准
        pcl::PointCloud<pcl::PointXYZI> output_cloud;
        ndt_.align(output_cloud, current_pose_);

        // 更新位姿
        current_pose_ = ndt_.getFinalTransformation();
        printPose(current_pose_, "processWithNDT, current_pose_");
        // 记录NDT pose
        geometry_msgs::Pose p;
        p.position.x = current_pose_(0,3);
        p.position.y = current_pose_(1,3);
        p.position.z = current_pose_(2,3);
        Eigen::Quaternionf q(current_pose_.block<3,3>(0,0));
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();
        ndt_pose_array_.header.stamp = timestamp;
        ndt_pose_array_.header.frame_id = "map";
        ndt_pose_array_.poses.push_back(p);
        ndt_pose_array_pub_.publish(ndt_pose_array_);

        // 检查是否需要添加到地图
        double shift = calculateShift();
        if (shift >= min_add_scan_shift_)
        {
            // 转换点云并添加到地图
            pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
            pcl::transformPointCloud(input_cloud, transformed_cloud, current_pose_);
            addCloudToBlock(transformed_cloud, current_pose_);

            // 更新NDT目标点云
            ndt_.setInputTarget(map_cloud_);
        }
    }

    double calculateShift()
    {
        Eigen::Vector3f current_translation = current_pose_.block<3, 1>(0, 3);
        Eigen::Vector3f previous_translation = previous_pose_.block<3, 1>(0, 3);
        return (current_translation - previous_translation).norm();
    }

    void publishMap()
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map = getDownsampledVisCloud();
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*filtered_map, map_msg);
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = ros::Time::now();
        map_pub_.publish(map_msg);
    // 打印各个分块的坐标范围与点数
    for (const auto& block : map_blocks_) {
        int bx = block.first.x;
        int by = block.first.y;
        double block_size = map_block_size_;
        double min_x = bx * block_size;
        double max_x = (bx + 1) * block_size;
        double min_y = by * block_size;
        double max_y = (by + 1) * block_size;
        size_t point_num = block.second->size();
        ROS_INFO("block index(x=%d, y=%d): range[x: %.2f~%.2f, y: %.2f~%.2f], point num: %zu", 
                 bx, by, min_x, max_x, min_y, max_y, point_num);
    }
    }

    void publishPose(const ros::Time& timestamp)
    {
        // 发布PoseStamped
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = timestamp;
        
        pose_msg.pose.position.x = current_pose_(0, 3);
        pose_msg.pose.position.y = current_pose_(1, 3);
        pose_msg.pose.position.z = current_pose_(2, 3);

        Eigen::Quaternionf q(current_pose_.block<3, 3>(0, 0));
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_pub_.publish(pose_msg);

        // 发布TF
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose_msg.pose.position.x, 
                                       pose_msg.pose.position.y, 
                                       pose_msg.pose.position.z));
        transform.setRotation(tf::Quaternion(pose_msg.pose.orientation.x,
                                            pose_msg.pose.orientation.y,
                                            pose_msg.pose.orientation.z,
                                            pose_msg.pose.orientation.w));
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, timestamp, "map", "base_link"));

        // 发布里程计
        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        odom_msg.header.stamp = timestamp;
        odom_msg.pose.pose = pose_msg.pose;
        odom_pub_.publish(odom_msg);


    }

    bool pubMapCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        publishMap();
        return true;
    }

    bool saveMapCallback(rtk_ndt_mapping::save_map::Request& req, rtk_ndt_mapping::save_map::Response& res)
    {
        if (map_cloud_->empty())
        {
            ROS_WARN("Map is empty, cannot save");
            return false;
        }

        float filter_res = req.resolution;
        std::string destination = req.destination;
        char buffer[80];
        std::time_t now = std::time(NULL);
        std::tm* pnow = std::localtime(&now);
        std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
        std::string filename;
        if (destination.empty())
            filename = "rtk_ndt_map_" + std::string(buffer) + ".pcd";
        else
            filename = destination + "/rtk_ndt_map_" + std::string(buffer) + ".pcd";

        pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (filter_res > 0.0f)
        {
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setLeafSize(filter_res, filter_res, filter_res);
            voxel_filter.setInputCloud(map_cloud_);
            voxel_filter.filter(*save_cloud);
            ROS_INFO("Filtered map: %lu points (resolution=%.2f)", save_cloud->size(), filter_res);
        }
        else
        {
            *save_cloud = *map_cloud_;
        }

        int result = pcl::io::savePCDFileASCII(filename, *save_cloud);
        if (result == 0)
        {
            ROS_INFO("Map saved to %s with %lu points", filename.c_str(), save_cloud->size());
            res.success = true;
        }
        else
        {
            ROS_ERROR("Failed to save map to %s", filename.c_str());
            res.success = false;
        }
        return true;
    }

    void printPose(const Eigen::Matrix4f& mat, const std::string& tag)
    {
        Eigen::Vector3f t = mat.block<3,1>(0,3);
        Eigen::Matrix3f rot = mat.block<3,3>(0,0);
        Eigen::Vector3f euler = rot.eulerAngles(2, 1, 0); // ZYX顺序，返回yaw pitch roll
        double yaw = euler[0] / M_PI * 180;
        double pitch = euler[1] / M_PI * 180;
        double roll = euler[2] / M_PI * 180;
        ROS_INFO("[%s] x: %.3f, y: %.3f, z: %.3f, roll: %.3f, pitch: %.3f, yaw: %.3f",
            tag.c_str(), t.x(), t.y(), t.z(), roll, pitch, yaw);
    }

    void setMatrixRotation(Eigen::Matrix4f &mat, double roll, double pitch, double yaw)
    {
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f rot = (yawAngle * pitchAngle * rollAngle).matrix();
        mat.block<3,3>(0,0) = rot;
    }

    BlockIndex getBlockIndex(double x, double y) {
        int bx = static_cast<int>(std::floor(x / map_block_size_));
        int by = static_cast<int>(std::floor(y / map_block_size_));
        return {bx, by};
    }
    void addCloudToBlock(const pcl::PointCloud<pcl::PointXYZI>& cloud, const Eigen::Matrix4f& pose) {
        for (const auto& pt : cloud) {
            Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4f p_map = p; // pose * p;
            BlockIndex idx = getBlockIndex(p_map.x(), p_map.y());
            if (!map_blocks_[idx]) map_blocks_[idx].reset(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointXYZI pt_map;
            pt_map.x = p_map.x(); pt_map.y = p_map.y(); pt_map.z = p_map.z(); pt_map.intensity = pt.intensity;
            map_blocks_[idx]->push_back(pt_map);
        }
        // 遍历所有block，降采样到0.1m
        for (auto& kv : map_blocks_) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setLeafSize(0.1f, 0.1f, 0.1f);
            vg.setInputCloud(kv.second);
            vg.filter(*ds);
            kv.second = ds;
        }
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr getNearbyBlocksCloud(const Eigen::Vector3f& center, double radius) {
        int block_num = 0;
        pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
        int bx0 = static_cast<int>(std::floor((center.x() - radius) / map_block_size_));
        int bx1 = static_cast<int>(std::floor((center.x() + radius) / map_block_size_));
        int by0 = static_cast<int>(std::floor((center.y() - radius) / map_block_size_));
        int by1 = static_cast<int>(std::floor((center.y() + radius) / map_block_size_));
        for (int bx = bx0; bx <= bx1; ++bx) {
            for (int by = by0; by <= by1; ++by) {
                BlockIndex idx{bx, by};
                if (map_blocks_.count(idx)) {
                    *result += *map_blocks_[idx];
                    block_num++;
                }
            }
        }
        ROS_INFO("getNearbyBlocksCloud, block_num: %d", block_num);
        return result;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr getDownsampledVisCloud() {
        pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& kv : map_blocks_) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setLeafSize(vis_downsample_res_, vis_downsample_res_, vis_downsample_res_);
            vg.setInputCloud(kv.second);
            vg.filter(*ds);
            *result += *ds;
        }
        return result;
    }
    void saveBlocks(const std::string& save_dir) {
        for (const auto& kv : map_blocks_) {
            char fname[256];
            snprintf(fname, sizeof(fname), "%s/block_%d_%d.pcd", save_dir.c_str(), kv.first.x, kv.first.y);
            // 分块保存前降采样
            pcl::PointCloud<pcl::PointXYZI>::Ptr ds(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setLeafSize(block_downsample_res_, block_downsample_res_, block_downsample_res_);
            vg.setInputCloud(kv.second);
            vg.filter(*ds);
            pcl::io::savePCDFileBinary(fname, *ds);
        }
    }
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rtk_ndt_mapping_node");
    RTKNDTMapping mapping;
    if (mapping.offline_bag_mode_) {
        mapping.runOfflineBag();
    } else {
        ros::spin();
    }
    return 0;
} 