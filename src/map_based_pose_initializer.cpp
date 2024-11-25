#include "map_based_pose_initializer/map_based_pose_initializer.hpp"

namespace map_based_pose_initializer
{
MapBasedPoseInitializer::MapBasedPoseInitializer(const rclcpp::NodeOptions & node_options) : Node("map_based_pose_initializer", node_options)
{
    param_.enable_status_debug = this->declare_parameter("enable_status_debug", false);
    param_.centerline_resolution = this->declare_parameter("centerline_resolution", 1);
    param_.localization_score_threshold = this->declare_parameter("localization_score_threshold", 0.2);
    param_.localization_accuracy_count_threshold = this->declare_parameter("localization_accuracy_count_threshold", 20);

    pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>("/sensing/gnss/pose_with_covariance", rclcpp::QoS(1));

    sub_pose_ = this->create_subscription<PoseStamped>(
        "/localization/pose_twist_fusion_filter/pose", rclcpp::QoS(1),
            std::bind(&MapBasedPoseInitializer::poseCallback, this, std::placeholders::_1));
    sub_localization_accuracy_ = this->create_subscription<LocalizationAccuracy>(
        "/localization_accuracy", rclcpp::QoS(1),
            std::bind(&MapBasedPoseInitializer::localizationAccuracyCallback, this, std::placeholders::_1));
    sub_lanelet_map_ = this->create_subscription<HADMapBin>(
        "/map/vector_map", rclcpp::QoS(1),
            std::bind(&MapBasedPoseInitializer::laneletCallback, this, std::placeholders::_1));

    cli_init_pose_ = this->create_client<InitializeLocalization>("/localization/initialize");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MapBasedPoseInitializer::run, this));

    publishGNSSPose();
}

void MapBasedPoseInitializer::publishGNSSPose()
{
    PoseWithCovarianceStamped pose_msg;

    pose_msg.pose.pose.position.x = 0.0;
    pose_msg.pose.pose.position.y = 0.0;
    pose_msg.pose.pose.position.z = 0.0;
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;
    pose_msg.pose.pose.orientation.w = 1.0;

    pose_pub_->publish(pose_msg);
}

void MapBasedPoseInitializer::poseCallback(const PoseStamped::ConstSharedPtr pose_msg)
{
    rclcpp::Time pose_time(pose_msg->header.stamp.sec, pose_msg->header.stamp.nanosec);
    pose_time_ = pose_time;
}

void MapBasedPoseInitializer::localizationAccuracyCallback(const LocalizationAccuracy::ConstSharedPtr localization_accuracy_msg)
{
    status_.localization_accuracy = *localization_accuracy_msg;
}

void MapBasedPoseInitializer::laneletCallback(const HADMapBin::ConstSharedPtr lanelet_msg)
{
    /** 
     * @brief generate centerpoint from lanelet2 map
     */
    lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(*lanelet_msg, lanelet_map_); //ROS2 msg -> lanelet map
    const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_); //lanelet map -> lanelets
    
    for(const lanelet::ConstLanelet & lanelet : all_lanelets)
    {
        const lanelet::ConstLineString2d centerline = lanelet.centerline2d(); //lanelet -> centerline2d

        for (const auto & point : centerline)
        {
            Point p;
            p.x = point.x();
            p.y = point.y();
            status_.generated_centerpoint.push_back(p);
        }
    }

    for(int i = 0; i < status_.generated_centerpoint.size(); i += param_.centerline_resolution)
    {
        status_.filtered_centerpoint.push_back(status_.generated_centerpoint[i]);
    }
}

/** 
 * @brief Loop initialization depending on ego state.
 *        This is the main function of the node. 
 */
void MapBasedPoseInitializer::run()
{
    updateCurrentState();

    debugCurrentState();

    /** 
     * @brief request initialization if not requested
     */
    if(!status_.is_init_requested && 
        status_.is_ego_initialized && 
            !status_.is_ego_on_initialization) 
    {
        if(status_.filtered_centerpoint.empty())
        {
            return;
        }

        status_.is_init_requested = initPose();
    }
    else if(status_.is_init_requested &&
        status_.is_ego_initialized &&
            !status_.is_ego_on_initialization)
    {
        if(status_.localization_accuracy.lateral_direction < param_.localization_score_threshold &&
            status_.localization_accuracy_count > param_.localization_accuracy_count_threshold)
        {
            RCLCPP_INFO(rclcpp::get_logger("map_based_pose_initializer"), "Initialized");
            RCLCPP_INFO(rclcpp::get_logger("map_based_pose_initializer"), "Localization score[%f] below threshold[%f]",
                status_.localization_accuracy.lateral_direction, param_.localization_score_threshold);
            status_.localization_accuracy_count = 0;
        }
        else if(status_.localization_accuracy.lateral_direction > param_.localization_score_threshold &&
            status_.localization_accuracy_count > param_.localization_accuracy_count_threshold)
        {
            RCLCPP_ERROR(rclcpp::get_logger("map_based_pose_initializer"), "Localization score[%f] above threshold[%f]",
                status_.localization_accuracy.lateral_direction, param_.localization_score_threshold);
            status_.is_init_requested = false;
            status_.localization_accuracy_count = 0;
        }
        else
        {
            status_.localization_accuracy_count++;
            return;
        }
    }
}

/** 
 * @brief update ego status
 */
void MapBasedPoseInitializer::updateCurrentState()
{
    rclcpp::Time current_time = rclcpp::Clock().now();
    const double dt = (current_time - pose_time_).seconds();
    if(dt < 0.1)
    {
        status_.is_ego_initialized = true;
    }
    else
    {
        status_.is_ego_initialized = false;
    }

    if(!status_.is_ego_initialized && status_.is_init_requested)
    {
        status_.is_ego_on_initialization = true;
    }
    else
    {
        status_.is_ego_on_initialization = false;
    }
}

/** 
 * @brief Debug status info.
 *        This is controlled by param "enable_status_debug"
 */
void MapBasedPoseInitializer::debugCurrentState()
{
    if(param_.enable_status_debug)
    {
        RCLCPP_INFO(rclcpp::get_logger("map_based_pose_initializer"), "is_init_requested:        %s", status_.is_init_requested ? "true" : "false");  
        RCLCPP_INFO(rclcpp::get_logger("map_based_pose_initializer"), "is_ego_initialized:       %s", status_.is_ego_initialized ? "true" : "false");  
        RCLCPP_INFO(rclcpp::get_logger("map_based_pose_initializer"), "is_ego_on_initialization: %s", status_.is_ego_on_initialization ? "true" : "false");  
        RCLCPP_INFO(rclcpp::get_logger("map_based_pose_initializer"), "init_count:               %d", status_.init_count);  
        RCLCPP_INFO(rclcpp::get_logger("map_based_pose_initializer"), "=================================");  
    }
    else
    {
        return;
    }
}

/** 
 * @brief Request localization initialization "/localization/initialize". 
 *        The initialization sequence is from the subscribed lanelet.
 * @return service call status(bool)
 */
bool MapBasedPoseInitializer::initPose()
{
    PoseWithCovarianceStamped pose_msg;

    pose_msg.pose.pose.position.x = status_.filtered_centerpoint[status_.init_count].x;
    pose_msg.pose.pose.position.y = status_.filtered_centerpoint[status_.init_count].y;
    pose_msg.pose.pose.position.z = 0.0;
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;
    pose_msg.pose.pose.orientation.w = 1.0;

    pose_pub_->publish(pose_msg);

    auto request = std::make_shared<InitializeLocalization::Request>();

    if (!cli_init_pose_->service_is_ready()) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("map_based_pose_initializer"), "Service not ready");
        return false;
    }

    cli_init_pose_->async_send_request(request);
    status_.init_count++;

    RCLCPP_INFO(rclcpp::get_logger("map_based_pose_initializer"), "Initialize service requested");
    return true;
}
}// namespace map_based_pose_initializer
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(map_based_pose_initializer::MapBasedPoseInitializer)
