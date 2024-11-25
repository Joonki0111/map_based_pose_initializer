#ifndef MAP_BASED_POSE_INITIALIZER__MAP_BASED_POSE_INITIALIZER_HPP_
#define MAP_BASED_POSE_INITIALIZER__MAP_BASED_POSE_INITIALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <autoware_localization_msgs/msg/localization_accuracy.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/utility/query.hpp>

namespace map_based_pose_initializer
{
using autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using autoware_adapi_v1_msgs::srv::InitializeLocalization;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Point;
using autoware_localization_msgs::msg::LocalizationAccuracy;
using autoware_auto_mapping_msgs::msg::HADMapBin;

class MapBasedPoseInitializer : public rclcpp::Node
{
    public:
        explicit MapBasedPoseInitializer(const rclcpp::NodeOptions & node_options);

    private:
        struct Status
        {
            bool is_init_requested{false};
            bool is_ego_initialized{false};
            bool is_ego_on_initialization{false};
            int localization_accuracy_count{0};
            int init_count{0};
            LocalizationAccuracy localization_accuracy{};
            std::vector<Point> generated_centerpoint{};
            std::vector<Point> filtered_centerpoint{};
        };

        struct Param
        {
            bool enable_status_debug;
            int centerline_resolution;
            float localization_score_threshold;
            int localization_accuracy_count_threshold;
        };

        rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub_;

        rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
        rclcpp::Subscription<LocalizationAccuracy>::SharedPtr sub_localization_accuracy_;
        rclcpp::Subscription<HADMapBin>::SharedPtr sub_lanelet_map_;

        rclcpp::Client<InitializeLocalization>::SharedPtr cli_init_pose_;

        rclcpp::TimerBase::SharedPtr timer_;

        void publishGNSSPose();
        void poseCallback(const PoseStamped::ConstSharedPtr pose_msg);
        void localizationAccuracyCallback(const LocalizationAccuracy::ConstSharedPtr lanelet_msg);
        void laneletCallback(const HADMapBin::ConstSharedPtr lanelet_msg);
        void run();
        void updateCurrentState();
        void debugCurrentState();
        bool initPose();

        Status status_{};
        Param param_{};
        lanelet::LaneletMapPtr lanelet_map_{nullptr};
        rclcpp::Time pose_time_{};
};
}
#endif  // MAP_BASED_POSE_INITIALIZER__MAP_BASED_POSE_INITIALIZER_HPP_