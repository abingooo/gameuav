#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

class PlanPointResponder
{
public:
  PlanPointResponder()
    : nh_private_("~"),
      received_count_(0)
  {
    point_topic_ = nh_private_.param<std::string>("topic", "/planning/goal");

    point_sub_ = nh_.subscribe(point_topic_, 10, &PlanPointResponder::pointCallback, this);

    ROS_INFO("PlanPointResponder listening on %s", point_topic_.c_str());
  }

private:
  void pointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    received_count_++;

    const auto &p = msg->pose.position;
    ROS_INFO("Received plan point #%d: (%.2f, %.2f, %.2f) frame=%s",
             received_count_, p.x, p.y, p.z, msg->header.frame_id.c_str());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber point_sub_;
  std::string point_topic_;
  int received_count_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_plan_point_responder_cpp");
  PlanPointResponder responder;
  ros::spin();
  return 0;
}
