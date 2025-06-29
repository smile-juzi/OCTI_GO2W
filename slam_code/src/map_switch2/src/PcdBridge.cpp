#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <unordered_map>
#include <string>

class PCDMapBridgeNode {
public:
  PCDMapBridgeNode(ros::NodeHandle& nh) : nh_(nh) {
    nh_.getParam("available_pcd_maps", map_list_);
    current_map_id_ = "";
    pub_pcd_ = nh_.advertise<sensor_msgs::PointCloud2>("pcd_map", 1, true);

    // 订阅所有点云地图源
    for (const auto& map_id : map_list_) {
      std::string topic = "/" + map_id + "/pcd_map";
      ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2>(
          topic, 1,
          [this, map_id](const sensor_msgs::PointCloud2::ConstPtr& msg) {
            this->pcdCallback(msg, map_id);
          });
      map_subs_[map_id] = sub;
      ROS_INFO("Subscribed to topic: %s", topic.c_str());
    }

    srv_ = nh_.advertiseService("/switch_pcd_map", &PCDMapBridgeNode::switchPCDMapCallback, this);

    nh_.getParam("target_pcd_map", current_map_id_);
    if (cache_.count(current_map_id_)) {
      pub_pcd_.publish(cache_[current_map_id_]);
      ROS_INFO("Published initial PCD map: %s", current_map_id_.c_str());
    } else {
      ROS_WARN("Initial PCD map %s not in cache yet", current_map_id_.c_str());
    }
  }

  void pcdCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& id) {
    cache_[id] = *msg;
    if (id == current_map_id_) {
      pub_pcd_.publish(msg);
    }
    if (cache_.count(id) == 0) {
      ROS_INFO("Received first PCD data for %s", id.c_str());
    }
  }

  bool switchPCDMapCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    std::string new_map;
    nh_.getParam("target_pcd_map", new_map);
    if (cache_.count(new_map) == 0) {
      res.success = false;
      res.message = "PCD map not received yet.";
      return true;
    }
    current_map_id_ = new_map;
    pub_pcd_.publish(cache_[new_map]);
    res.success = true;
    res.message = "Switched to PCD map: " + new_map;
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_pcd_;
  ros::ServiceServer srv_;
  std::vector<std::string> map_list_;
  std::unordered_map<std::string, sensor_msgs::PointCloud2> cache_;
  std::unordered_map<std::string, ros::Subscriber> map_subs_;
  std::string current_map_id_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcd_map_bridge_node");
  ros::NodeHandle nh;
  PCDMapBridgeNode node(nh);
  ros::spin();
  return 0;
}
