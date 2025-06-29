#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <unordered_map>
#include <string>

class MapBridgeNode {
public:
  MapBridgeNode(ros::NodeHandle& nh) : nh_(nh) {
    // 初始化地图列表
    nh_.getParam("available_maps", map_list_);
    current_map_id_ = "";
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);

    // 订阅所有地图源
    for (const auto& map_id : map_list_) {
      std::string topic = "/" + map_id + "_raw";  // 修复路径
      ros::Subscriber sub = nh_.subscribe<nav_msgs::OccupancyGrid>(
          topic, 1,
          [this, map_id](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
            this->mapCallback(msg, map_id);
          });
      map_subs_[map_id] = sub;
      ROS_INFO("Subscribed to topic: %s", topic.c_str());
    }
    

    // 地图切换服务
    srv_ = nh_.advertiseService("/switch_map", &MapBridgeNode::switchMapCallback, this);

    nh_.getParam("target_map", current_map_id_);
    if (cache_.count(current_map_id_)) {
      pub_map_.publish(cache_[current_map_id_]);
      ROS_INFO("Published initial map: %s", current_map_id_.c_str());
    } else {
      ROS_WARN("Initial map %s not in cache yet", current_map_id_.c_str());
    }
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, const std::string& id) {
    if (id == current_map_id_) {
        pub_map_.publish(msg);
    }
    cache_[id] = *msg;
    ROS_INFO("Received map data for %s, cache size: %zu", id.c_str(), cache_.size());
    ROS_INFO("Map width: %d, height: %d", msg->info.width, msg->info.height);
}

  bool switchMapCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    std::string new_map;
    nh_.getParam("target_map", new_map);
    if (cache_.count(new_map) == 0) {
      res.success = false;
      res.message = "Map not found or not received yet.";
      return true;
    }
    current_map_id_ = new_map;
    pub_map_.publish(cache_[new_map]);
    res.success = true;
    res.message = "Switched to map: " + new_map;
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_map_;
  ros::ServiceServer srv_;
  std::vector<std::string> map_list_;
  std::unordered_map<std::string, nav_msgs::OccupancyGrid> cache_;
  std::unordered_map<std::string, ros::Subscriber> map_subs_;
  std::string current_map_id_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_bridge_node");
  ros::NodeHandle nh;
  MapBridgeNode node(nh);
  ros::spin();
  return 0;
}
