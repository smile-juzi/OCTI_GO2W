#include <ros/ros.h>
#include <Eigen/Eigen>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class pointcloudTF{
    public:
        pointcloudTF(){
            nh.param<std::string>("tf_input_pcl",tf_input_pcl,"/cloud_registered");
            nh.param<std::string>("tf_output_pcl",tf_output_pcl,"/tf_pointcloud");
            nh.param<std::string>("output_id",output_id,"robot");
            // nh.param<double>("x_T",x_T,0.0);
            // nh.param<double>("y_T",y_T,0.0);
            // nh.param<double>("z_T",z_T,0.0);
            // nh.param<double>("roll_T",roll_T,0.0);
            // nh.param<double>("pitch_T",pitch_T,0.0);
            // nh.param<double>("yaw_T",yaw_T,0.0);

            sub_pointcloud2 = nh.subscribe(tf_input_pcl,200000,&pointcloudTF::tf_pcl_cbk,this);
            pub_pointcloud2 = nh.advertise<sensor_msgs::PointCloud2>(tf_output_pcl,1);

        }

        void tf_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
        { 
            if(output_id != msg->header.frame_id)
                {
                    tf::StampedTransform transform; 
                    try
                    {
                        listener.lookupTransform(output_id,msg->header.frame_id,ros::Time(0),transform);
                        sensor_msgs::PointCloud2 transform_cloud;
                        pcl_ros::transformPointCloud(output_id,transform,*msg,transform_cloud);

                        transform_cloud.header.frame_id = output_id;
                        transform_cloud.header.stamp = ros::Time::now();
                        pub_pointcloud2.publish(transform_cloud);

                        //以下为再次基于坐标原点的变换

                        // Eigen::Translation3d translation(x_T,y_T,z_T);              //三维平移矩阵
                        // Eigen::Affine3d rotation = Eigen::Affine3d::Identity();     //创建三维仿射变换矩阵rotation,并将其单位化Identity
                        // rotation.rotate(Eigen::AngleAxisd(roll_T, Eigen::Vector3d::UnitX()));
                        // rotation.rotate(Eigen::AngleAxisd(pitch_T, Eigen::Vector3d::UnitY()));
                        // rotation.rotate(Eigen::AngleAxisd(yaw_T, Eigen::Vector3d::UnitZ()));
                        // Eigen::Affine3d transform_matrix = translation * rotation;

                        // pcl_ros::transformPointCloud(transform_matrix, transform_cloud, transform_cloud);
                        
                    }
                    catch(tf::LookupException &e)
                    {
                        ROS_ERROR("Failed to transform point cloud: %s", e.what());
                    }    
                }
                else
                {
                    ROS_WARN("transform in the same coordinate system");
                }
        }
    
    private:
        ros::NodeHandle nh;

        ros::Subscriber sub_pointcloud2;
        ros::Publisher pub_pointcloud2;
        tf::TransformListener listener;

        std::string tf_input_pcl;
        std::string tf_output_pcl;
        std::string output_id;

        // double x_T;
        // double y_T;
        // double z_T;
        // double roll_T;
        // double pitch_T;
        // double yaw_T;

};


int main(int argc,char* argv[])
{
    ros::init(argc, argv, "pointcloud_tf");
    pointcloudTF transformer;
    ros::spin();
    return 0;
}
