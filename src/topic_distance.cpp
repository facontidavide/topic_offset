#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// REFERENCE: http://wiki.ros.org/rosbag/Code%20API

int main(int argc, char **argv)
{
    bool remove_initial_offset = true;

    ros::init( argc, argv, ROS_PACKAGE_NAME );
    std::string pose1_name, pose2_name;
    std::string inputfile, outputfile;

    ros::param::get( "~pose1", pose1_name );
    ros::param::get( "~pose2", pose2_name );
    ros::param::get( "~remove_initial_offset", remove_initial_offset );
    ros::param::get( "~inputfile", inputfile );
    ros::param::get( "~outputfile", outputfile );

    std::cout << "Argument remove_initial_offset: " <<remove_initial_offset << std::endl;
    std::cout << "Argument pose1: " <<pose1_name << std::endl;
    std::cout << "Argument pose2: " <<pose2_name << std::endl;
    std::cout << "Argument inputfile: " <<inputfile << std::endl;
    std::cout << "Argument outputfile: " <<outputfile << std::endl;

    if( inputfile.size()==0)
    {
        std::cout << "inputfile missing: ABORT" << std::endl;
        return 1;
    }
    if( outputfile.size()==0)
    {
        std::cout << "outputfile missing: ABORT" << std::endl;
        return 1;
    }

    rosbag::Bag input_bag;
    input_bag.open( inputfile, rosbag::bagmode::Read);

    rosbag::Bag output_bag( outputfile, rosbag::BagMode::Write);

    std::vector<std::string> topics;
    topics.push_back( pose1_name );
    topics.push_back( pose2_name );

    rosbag::View view(input_bag, rosbag::TopicQuery(topics));

    geometry_msgs::PoseStamped::Ptr pose1;
    geometry_msgs::PoseStamped::Ptr pose2;

    std::cout << "start reading" << std::endl;

    bool first_offset = true;
    geometry_msgs::Point initial_offset;
    initial_offset.x = 0;
    initial_offset.y = 0;
    initial_offset.z = 0;

    for(rosbag::MessageInstance const m: view)
    {
        if( m.getTopic() == pose1_name)
        {
            pose1 = m.instantiate<geometry_msgs::PoseStamped>();
        }
        else if( m.getTopic() == pose2_name)
        {
            pose2 = m.instantiate<geometry_msgs::PoseStamped>();
            if( pose1 && pose2 )
            {
                if( first_offset && remove_initial_offset)
                {
                    first_offset = false;
                    initial_offset.x = pose2->pose.position.x - pose1->pose.position.x;
                    initial_offset.y = pose2->pose.position.y - pose1->pose.position.y;
                    initial_offset.z = pose2->pose.position.z - pose1->pose.position.z;
                }

                geometry_msgs::Point offset;
                offset.x = pose2->pose.position.x - pose1->pose.position.x - initial_offset.x;
                offset.y = pose2->pose.position.y - pose1->pose.position.y - initial_offset.y;
                offset.z = pose2->pose.position.z - pose1->pose.position.z - initial_offset.z;
                output_bag.write("error_offset", m.getTime(), offset) ;

                std_msgs::Float32 yaw;
                yaw.data    = tf::getYaw(pose2->pose.orientation)   - tf::getYaw(pose1->pose.orientation);
                output_bag.write("yaw_offset", m.getTime(),   yaw) ;
            }
        }
    }
    input_bag.close();
    output_bag.close();
    std::cout << "export done" << std::endl;
}
