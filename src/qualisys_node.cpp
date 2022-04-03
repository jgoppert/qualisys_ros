#include <unistd.h>
#include <math.h>
#include <map>

#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include "RTProtocol.h"
#include "RTPacket.h"

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    using namespace std;

    // Set up ROS.
    ros::init(argc, argv, "qualisys_node");
    ros::NodeHandle nh("~");;

    // publications
    std::map<std::string, ros::Publisher> pub_pose;
    std::map<std::string, ros::Publisher> pub_odom;

    std::map<std::string, ros::Time> pub_stamps;

    // Set up a dynamic reconfigure server.
    // This should be done before reading parameter server values.
    //dynamic_reconfigure::Server<node_example::node_example_paramsConfig> dr_srv;
    //dynamic_reconfigure::Server<node_example::node_example_paramsConfig>::CallbackType cb;
    //cb = boost::bind(&NodeExample::configCallback, node_example, _1, _2);
    //dr_srv.setCallback(cb);

    // declare variables that can be modified by launch file or command line.
    string server;
    string parent_frame;
    double rate_limit;
    int slow_count = 0; // watch for slow publication

    // for real-time we want a small queue_size
    const int queue_size = 1;

    // initialize parameters from launch file or command line.
    nh.param("server", server, string("127.0.0.1"));
    nh.param("rate_limit", rate_limit, 10.0);
    nh.param("parent_frame", parent_frame, string("qualisys"));

    try
    {
        CRTProtocol rtProtocol;

        //Example code for how to use discovery calls.
        //if (rtProtocol.DiscoverRTServer(4534, false))
        //{
        //    sleep(1);
        //    const auto numberOfResponses = rtProtocol.GetNumberOfDiscoverResponses();
        //    for (auto index = 0; index < numberOfResponses; index++)
        //    {
        //        unsigned int addr;
        //        unsigned short basePort;
        //        std::string message;
        //        if (rtProtocol.GetDiscoverResponse(index, addr, basePort, message))
        //        {
        //            printf("%2d - %d.%d.%d.%d:%d\t- %s\n", index, 0xff & addr, 0xff & (addr >> 8), 0xff & (addr >> 16), 0xff & (addr >> 24), basePort, message.c_str());
        //        }
        //    }
        //}
        //else
        //{
        //    printf("%s", rtProtocol.GetErrorString());
        //}

        const unsigned short basePort = 22222;
        const int majorVersion = 1;
        const int minorVersion = 19;
        const bool bigEndian = false;

        bool dataAvailable = false;
        bool streamFrames = false;
        unsigned short udpPort = 6734;

        // Main loop.
        while (nh.ok())
        {
            if (!rtProtocol.Connected())
            {
                if (!rtProtocol.Connect(server.c_str(), basePort, &udpPort, majorVersion, minorVersion, bigEndian))
                {
                    ROS_WARN("rtProtocol.Connect: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
            }

            if (!dataAvailable)
            {
                if (!rtProtocol.Read6DOFSettings(dataAvailable))
                {
                    ROS_WARN("rtProtocol.Read6DOFSettings: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
            }

            if (!streamFrames)
            {
                if (!rtProtocol.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort, NULL, CRTProtocol::cComponent6d))
                {
                    ROS_WARN("rtProtocol.StreamFrames: %s\n\n", rtProtocol.GetErrorString());
                    sleep(1);
                    continue;
                }
                streamFrames = true;

                ROS_INFO("Starting to streaming 6DOF data");
            }

            CRTPacket::EPacketType packetType;

            if (rtProtocol.ReceiveRTPacket(packetType, true) > 0)
            {
                if (packetType == CRTPacket::PacketData)
                {
                    ros::Time now = ros::Time::now();
                    float fX, fY, fZ;
                    float rotationMatrix[9];

                    CRTPacket *rtPacket = rtProtocol.GetRTPacket();

                    //ROS_WARN("Frame %d\n", rtPacket->GetFrameNumber());

                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                    {

                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                        {
                            string name(rtProtocol.Get6DOFBodyName(i));
                            //ROS_WARN("data received for rigid body %s", name.c_str());

                            if (!isfinite(fX) || !isfinite(fY) || !isfinite(fZ)) {
                                ROS_WARN_THROTTLE(3, "rigid body %s tracking lost", name.c_str());
                                continue;
                            }

                            for (int i=0; i<9; i++) {
                                if (!isfinite(rotationMatrix[i])) {
                                    ROS_WARN_THROTTLE(3, "rigid body %s tracking lost", name.c_str());
                                    continue;
                                }
                            }


                            // convert to quaternion
                            tf2::Matrix3x3 R(
                              rotationMatrix[0], rotationMatrix[3], rotationMatrix[6],
                              rotationMatrix[1], rotationMatrix[4], rotationMatrix[7],
                              rotationMatrix[2], rotationMatrix[5], rotationMatrix[8]);
                            tf2::Quaternion q;
                            R.getRotation(q);

                            // scale position to meters from mm
                            double x = fX/1.0e3;
                            double y = fY/1.0e3;
                            double z = fZ/1.0e3;
                            double elapsed = 0;

                            // publish data if rate limit met
                            if (pub_stamps.count(name) == 0) {
                                elapsed = 0;
                            } else {
                              elapsed = (now - pub_stamps[name]).toSec();
                              if (elapsed < 0.99/rate_limit) {
                                // wait
                                continue;
                              }
                            }
                            pub_stamps[name] = now;

                            // warning if slow
                            if (elapsed > 3.0/rate_limit) {
                                slow_count += 1;
                                if (slow_count > 10) {
                                    ROS_WARN_THROTTLE(3, "publication rate low: %10.4f Hz", 1.0/elapsed);
                                    slow_count = 0;
                                }
                            }

                            // publish transform
                            {
                                static tf2_ros::TransformBroadcaster br;
                                geometry_msgs::TransformStamped transformStamped;
                                transformStamped.header.stamp = now;
                                transformStamped.header.frame_id = parent_frame;
                                transformStamped.child_frame_id = name;
                                transformStamped.transform.translation.x = x;
                                transformStamped.transform.translation.y = y;
                                transformStamped.transform.translation.z = z;
                                transformStamped.transform.rotation.x = q.x();
                                transformStamped.transform.rotation.y = q.y();
                                transformStamped.transform.rotation.z = q.z();
                                transformStamped.transform.rotation.w = q.w();
                                br.sendTransform(transformStamped);
                            }

                            // publish pose stamped message
                            {
                                if (pub_pose.find(name) == pub_pose.end()) {
                                    ROS_INFO("rigid body %s pose added", name.c_str());
                                    pub_pose[name] = nh.advertise<geometry_msgs::PoseStamped>(name + "/pose", queue_size);
                                }
                                geometry_msgs::PoseStamped msg;
                                msg.header.frame_id= parent_frame;
                                msg.header.stamp = now;
                                msg.pose.position.x = x;
                                msg.pose.position.y = y;
                                msg.pose.position.z = z;
                                msg.pose.orientation.x = q.x();
                                msg.pose.orientation.y = q.y();
                                msg.pose.orientation.z = q.z();
                                msg.pose.orientation.w = q.w();
                                pub_pose[name].publish(msg);
                            }

                            // publish odom message
                            {
                                if (pub_odom.find(name) == pub_odom.end()) {
                                    ROS_INFO("rigid body %s odom added", name.c_str());
                                    pub_odom[name] = nh.advertise<nav_msgs::Odometry>(name + "/odom", queue_size);
                                }
                                nav_msgs::Odometry msg;
                                msg.header.frame_id= parent_frame;
                                msg.header.stamp = now;
                                msg.child_frame_id=name;
                                for (int i=0; i < 36; i++) msg.pose.covariance[i] = NAN;
                                msg.pose.pose.position.x = x;
                                msg.pose.pose.position.y = y;
                                msg.pose.pose.position.z = z;
                                msg.pose.pose.orientation.x = q.x();
                                msg.pose.pose.orientation.y = q.y();
                                msg.pose.pose.orientation.z = q.z();
                                msg.pose.pose.orientation.w = q.w();
                                for (int i=0; i < 36; i++) msg.twist.covariance[i] = NAN;
                                msg.twist.twist.linear.x = NAN;
                                msg.twist.twist.linear.y = NAN;
                                msg.twist.twist.linear.z = NAN;
                                msg.twist.twist.angular.x = NAN;
                                msg.twist.twist.angular.y = NAN;
                                msg.twist.twist.angular.z = NAN;
                                pub_odom[name].publish(msg);
                            }
                        }
                    }
                }
            }
        }
        ros::spinOnce();
        rtProtocol.StreamFramesStop();
        rtProtocol.Disconnect();
    }
    catch (std::exception &e)
    {
        printf("%s\n", e.what());
    }
    return 1;

    return 0;
} // end main()
