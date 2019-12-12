#include "RTProtocol.h"
#include "RTPacket.h"
#include <unistd.h>
#include <math.h>
#include "ros/ros.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
    using namespace std;

    // Set up ROS.
    ros::init(argc, argv, "qualisys_node");
    ros::NodeHandle n;

    std::map<std::string, ros::Publisher> publishers;

    // Create a new NodeExample object.
    //NodeExample *node_example = new NodeExample();

    // Set up a dynamic reconfigure server.
    // This should be done before reading parameter server values.
    //dynamic_reconfigure::Server<node_example::node_example_paramsConfig> dr_srv;
    //dynamic_reconfigure::Server<node_example::node_example_paramsConfig>::CallbackType cb;
    //cb = boost::bind(&NodeExample::configCallback, node_example, _1, _2);
    //dr_srv.setCallback(cb);

    // Declare variables that can be modified by launch file or command line.
    //int a;
    //int b;
    //string message;
    //string topic;
    string server;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can
    // be run simultaneously while using different parameters.
    // Parameters defined in the .cfg file do not need to be initialized here
    // as the dynamic_reconfigure::Server does this for you.
    ros::NodeHandle private_node_handle_("~");
    //private_node_handle_.param("rate", rate, int(40));
    //private_node_handle_.param("topic", topic, string("example"));
    private_node_handle_.param("server", server, string("127.0.0.1"));

    // Create a publisher and name the topic.
    //ros::Publisher pub_message = n.advertise<node_example::node_example_data>(topic.c_str(), 10);

    // Tell ROS how fast to run this node.
    //ros::Rate r(rate);

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
        while (n.ok())
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
                    float fX, fY, fZ;
                    float rotationMatrix[9];

                    CRTPacket *rtPacket = rtProtocol.GetRTPacket();

                    //printf("Frame %d\n", rtPacket->GetFrameNumber());

                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                    {
                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                        {
                            string name(rtProtocol.Get6DOFBodyName(i));

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

                            // publish pose stamped message
                            {
                                if (publishers.find(name) == publishers.end()) {
                                    ROS_INFO("rigid body %s added", name.c_str());
                                    publishers[name] = n.advertise<geometry_msgs::PoseStamped>(name + "/pose", 10);
                                }
                                geometry_msgs::PoseStamped msg;
                                msg.header.frame_id="qualisys";
                                msg.header.stamp = ros::Time::now();
                                msg.pose.position.x = fX/1.0e3;
                                msg.pose.position.y = fY/1.0e3;
                                msg.pose.position.z = fZ/1.0e3;
                                msg.pose.orientation.x = q.getX();
                                msg.pose.orientation.y = q.getY();
                                msg.pose.orientation.z = q.getZ();
                                msg.pose.orientation.w = q.getW();
                                publishers[name].publish(msg);
                            }
                        }
                    }
                }
            }
            ros::spinOnce();
        }
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
