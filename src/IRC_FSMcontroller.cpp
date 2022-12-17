#include "ros/ros.h"
#include "IRC_pkg/IRC_FSMlinetrace.h"
#include "IRC_pkg/IRC_FSMballshoot.h"

#define linetrace_target_vel 0.2
#define init_crossline_cnt 5
#define anglevel_target 30

int cmd_cnt = 0;

int main(int argc, char **argv)
{
    float target_vel = linetrace_target_vel;
    int crossline_cnt = init_crossline_cnt;

    ros::init(argc, argv, "IRC_FSMcontroller");

    ros::NodeHandle nh;
    ros::ServiceClient linetrace_client = nh.serviceClient<IRC_pkg::IRC_FSMlinetrace>("linetrace1");
    ros::ServiceClient ballshoot_client = nh.serviceClient<IRC_pkg::IRC_FSMballshoot>("ballshoot");

    IRC_pkg::IRC_FSMlinetrace linetrace_srv;
    IRC_pkg::IRC_FSMballshoot ballshoot_srv;

    while(ros::ok())
    {
        switch(cmd_cnt)
        {
        case 0:
            linetrace_srv.request.vel = target_vel;
            linetrace_srv.request.linetrace_en = true;
            linetrace_srv.request.crossline_cnt = crossline_cnt;
            if (linetrace_client.call(linetrace_srv))
            {
                ROS_INFO("correct linetrace work");
                cmd_cnt = 1;
            }
            else
            {
                ROS_ERROR("Failed to call service linetrace");
                return 1;
            }
            break;

        case 1:
            ballshoot_srv.request.vel = target_vel;
            ballshoot_srv.request.anglevel = anglevel_target;
            ballshoot_srv.request.linetrace_en = false;
            ballshoot_srv.request.odom = {0.15, 0.15, 45};
            ballshoot_srv.request.odom_reset = 1;
            if (ballshoot_client.call(ballshoot_srv))
            {
                cmd_cnt = 2;
            }
            else
            {
                ROS_ERROR("Failed to call service ballshoot");
                return 1;
            }
            break;
        }
    }

    return 0;
}