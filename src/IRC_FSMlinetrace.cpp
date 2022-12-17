#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "IRC_pkg/IRC_FSMlinetrace.h"

class Server {

private:
  ros::NodeHandle nh;         // ノードハンドルの宣言
  ros::Publisher cmd_vel;     // 目標速度出力
  ros::Publisher linetrace_en; //ライントレースの有無
  ros::Subscriber crossline_sub;   // クロスライン検出
  ros::ServiceServer linetrace_srv; // サービス
  std_msgs::Float32 target_vel;
  std_msgs::Bool linetrace_EN;
  int crossline_cnt = 0;
  int crossline_find = 0;

public:
  Server();      // コンストラクタ
  ~Server() {};  // デストラクタ
  // サーバーのコールバック関数（サービスの本体）
  bool linetrace_srvCallback(IRC_pkg::IRC_FSMlinetrace::Request  &req,
                     IRC_pkg::IRC_FSMlinetrace::Response &res);

  void IRCcrosslineCallback(const std_msgs::Bool& crossline);

};

// コンストラクタ
Server::Server()
{

  //サービスの設定
  linetrace_srv = nh.advertiseService("linetrace1", &Server::linetrace_srvCallback, this);

  // パブリッシャの設定:                                                             
  cmd_vel= nh.advertise<std_msgs::Float32>("/IRCcmd_vel", 10);
  linetrace_en = nh.advertise<std_msgs::Bool>("/IRC_linetrace", 10);

  // サブスクライバの設定                                       
  crossline_sub = nh.subscribe("/IRC_crossline", 10, &Server::IRCcrosslineCallback, this);

}

// サーバーのコールバック関数                                                              
bool Server::linetrace_srvCallback(IRC_pkg::IRC_FSMlinetrace::Request  &req,
                   IRC_pkg::IRC_FSMlinetrace::Response &res)
{
    ros::Rate loop_rate(10);

    target_vel.data = (float)req.vel;
    linetrace_EN.data = (bool)req.linetrace_en;
    crossline_find = (int)req.crossline_cnt;
    crossline_cnt = 0;

    while(crossline_cnt < crossline_find){
        cmd_vel.publish(target_vel);
        linetrace_en.publish(linetrace_EN);
        ros::spinOnce();
        loop_rate.sleep();
    }

    target_vel.data = 0.0;
    cmd_vel.publish(target_vel);

    return true;
}

// コールバック関数                                        
void Server::IRCcrosslineCallback(const std_msgs::Bool& crossline)
{
    if(crossline.data)
    {
        crossline_cnt++;
        ROS_INFO("Find crossline count(%d)", crossline_cnt);
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "IRC_FSM_linetraceServer");
    Server srv;
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}