#include "pub_des_state.h"
#include <iostream>
using namespace std;

TrajBuilder trajBuilder_; 

mobot_pub_des_state::path append_stuff() {
  double x;
  double y;
  double psi;
  int cont = 1;
  int i = 0;
  std::vector<geometry_msgs::PoseStamped> points;
  mobot_pub_des_state::path path_srv;
  while (cont == 1) {
    cout << "Enter x: ";
    cin >> x;
    cout << "Enter y: ";
    cin >> y;
    cout << "Enter psi: ";
    cin >> psi;
    points.push_back(trajBuilder_.xyPsi2PoseStamped(x,y,psi));
    i++;
    cout << "(Y=1, N=0) Continue? ";
    cin >> cont;
  }

  // geometry_msgs::PoseStamped path[i]; 
  // for (int j = 0; j < i; ++j)
  // {
  //   path[j] = points[j];
  // }
  path_srv.request.path.poses = points;
  return path_srv;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "poke_the_robot");
  ros::NodeHandle nh;


  ros::ServiceClient estop_client = nh.serviceClient<std_srvs::Trigger>("/estop_service");
  ros::ServiceClient clear_stop_client = nh.serviceClient<std_srvs::Trigger>("/clear_estop_service");
  ros::ServiceClient flush_client = nh.serviceClient<std_srvs::Trigger>("/flush_path_queue_service");
  ros::ServiceClient append_client = nh.serviceClient<mobot_pub_des_state::path>("/append_path_queue_service");
  bool ready2go = estop_client.exists() && clear_stop_client.exists() && flush_client.exists() && append_client.exists();
  while (!ready2go) {
    ROS_INFO("Waiting for services to initialize...");
    ros::Duration(1.0).sleep();
  }
  const int STAHP = 1;
  const int CLEAR = 2;
  const int FLUSH = 3;
  const int APPEND = 4;
  const int EXIT = -1;
  
  // main loop; publish a desired state every iteration
  int button = 0;
  while (ros::ok()) {
    cout << "If you want to e-stop, Press 1 " << endl;
    cout << "If you want to clear e-stop, Press 2 " << endl;
    cout << "If you want to flush the path, Press 3 " << endl;
    cout << "If you want to append to the queue, Press 4 " << endl;
    cout << "If you want to exit the program, press -1 " << endl;
    cout << "Enter choice: ";
    cin >> button;

    switch (button) {
      case STAHP: {
        std_srvs::Trigger estop_srv; 
        //estop_srv.request; 
        ROS_INFO("Requested E-Stop");
        estop_client.call(estop_srv);
        ROS_INFO("estop_client has been called");
        button = 0;
        break;
      }
      case CLEAR: {
        std_srvs::Trigger clearstop_srv; 
        //clearstop_srv.request; 
        ROS_INFO("Requested clear of E-Stop");
        clear_stop_client.call(clearstop_srv);
        ROS_INFO("clear_stop_client has been called");
        button = 0;
        break;
      }
      case FLUSH: {
        std_srvs::Trigger flush_srv; 
        //flush_srv.request; 
        ROS_INFO("Requested path queue flush");
        flush_client.call(flush_srv);
        ROS_INFO("flush_client has been called");
        button = 0;
        break;
      }
      case APPEND: {
        ROS_INFO("Requested path queue append");
        mobot_pub_des_state::path path_srv = append_stuff();
        append_client.call(path_srv);
        ROS_INFO("append_client has been called");
        button = 0;
        break;
      }
      case EXIT: {
        return 0;
      }
      default: {
        button = 0;
        break; 
      }
    }
  }
  return 0;
}

