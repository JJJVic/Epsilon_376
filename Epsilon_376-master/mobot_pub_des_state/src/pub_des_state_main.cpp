#include "pub_des_state.h"
#include <iostream>
using namespace std;

bool *alarm_p;
bool *estop;


void alarmCB(const std_msgs::Bool g_alarm) {

  if (g_alarm.data == true){
    *alarm_p = true;
  }
  else{*alarm_p = false;}
}
void estopCallback(const std_msgs::Bool estop_call) {
  if (estop_call.data == true){
    *estop = true;
  }
  else{*estop = false;}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "des_state_publisher");
  ros::NodeHandle nh;
  //instantiate a desired-state publisher object
  DesStatePublisher desStatePublisher(nh);
  alarm_p = &(desStatePublisher.alarm);
  estop = &(desStatePublisher.h_e_stop_);

  ros::Subscriber sub = nh.subscribe("/lidar_alarm", 1, alarmCB);
  ros::Subscriber estop_subscriber = nh.subscribe("/motors_enabled", 1, estopCallback);

  // ros::ServiceClient estop_client = nh.serviceClient<std_srvs::Trigger>("/estop_service");
  // while (!estop_client.exists()) {
  //   ROS_INFO("waiting for estop service...");
  //   ros::Duration(1.0).sleep();
  // }
  // ros::ServiceClient clear_stop_client = nh.serviceClient<std_srvs::Trigger>("/clear_estop_service");
  // while (!clear_stop_client.exists()) {
  //   ROS_INFO("waiting for clear estop service...");
  //   ros::Duration(1.0).sleep();
  // }
  // ros::ServiceClient flush_client = nh.serviceClient<std_srvs::Trigger>("/flush_path_queue_service");
  // while (!flush_client.exists()) {
  //   ROS_INFO("waiting for flush service...");
  //   ros::Duration(1.0).sleep();
  // }
  //dt is set in header file pub_des_state.h    
  ros::Rate looprate(1 / dt); //timer for fixed publication rate
  desStatePublisher.set_init_pose(0,0,0); //x=0, y=0, psi=0
  //put some points in the path queue--hard coded here
  desStatePublisher.append_path_queue(5.0,0.0,0.0);
  desStatePublisher.append_path_queue(0.0,0.0,0.0);

  const int STAHP = 1;
  const int RESTART = 2;
  const int FLUSH = 3;
  const int APPEND = 4;
  const int SEGFAULT = -1;
  
  // main loop; publish a desired state every iteration
  int set_botton = 0;
  while (ros::ok()) {
    desStatePublisher.pub_next_state();
    // cout << "If you want to e-stop, Press 1 " << endl;
    // cout << "If you want to clear e-stop, Press 2 " << endl;
    // cout << "If you want to flush the path, Press 3 " << endl;
    // cout << "If you want to append to the queue, Press 4 " << endl;
    // cout << "If you want to segfault the program, press -1 " << endl;
    // cout << "Enter choice: ";
    // cin >> set_botton;

    // switch (set_botton) {
    //   case STAHP: {
    //     std_srvs::Trigger estop_srv; 
    //     //estop_srv.request; 
    //     ROS_INFO("requested estop_srv");
    //     estop_client.call(estop_srv);
    //     ROS_INFO("estop_client has called");
    //     set_botton = 0;
    //     break;
    //   }
    //   case RESTART: {
    //     std_srvs::Trigger clearstop_srv; 
    //     clearstop_srv.request; 
    //     clear_stop_client.call(clearstop_srv);
    //     set_botton = 0;
    //     break;
    //   }
    //   case FLUSH: {
    //     std_srvs::Trigger flush_srv; 
    //     flush_srv.request; 
    //     flush_client.call(flush_srv);
    //     set_botton = 0;
    //     break;
    //   }
    //   case SEGFAULT: {
    //     return 0;
    //   }
    //   default: {
    //     set_botton = 0;
    //     break; 
    //   }
    // }
    ros::spinOnce();
    looprate.sleep(); //sleep for defined sample period, then do loop again
  }
  return 0;
}

