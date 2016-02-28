#include "pub_des_state.h"
#include <iostream>
using namespace std;

bool *alarm_p;


void alarmCB(const std_msgs::Bool g_alarm) {

    if (g_alarm.data == true){
        ROS_WARN("ALARM CALLBACK IS ON!");
       *alarm_p = true;
    }
    else{*alarm_p = false;}

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher");
    ros::NodeHandle nh;
    //instantiate a desired-state publisher object
    DesStatePublisher desStatePublisher(nh);
    alarm_p = &(desStatePublisher.alarm);

    ros::Subscriber sub = nh.subscribe("/scan", 1, alarmCB);
    //dt is set in header file pub_des_state.h    
    ros::Rate looprate(1 / dt); //timer for fixed publication rate
    desStatePublisher.set_init_pose(0,0,0); //x=0, y=0, psi=0
    //put some points in the path queue--hard coded here
    desStatePublisher.append_path_queue(5.0,0.0,0.0);
    desStatePublisher.append_path_queue(0.0,0.0,0.0);

 
    
    // main loop; publish a desired state every iteration
    while (ros::ok()) {
        int set_botton =0;
        desStatePublisher.pub_next_state();

        cout << "If you want to e-stop, Press 1 " << endl;
        cout << "If you want to start over, Press 2 " << endl;
        cout << "If you want to flush the path, Press 3 " << endl;
        
        cin >> set_botton;

        if(set_botton == 1){
          ros::ServiceClient estop_client = nh.serviceClient<std_srvs::Trigger>("estop_service");
          ros::Duration(5.0).sleep();
          std_srvs::Trigger estop_srv; 
          estop_srv.request; 
          estop_client.call(estop_srv);
          set_botton = 0;
          
        }

        if(set_botton == 2){
          ros::ServiceClient clear_stop_client = nh.serviceClient<std_srvs::Trigger>("clear_estop_service");
          ros::Duration(5.0).sleep();
          std_srvs::Trigger clearstop_srv; 
          clearstop_srv.request; 
          clear_stop_client.call(clearstop_srv);
          set_botton = 0;
        }

        if(set_botton == 3){
          ros::ServiceClient flush_client = nh.serviceClient<std_srvs::Trigger>("flush_path_queue_service");
          ros::Duration(5.0).sleep();
          std_srvs::Trigger flush_srv; 
          flush_srv.request; 
          flush_client.call(flush_srv);
          set_botton = 0;
        }

        ros::spinOnce();
        looprate.sleep(); //sleep for defined sample period, then do loop again
    }
}

