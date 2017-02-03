#include "ros/ros.h"
#include <cstdlib>
#include <string.h>
#include <stdlib.h>

#include "ar_sys/Board_service.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "changeboard_client");
    //  if (argc != 4)
    //  {
    //    ROS_INFO("usage: add_two_ints_client X Y");
    //    return 1;
    //  }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ar_sys::Board_service>("/ar_multi_boards/changeboard");
    ar_sys::Board_service srv;

    srv.request.marker_family = atoi(argv[1]);
    srv.request.marker_id = atoi(argv[2]);
   // srv.request.marker_size = atoi(argv[3]);

    string mk_size = argv[3];
    srv.request.marker_size = atoi(mk_size.c_str());


    //  srv.request.corners = atof(argv[4]);
    if (client.call(srv)){
        if (srv.response.result){
            ROS_INFO("Succesfully changed board");
        }
        else{
            ROS_INFO("Problem changing board, server replied false");
        }
    }
    else{
        ROS_INFO("Failed to call service changeboard");
    }
}
