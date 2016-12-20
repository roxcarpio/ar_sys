#include "ros/ros.h"
#include <cstdlib>
#include <string.h>
#include <stdlib.h>

#include "ar_sys/Board_service.h"

using namespace std;

string corner_matrix( const char * selection )
{
    string result = "Not valid";
    switch ( atoi( selection ) ){
        // Left Column
        case 11:
            result = "[-1.285,1.92,0.],[0.218,1.92,0.],[0.218,0.4,0.],[-1.285,0.4,0.]";
            break;
        case 12:
            result = "[1.285,0.34,0.],[-0.081,0.34,0.],[-0.081,-0.861,0.],[-1.285,-0.861,0.]";
            break;
        case 13:
            result = "[-1.285,-0.919,0.],[-0.384,-0.919,0.],[-0.384,-1.821,0.],[-1.285,-1.821,0.]";
            break;
        // Right Column
        case 21:
            result = "[0.485,1.902,0.],[1.186,1.902,0.],[1.186,1.199,0.],[0.485,1.199,0.]";
            break;
        case 22:
            result = "[0.686,0.34,0.],[1.186,0.34,0.],[1.186,-0.16,0.],[0.686,-0.16,0.]";
            break;
        case 23:
            result = "[0.983,-0.922,0.],[1.184,-0.922,0.],[1.184,-1.123,0.],[0.983,-1.123,0.]";
            break;

        default:
            ROS_INFO("Corner selection [%s] not found" , selection);
            ROS_INFO("The dimension of the board is 2x4");
    }
    ROS_INFO("Result of the selection is %s" , result.c_str());
    return result;
}

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

    srv.request.id = atoi(argv[1]);
    string mk_size = argv[2];
    srv.request.marker_size = atoi(mk_size.c_str());
    srv.request.corners = corner_matrix ( argv[3] );

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
