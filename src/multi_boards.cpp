/**
* @file multi_boards.cpp
* @author Hamdi Sahloul
* @date September 2014
* @version 0.1
* @brief Detect multi-boards simultaneously.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ar_sys/utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>


#include "ar_sys/Board_service.h"
#include "visualization_msgs/InteractiveMarker.h"

using namespace aruco;

struct board_t
{
	int uid;
	std::string name;
	BoardConfiguration config;
	double marker_size;
};

class ArSysMultiBoards
{
    private:
        cv::Mat inImage, resultImg;
        aruco::CameraParameters camParam;
        bool useRectifiedImages;
        bool draw_markers;
        bool draw_markers_cube;
        bool draw_markers_axis;
        bool publish_tf;
        MarkerDetector mDetector;
        vector<Marker> markers;
        BoardConfiguration the_board_config;
        BoardDetector the_board_detector;
        ros::Subscriber cam_info_sub;
        bool cam_info_received;
        image_transport::Publisher image_pub;
        image_transport::Publisher debug_pub;
        ros::Publisher pose_pub;
        ros::Publisher transform_pub;
        ros::Publisher position_pub;
        std::string boards_config;
        std::string boards_config_multi;
        double marker_size;
        std::string boards_directory;
        vector<board_t> boards;

        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber image_sub;

        tf::TransformListener _tfListener;

        ros::ServiceServer board_srv;

        ros::Publisher comput_time_pub;
        ros::Publisher pose_tProc_tDetec_pub;
        ros::Subscriber decision_process_sub;

        ros::Time t1_img_proc_aruco;

        // Current display
        std::string displayed_yaml;
        // Current selection by decision process
        std::string current_yaml;
        // Next expected yaml
        std::string next_yaml;
        bool first_detection;

        // Keep track of the number of times used
        unsigned int counter88;
        unsigned int counter00;

        // Publish info of the current status
        ros::Publisher status_pub;

        //
        ros::Subscriber reset_counters_sub;

    public:
        ArSysMultiBoards()
                : cam_info_received(false),
                nh("~"),
                it(nh)
        {
            first_detection = false;
            counter00 = 0;
            counter88 = 0;

            /// Subscribers ///
            image_sub = it.subscribe("/image", 1, &ArSysMultiBoards::image_callback, this);
			cam_info_sub = nh.subscribe("/camera_info", 1, &ArSysMultiBoards::cam_info_callback, this);

            // Read data from Decision_process
            decision_process_sub = nh.subscribe("/type_size_marker", 1, &ArSysMultiBoards::DecisionProcess_listenerCallback , this);
            reset_counters_sub = nh.subscribe("/reset_counters", 1, &ArSysMultiBoards::reset_counters_callback , this);

            status_pub = nh.advertise<geometry_msgs::PointStamped>("detect_status", 100);

			image_pub = it.advertise("result", 1);
            debug_pub = it.advertise("debug", 1);
			pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
			transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
            position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);

            comput_time_pub = nh.advertise<std_msgs::Float64>("computation_time", 100);
            pose_tProc_tDetec_pub = nh.advertise<geometry_msgs::PoseStamped>("Data_Experiments", 100);

            board_srv = nh.advertiseService("changeboard", &ArSysMultiBoards::setBoard, this);

			nh.param<std::string>("boards_config", boards_config, "boardsConfiguration.yml");
			nh.param<std::string>("boards_config_multi", boards_config_multi, "boardsMultiConfiguration.yml");
			nh.param<std::string>("boards_directory", boards_directory, "./data");
			nh.param<bool>("image_is_rectified", useRectifiedImages, true);
			nh.param<bool>("draw_markers", draw_markers, false);
			nh.param<bool>("draw_markers_cube", draw_markers_cube, false);
			nh.param<bool>("draw_markers_axis", draw_markers_axis, false);
                        nh.param<bool>("publish_tf", publish_tf, false);

			readFromFile(boards_config.c_str());
			ROS_INFO("ArSys node started with boards configuration: %s", boards_config.c_str());
        }


        ///Service code one marker
        bool setBoard(ar_sys::Board_service::Request &req ,
                       ar_sys::Board_service::Response &res)
        {

            // erase the boards
            if(boards.size()!=1){
                for ( int i=1 ;i < boards.size() ; i++)
                {
                  boards.pop_back();
                }
            }

            double size_mk = req.marker_size; //mm to cm
            int id_manual = req.marker_id;
            int option = req.marker_family;

            ROS_INFO("Sending to change parameters");
            ROS_INFO("ArSys node started with manual board configuration: \n");

                if (option == 1)
                {
                    change_parametersOneMarker(id_manual , size_mk);
                }

                else if (option == 2)
                {
                    change_parametersMultiMarker( size_mk );
                }

            res.result = true;

            return true;

        }

        void change_parametersOneMarker(int id_manual , double size_mk)
        {
            boards[0].uid = 1;
            boards[0].name = "Manually";
            boards[0].marker_size = size_mk/100;
            boards[0].config.updateBoardServiceOneMarker(id_manual, size_mk/100);

            ROS_INFO("\n Finish aruco_single configuration \n");

          }

        void change_parametersMultiMarker( double size_mk )
        {
            boards[0].uid = 1;
            boards[0].name = "MULTI_Manually";
            boards[0].marker_size = size_mk/100;

            std::string path(boards_config_multi);
            boards[0].config.readFromFile(path);

            ROS_INFO("\n Finish aruco_multi configuration \n");

        }


		void readFromFile ( string sfile ) throw ( cv::Exception )
		{
			try
			{
				cv::FileStorage fs ( sfile,cv::FileStorage::READ );
				readFromFile ( fs );
			}
			catch (std::exception &ex)
			{
				throw	cv::Exception ( 81818,"ArSysMultiBoards::readFromFile",ex.what()+string(" file=)")+sfile ,__FILE__,__LINE__ );
			}
		}


		void readFromFile ( cv::FileStorage &fs ) throw ( cv::Exception )
		{
            //look for the ar_sys_boards
			if (fs["ar_sys_boards"].name() != "ar_sys_boards")
				throw cv::Exception ( 81818,"ArSysMultiBoards::readFromFile","invalid file type" ,__FILE__,__LINE__ );

			cv::FileNode FnBoards=fs["ar_sys_boards"];
			for (cv::FileNodeIterator it = FnBoards.begin(); it != FnBoards.end(); ++it)

			{
				board_t board;

				board.uid = boards.size();
				board.name = (std::string)(*it)["name"];
				board.marker_size = (double)(*it)["marker_size"];

				std::string path(boards_directory);
				path.append("/");
				path.append((std::string)(*it)["path"]);
				board.config.readFromFile(path);


                                boards.push_back(board);
                         }


			ROS_ASSERT(boards.size() > 0);
		}

        ///Decision process callback code
        void DecisionProcess_listenerCallback(const visualization_msgs::InteractiveMarker & data)
        {


            if ( true ){
                // erase the boards
                if(boards.size()!=1){
                    for ( int i=1 ;i < boards.size() ; i++)
                    {
                      boards.pop_back();
                    }
                }

                double size_mk = data.scale/100;
                int id_manual = data.pose.position.x;

                ROS_INFO_STREAM("Sending to change parameters");
                change_parameters_DecisionProcess(id_manual , size_mk);
            }
            else{
                int id = data.pose.position.x;
                if ( id == 88)
                {
                    current_yaml = "YAML_88_14";
                }
                else
                {
                    current_yaml = "YAML_00_14";
                }
                next_yaml = current_yaml;
                //ROS_INFO_STREAM("Set current_yaml " << current_yaml);
            }
        }

        void change_parameters_DecisionProcess(int id_manual , double size_mk)
        {
            boards[0].uid = 1;
            boards[0].name = "Manually";
            boards[0].marker_size = size_mk;
            boards[0].config.updateBoardDecisionProcess(id_manual, size_mk);

            ROS_INFO("\n ArSys node started with decision process configuration: \n");
        }

		void image_callback(const sensor_msgs::ImageConstPtr& msg)
		{

            static tf::TransformBroadcaster br;

            if(!cam_info_received) return;

            t1_img_proc_aruco = ros::Time::now();

			cv_bridge::CvImagePtr cv_ptr;
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inImage = cv_ptr->image;
				resultImg = cv_ptr->image.clone();

                //detection results will go into "markers"
				markers.clear();

				//Ok, let's detect
				double min_size = boards[0].marker_size;
				for (int board_index = 1; board_index < boards.size(); board_index++)
					if (min_size > boards[board_index].marker_size) min_size = boards[board_index].marker_size;
				mDetector.detect(inImage, markers, camParam, min_size, false);

                //computation time - image process
                //double result_img_proc = mDetector.t2_img_proc_aruco.toNSec() - t1_img_proc_aruco.toNSec();

                for (int board_index = 0; board_index < boards.size(); board_index++)
				{
					Board board_detected;

					//Detection of the board
					float probDetect = the_board_detector.detect(markers, boards[board_index].config, board_detected, camParam, boards[board_index].marker_size);

                    if (probDetect > 0.0)
					{
						tf::Transform transform = ar_sys::getTf(board_detected.Rvec, board_detected.Tvec);

						tf::StampedTransform stampedTransform(transform, msg->header.stamp, msg->header.frame_id, boards[board_index].name);
                        ros::Time t2_detecion = ros::Time::now(); // time elapsed since the beginning of the processing
                        if (publish_tf)
                            br.sendTransform(stampedTransform);

						geometry_msgs::PoseStamped poseMsg;
						tf::poseTFToMsg(transform, poseMsg.pose);
						poseMsg.header.frame_id = msg->header.frame_id;
                        poseMsg.header.stamp = msg->header.stamp;
                        pose_pub.publish(poseMsg);

						geometry_msgs::TransformStamped transformMsg;
						tf::transformStampedTFToMsg(stampedTransform, transformMsg);
                        transform_pub.publish(transformMsg);

                        //
                        detection_manager( transformMsg.child_frame_id );

						geometry_msgs::Vector3Stamped positionMsg;
						positionMsg.header = transformMsg.header;
						positionMsg.vector = transformMsg.transform.translation;
						position_pub.publish(positionMsg);

						if(camParam.isValid())
						{
							//draw board axis
							CvDrawingUtils::draw3dAxis(resultImg, board_detected, camParam);
						}

                        //computation time - detection
                       // double result_detect = t2_detecion.toNSec() - mDetector.t1_detec_aruco.toNSec();

                        // computation time detection - computation time image procesin and pose z in cm
                        geometry_msgs::PoseStamped DataMsg;
                        DataMsg.pose.position.x = 100 * poseMsg.pose.position.z;
                        DataMsg.pose.position.y = result_detect;
                        DataMsg.pose.position.z = result_img_proc;
                        pose_tProc_tDetec_pub.publish(DataMsg);

                        //ROS_INFO_STREAM("Computed time = " << result_detect);
					}
                }

				//for each marker, draw info and its boundaries in the image
				for(size_t i=0; draw_markers && i < markers.size(); ++i)
				{
					markers[i].draw(resultImg,cv::Scalar(0,0,255),2);
				}

				if(camParam.isValid())
				{
					//draw a 3d cube in each marker if there is 3d info
					for(size_t i=0; i<markers.size(); ++i)
					{
						if (draw_markers_cube) CvDrawingUtils::draw3dCube(resultImg, markers[i], camParam);
						if (draw_markers_axis) CvDrawingUtils::draw3dAxis(resultImg, markers[i], camParam);
					}
				}

				if(image_pub.getNumSubscribers() > 0)
				{
					//show input with augmented information
					cv_bridge::CvImage out_msg;
					out_msg.header.frame_id = msg->header.frame_id;
					out_msg.header.stamp = msg->header.stamp;
					out_msg.encoding = sensor_msgs::image_encodings::RGB8;
					out_msg.image = resultImg;
					image_pub.publish(out_msg.toImageMsg());
				}

				if(debug_pub.getNumSubscribers() > 0)
				{
					//show also the internal image resulting from the threshold operation
					cv_bridge::CvImage debug_msg;
					debug_msg.header.frame_id = msg->header.frame_id;
					debug_msg.header.stamp = msg->header.stamp;
					debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
					debug_msg.image = mDetector.getThresholdedImage();
					debug_pub.publish(debug_msg.toImageMsg());
				}
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
		}

		// wait for one camerainfo, then shut down that subscriber
		void cam_info_callback(const sensor_msgs::CameraInfo &msg)
		{
			camParam = ar_sys::getCamParams(msg, useRectifiedImages);
			cam_info_received = true;
            cam_info_sub.shutdown();
        }

        //
        void detection_manager ( std::string yaml_name)
        {
            geometry_msgs::PointStamped status_msg;
//            if ( first_detection == false )
//            {
//                next_yaml = current_yaml;
//                first_detection = true;
//            }
            displayed_yaml = yaml_name;

            if ( current_yaml == next_yaml)
            {
                if ( displayed_yaml == current_yaml)
                {
                    // Check is correct
                    status_msg.point.z = 1;

                    if ( displayed_yaml == "YAML_00_14" )
                    {
                        //send YAML_00_14
                        status_msg.header.frame_id = displayed_yaml;
                        // Increase counter
                        counter00++;
                        // Store value
                        status_msg.point.x = counter00;
                        status_msg.point.y = counter88;
                        // Update
                        next_yaml = "YAML_88_14";
                    }
                    else if ( displayed_yaml == "YAML_88_14" )
                    {
                        //send YAML_88_14
                        status_msg.header.frame_id = displayed_yaml;
                        // Increase counter
                        counter88++;
                        // Store value
                        status_msg.point.x = counter00;
                        status_msg.point.y = counter88;
                        // Update
                        next_yaml = "YAML_00_14";
                    }
                    status_pub.publish( status_msg );
                }
                else
                {
                    //
                }
            }
        }

        void reset_counters_callback(const std_msgs::Float64 &msg)
        {
            // Reset counters
            if ( msg.data == -1 )
            {
                ROS_INFO("ERROR - reset counters");
                counter88 = 0;
                counter00 = 0;

                next_yaml = current_yaml;
            }
        }
};



int main(int argc,char **argv)
{
	ros::init(argc, argv, "ar_multi_boards");

	ArSysMultiBoards node;

	ros::spin();
}
