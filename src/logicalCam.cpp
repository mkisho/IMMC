#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/msgs/logical_camera_sensor.pb.h>
#include <gazebo/sensors/LogicalCameraSensor.hh>
#include <gazebo/gazebo_client.hh>
using namespace gazebo;


void callback(ConstLogicalCameraImagePtr &_msg)
{
	std::cout << "MSG LOGICAL CAMERA\n";
/*
   osrf_gear::LogicalCameraImage imageMsg;
  msgs::Vector3d cameraPosition = _msg->pose().position();
  msgs::Quaternion cameraOrientation = _msg->pose().orientation();
  imageMsg.pose.position.x = cameraPosition.x();
  imageMsg.pose.position.y = cameraPosition.y();
  imageMsg.pose.position.z = cameraPosition.z();
  imageMsg.pose.orientation.x = cameraOrientation.x();
  imageMsg.pose.orientation.y = cameraOrientation.y();
 imageMsg.pose.orientation.z = cameraOrientation.z();
 imageMsg.pose.orientation.w = cameraOrientation.w();
  
  for (int i = 0; i < _msg->model_size(); ++i)
  {
  msgs::Vector3d position = _msg->model(i).pose().position();
  msgs::Quaternion orientation = _msg->model(i).pose().orientation();
  osrf_gear::Model modelMsg;
  modelMsg.pose.position.x = position.x();
  modelMsg.pose.position.y = position.y();
  modelMsg.pose.position.z = position.z();
  modelMsg.pose.orientation.x = orientation.x();
  modelMsg.pose.orientation.y = orientation.y();
  modelMsg.pose.orientation.z = orientation.z();
  modelMsg.pose.orientation.w = orientation.w();
  modelMsg.type = ariac::DetermineModelType(_msg->model(i).name());
  imageMsg.models.push_back(modelMsg);
  }
  this->imagePub.publish(imageMsg);
*/
}

int main(int argc, char **argv)
{ gazebo::client::setup(argc, argv);
  ros::init(argc, argv, "LogicalCamBridge");

  ros::NodeHandle n;

  ros::Publisher pub= n.advertise<std_msgs::String>("logCam", 1000);

  ros::Rate loop_rate(10);
  //sensors::SensorPtr genericSensor = sensors::get_sensor("LogCam1::link::logical_camera");
  //sensors::LogicalCameraSensorPtr logicalCamera = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(genericSensor);
  //msgs::LogicalCameraImage sensorOutput = logicalCamera->Image();
  //std::cout << sensorOutput.model_size()<<"\n"; 
  transport::NodePtr node(new transport::Node());
  node->Init();
  transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/LogCam1/link/logical_camera/models", callback);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    gazebo::common::Time::MSleep(20);

	//ros::spinOnce();

  //  loop_rate.sleep();
  }


  return 0;
}

