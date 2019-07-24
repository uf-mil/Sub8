/*
 * This file was modified from the original version within Gazebo:
 *
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Modifications:
 *
 * Copyright 2018 Nils Bore (nbore@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef GAZEBO_ROS_DEPTH_CAMERA_HH
#define GAZEBO_ROS_DEPTH_CAMERA_HH

// ros stuff
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

// message stuff
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/fill_image.h>
#include <std_msgs/Float64.h>

// gazebo stuff
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/Param.hh>

// dynamic reconfigure stuff
#include <dynamic_reconfigure/server.h>
#include <gazebo_plugins/GazeboRosCameraConfig.h>

// boost stuff
#include <boost/thread/mutex.hpp>

// camera stuff
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

// misc stuff
#include <opencv2/core/core.hpp>
#include <vector>

// mil stuff
//#include <sub8_gazebo/simulated_sonar_ping.h>
#include <mil_blueview_driver/BlueViewPing.h>

namespace gazebo
{
class GazeboRosImageSonar : public SensorPlugin, GazeboRosCameraUtils
{
public:
	GazeboRosImageSonar();

	// destructor
	~GazeboRosImageSonar();

	// load the plugin
	virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

	// Advertise point cloud and depth image
	virtual void Advertise();

protected:
	virtual void OnNewDepthFrame(const float* _image, unsigned int _width, unsigned int _height, unsigned int _depth,
															 const std::string& _format);

	virtual void OnNewRGBPointCloud(const float* _pcd, unsigned int _width, unsigned int _height, unsigned int _depth,
																	const std::string& _format);

	virtual void OnNewImageFrame(const unsigned char* _image, unsigned int _width, unsigned int _height,
															 unsigned int _depth, const std::string& _format);

private:
	void FillPointdCloud(const float* _src);
	void FillDepthImage(const float* _src);

	// Takes the Gazebo depth image and publishes the simulated sonar image
	void ComputeSonarImage(const float* _src);

	// calculates normals from the depth image
	cv::Mat ComputeNormalImage(cv::Mat& depth);

	// Creates and pubs 'Multibeam' image using a depthmap and the computed normals, adds 
	// sonar params for the signal to noise ratio SNR(sonar equation)
	cv::Mat ConstructSonarImage(cv::Mat& depth, cv::Mat& normals);

	// Publishes the 'raw_scan' sonar image, applies intensities and creates a ping message
	cv::Mat ConstructScanImage(cv::Mat& depth, cv::Mat& SNR);

//TODO: maybe make it it's own function
	//cv::Mat ConstructPingMsg();

	// Pubishes the pretty sonar image to be viewed in Rviz. Adds gridlines, applies colormap
	cv::Mat ConstructVisualScanImage(cv::Mat& raw_scan);

	void ApplySpeckleNoise(cv::Mat& scan, float fov);
	void ApplyMedianFilter(cv::Mat& scan);

	//TODO: Make better, otherwise it remains unused
	void ApplySmoothing(cv::Mat& scan, float fov);
	

	// Keep track of number of connctions for point clouds
	int point_cloud_connect_count_;
	void PointCloudConnect();
	void PointCloudDisconnect();

	// Keep track of number of connctions for point clouds
	int depth_image_connect_count_;
	void DepthImageConnect();
	void DepthImageDisconnect();
	void NormalImageConnect();
	void NormalImageDisconnect();
	void MultibeamImageConnect();
	void MultibeamImageDisconnect();
	void SonarImageConnect();
	void SonarImageDisconnect();
	void RawSonarImageConnect();
	void RawSonarImageDisconnect();

	common::Time last_depth_image_camera_info_update_time_;

	bool FillPointCloudHelper(sensor_msgs::PointCloud2& point_cloud_msg, uint32_t rows_arg, uint32_t cols_arg,
														uint32_t step_arg, void* data_arg);
	bool FillDepthImageHelper(sensor_msgs::Image& image_msg, uint32_t rows_arg, uint32_t cols_arg, uint32_t step_arg,
														void* data_arg);

	// Publishers. If not wanted most can be commented out 
	ros::Publisher point_cloud_pub_;
	ros::Publisher depth_image_pub_;
	ros::Publisher normal_image_pub_;
	ros::Publisher multibeam_image_pub_;
	ros::Publisher sonar_image_pub_;
	ros::Publisher raw_sonar_image_pub_;
	ros::Publisher sonar_ping_msg_pub_;

//public:
	//sub8_gazebo::simulated_sonar_pingPtr ping_ptr;
	mil_blueview_driver::BlueViewPingPtr ping_ptr;

//private:
	// PointCloud2 point cloud message
	sensor_msgs::PointCloud2 point_cloud_msg_;
	sensor_msgs::Image depth_image_msg_;
	sensor_msgs::Image normal_image_msg_;
	sensor_msgs::Image multibeam_image_msg_;
	sensor_msgs::Image sonar_image_msg_;
	sensor_msgs::Image raw_sonar_image_msg_;
	double point_cloud_cutoff_;

	// Ros topic names
	std::string point_cloud_topic_name_;
	std::string sonar_ping_topic_name_;

	void InfoConnect();

	void InfoDisconnect();

	using GazeboRosCameraUtils::PublishCameraInfo;

protected:
	// TODO: Determine if worthwhile or cut
	virtual void PublishCameraInfo();

private:
	std::string depth_image_topic_name_;
	std::string depth_image_camera_info_topic_name_;
	std::default_random_engine generator;
	int depth_info_connect_count_;

	void DepthInfoConnect();
	void DepthInfoDisconnect();

	// Ping intensity extremes for adaptive thresholding. *TBC..
	double ping_scan_max_;
	double ping_scan_min_;

	int sonar_ping_count_;
	void SonarPingConnect();
	void SonarPingDisconnect();

	// overload with our own
	common::Time depth_sensor_update_time_;
	event::ConnectionPtr load_connection_;
	event::ConnectionPtr newDepthFrameConnection;
	event::ConnectionPtr newRGBPointCloudConnection;
	event::ConnectionPtr newImageFrameConnection;

protected:
	ros::Publisher depth_image_camera_info_pub_;

	// from DepthCameraPlugin
	unsigned int width, height, depth;
	std::string format;

	// precomputed things for the forward-looking sonar
	cv::Mat dist_matrix_;

	// Remove if not using smothing
	std::vector<std::vector<int> > angle_range_indices_;
	std::vector<int> angle_nbr_indices_;

	sensors::DepthCameraSensorPtr parentSensor;
	rendering::DepthCameraPtr depthCamera;
};  // end GazeboRosImageSonar
}  // end namespace gazebo
#endif
