/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <dvo_ros/camera_base.h>

namespace dvo_ros
{

CameraBase::CameraBase(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
  nh_(nh),
  nh_private_(nh_private),

  //rgb_image_subscriber_(nh, "camera/rgb/image_rect", 1),
  //depth_image_subscriber_(nh, "camera/depth_registered/image_rect_raw", 1),
  //rgb_camera_info_subscriber_(nh, "camera/rgb/camera_info", 1),
  //depth_camera_info_subscriber_(nh, "camera/depth_registered/camera_info", 1),
  rgb_image_subscriber_(nh, "/camera/rgb/input_image", 1),
  depth_image_subscriber_(nh, "/camera/depth_registered/input_image", 1),
  rgb_camera_info_subscriber_(nh, "/camera/rgb/camera_info", 1),
  depth_camera_info_subscriber_(nh, "/camera/depth_registered/camera_info", 1),
  synchronizer_(RGBDWithCameraInfoPolicy(5), rgb_image_subscriber_, depth_image_subscriber_, rgb_camera_info_subscriber_, depth_camera_info_subscriber_),
  //synchronizer_(RGBDWithCameraInfoPolicy(5), rgb_image_subscriber_, depth_image_subscriber_, rgb_camera_info_subscriber_),
  connected(false)
{
    //initializeSubscribersAndPublishers();

    //std::cout << " Connection = " << (connected==true) << std::endl;
 }

#ifdef NOT_DEFINED
    void CameraBase::initializeSubscribersAndPublishers() {

        image_transport::ImageTransport depth_image_transport(nh_);
        image_transport::TransportHints depth_transport_hints("raw", ros::TransportHints(), nh_, "depth_image_transport");
        depth_image_subscriber_.subscribe(depth_image_transport, "depth_topic", 1, depth_transport_hints);

        image_transport::ImageTransport rgb_image_transport(nh_);
        image_transport::TransportHints rgb_transport_hints("raw", ros::TransportHints(), nh_, "rgb_image_transport");
        rgb_image_subscriber_.subscribe(rgb_image_transport, "rgb_topic", 1, rgb_transport_hints);

        rgb_camera_info_subscriber_.subscribe(nh_, "camera_info_topic", 1);

        size_t queue_size = 10;
        typedef message_filters::sync_policies::ApproximateTime
                <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximateSyncPolicyRGBD;
        static message_filters::Synchronizer<ApproximateSyncPolicyRGBD> rgbd_approx_sync(ApproximateSyncPolicyRGBD(queue_size),
                rgb_image_subscriber_, depth_image_subscriber_, rgb_camera_info_subscriber_);
        rgbd_approx_sync.registerCallback(boost::bind(&CameraBase::handleImages,
                this, _1, _2, _3));
        connected = true;
    }
#endif
    
CameraBase::~CameraBase()
{
  stopSynchronizedImageStream();
}

bool CameraBase::isSynchronizedImageStreamRunning()
{
  return connected;
}

void CameraBase::startSynchronizedImageStream()
{
  if(!connected)
  {
    connection = synchronizer_.registerCallback(boost::bind(&CameraBase::handleImages, this, _1, _2, _3, _4));
    connected = true;
  }
}

void CameraBase::stopSynchronizedImageStream()
{
  if(connected)
  {
    connection.disconnect();
    connected = false;
  }
}

} /* namespace dvo_ros */
