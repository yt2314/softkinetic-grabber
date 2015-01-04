/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/pcl_config.h>
#define HAVE_SOFTKINETIC
#ifdef HAVE_SOFTKINETIC

//#include <pcl/io/softkinetic_grabber.h>
#include "softkinetic_grabber.h"
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/io.h>
#include <pcl/io/png_io.h>
#include <pcl/console/print.h>
#include <pcl/io/boost.h>
#include <pcl/exceptions.h>
//#include <pcl/gpu/kinfu/pixel_rgb.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <iostream>
#include <fstream>
#include <queue>
#include <vector>
#include "softkinetic_device.h"

//#include <bind>

using namespace DepthSense;
//using namespace std;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::SKGrabber::SKGrabber ()
  : running_ (false)
  // , fps_ (0.0f)
  // , rgb_array_size_(0)
  // , depth_buffer_size_(0)
  // , image_width_(0)
  // , image_height_(0)
  // , depth_width_(0)
  // , depth_height_(0)
  // , has_color_data_(false)
  // , has_depth_data_(false)
{
  point_cloud_signal_      = createSignal<sig_cb_sk_point_cloud> ();
  //point_cloud_rgb_signal_  = createSignal<sig_cb_sk_point_cloud_rgb> ();
  point_cloud_rgba_signal_ = createSignal<sig_cb_sk_point_cloud_rgba> ();
  color_image_signal_      = createSignal<sig_cb_sk_color_image> ();
  depth_image_signal_      = createSignal<sig_cb_sk_depth_image> ();
  //pose_signal_             = createSignal<sig_cb_sk_camera_pose> ();

  image_callback_handle_ = SoftKineticDevice::getInstance().registerImageCallback (&SKGrabber::imageCallback, *this);
  depth_callback_handle_ = SoftKineticDevice::getInstance().registerDepthCallback (&SKGrabber::depthCallback, *this);
  cloud_callback_handle_ = SoftKineticDevice::getInstance().registerCloudCallback (&SKGrabber::cloudCallback, *this);
  //cloud_rgba_callback_handle_ = SoftKineticDevice::getInstance().registerCloudCallback (&SKGrabber::cloudRGBACallback, *this);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::SKGrabber::~SKGrabber () throw ()
{
  stop ();

  // unregister callbacks
  SoftKineticDevice::getInstance().unregisterDepthCallback (depth_callback_handle_);
  SoftKineticDevice::getInstance().unregisterImageCallback (image_callback_handle_);
  SoftKineticDevice::getInstance().unregisterCloudCallback (cloud_callback_handle_);

  disconnect_all_slots<sig_cb_sk_point_cloud> ();
  //disconnect_all_slots<sig_cb_sk_point_cloud_rgb> ();
  disconnect_all_slots<sig_cb_sk_point_cloud_rgba> ();
  disconnect_all_slots<sig_cb_sk_color_image> ();
  disconnect_all_slots<sig_cb_sk_depth_image> ();
  //disconnect_all_slots<sig_cb_sk_camera_pose> ();

}


// /*----------------------------------------------------------------------------*/
// // New audio sample event handler
// void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data)
// {
//     printf("A#%u: %d\n",g_aFrames,data.audioData.size());
//     g_aFrames++;
// }

void
pcl::SKGrabber::imageCallback (boost::shared_ptr<softkinetic_wrapper::Image> image, void*)
{
  // if (num_slots<sig_cb_openni_point_cloud_rgb>   () > 0 ||
  //     num_slots<sig_cb_openni_point_cloud_rgba>  () > 0 ||
  //     num_slots<sig_cb_openni_image_depth_image> () > 0)
  //   rgb_sync_.add0 (image, image->getTimeStamp ());

  if (color_image_signal_->num_slots () > 0)
    color_image_signal_->operator()(image);
}

void
pcl::SKGrabber::cloudCallback (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, void*)
{
  if (point_cloud_signal_->num_slots () > 0)
    point_cloud_signal_->operator()(cloud);
}

void
pcl::SKGrabber::cloudRGBACallback (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, void*)
{
  if (point_cloud_rgba_signal_->num_slots () > 0)
    point_cloud_rgba_signal_->operator()(cloud);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::SKGrabber::depthCallback (boost::shared_ptr<softkinetic_wrapper::DepthImage> depth_image, void*)
{
  if (depth_image_signal_->num_slots () > 0)
    depth_image_signal_->operator()(depth_image);

}


// /*----------------------------------------------------------------------------*/
// void pcl::SKGrabber::configureAudioNode()
// {
//     g_anode.newSampleReceivedEvent().connect(&onNewAudioSample);

//     AudioNode::Configuration config = g_anode.getConfiguration();
//     config.sampleRate = 44100;

//     try 
//     {
//         g_context.requestControl(g_anode,0);

//         g_anode.setConfiguration(config);
        
//         g_anode.setInputMixerLevel(0.5f);
//     }
//     catch (ArgumentException& e)
//     {
//         printf("Argument Exception: %s\n",e.what());
//     }
//     catch (UnauthorizedAccessException& e)
//     {
//         printf("Unauthorized Access Exception: %s\n",e.what());
//     }
//     catch (ConfigurationException& e)
//     {
//         printf("Configuration Exception: %s\n",e.what());
//     }
//     catch (StreamingException& e)
//     {
//         printf("Streaming Exception: %s\n",e.what());
//     }
//     catch (TimeoutException&)
//     {
//         printf("TimeoutException\n");
//     }
// }



// void
// pcl::SKGrabber::close ()
// {
//   SoftKineticDevice::getInstance().close();
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::SKGrabber::start ()
{
  SoftKineticDevice::getInstance().start();
  running_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::SKGrabber::stop ()
{
  SoftKineticDevice::getInstance().stop();
  running_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::SKGrabber::isRunning () const
{
  return (running_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::SKGrabber::getName () const
{
  return (std::string ("SKGrabber"));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::SKGrabber::getFramesPerSecond () const
{
  // fps_mutex_.lock ();
  // float fps = fps_;
  // fps_mutex_.unlock ();

  // return (fps);
  SoftKineticDevice::getInstance().getFramesPerSecond();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif
