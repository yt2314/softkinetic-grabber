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
//#include <bind>

using namespace DepthSense;
//using namespace std;

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;
AudioNode g_anode;

uint32_t g_aFrames = 0;
uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::SKGrabber::SKGrabber ()
  : running_ (false)
  , fps_ (0.0f)
  , rgb_array_size_(0)
  , depth_buffer_size_(0)
  , image_width_(0)
  , image_height_(0)
  , depth_width_(0)
  , depth_height_(0)
  , has_color_data_(false)
  , has_depth_data_(false)
{
  point_cloud_signal_      = createSignal<sig_cb_sk_point_cloud> ();
  point_cloud_rgb_signal_  = createSignal<sig_cb_sk_point_cloud_rgb> ();
  point_cloud_rgba_signal_ = createSignal<sig_cb_sk_point_cloud_rgba> ();
  color_image_signal_      = createSignal<sig_cb_sk_color_image> ();
  depth_image_signal_      = createSignal<sig_cb_sk_depth_image> ();
  pose_signal_             = createSignal<sig_cb_sk_camera_pose> ();
  intrinsics_              = Eigen::Matrix3f::Identity ();
  extrinsics_              = Eigen::Matrix4f::Identity ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::SKGrabber::~SKGrabber () throw ()
{
  stop ();

  disconnect_all_slots<sig_cb_sk_point_cloud> ();
  disconnect_all_slots<sig_cb_sk_point_cloud_rgb> ();
  disconnect_all_slots<sig_cb_sk_point_cloud_rgba> ();
}


// /*----------------------------------------------------------------------------*/
// // New audio sample event handler
// void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data)
// {
//     printf("A#%u: %d\n",g_aFrames,data.audioData.size());
//     g_aFrames++;
// }

/*----------------------------------------------------------------------------*/
// New color sample event handler
void pcl::SKGrabber::onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
//  printf("C#%d: %d\n",g_cFrames,data.colorMap.size());

  if (has_color_data_)
    return;

  int32_t w, h;
  FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);
  //printf("C# width %d height %d\n",w,h);
  image_width_= w;
  image_height_ = h;
  if (rgb_array_size_ < image_width_ * image_height_ * 3)
  {
    rgb_array_size_ = image_width_ * image_height_ * 3;
    rgb_array_.reset (new unsigned char [rgb_array_size_]);
    lockedRGBImage_.setDimension(w,h);
  }

  unsigned char* buffer = rgb_array_.get();
  unsigned char* colorMap = reinterpret_cast<unsigned char *>(const_cast<unsigned char*>(static_cast<const unsigned char  *>(data.colorMap)));
  std::memcpy(buffer, colorMap, image_width_ * image_height_ * 3);
  //std::string filename = "color.png";
  //pcl::io::saveRgbPNGFile (filename, (unsigned char*)buffer, image_width_, image_height_);

  {
    boost::mutex::scoped_lock lock(data_ready_mutex_);
    has_color_data_ = true;
  }
  g_cFrames++;

  //printf("new color done\n");
}

void pcl::SKGrabber::newColorSampleCallback(ColorNode node, ColorNode::NewSampleReceivedData data)
{
  getInstance().onNewColorSample(node, data);
}

/*----------------------------------------------------------------------------*/
// New depth sample event handler
void pcl::SKGrabber::onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
//  printf("Z#%d: %d\n",g_dFrames,data.vertices.size());

  //if (has_depth_data_) printf("has depth data\n"); else printf("no depth data\n");
  //if (has_color_data_) printf("has color data\n"); else printf("no color data\n");
  // has data that haven't been fetched or the color data is not ready
  if (has_depth_data_ || !has_color_data_)
    return;

  if (data.depthMapFloatingPoint==0) 
  {
    printf("no depthmap\n");
    return;
  }

  {
    boost::mutex::scoped_lock lock(data_ready_mutex_);

    //if (!proj_helper_)
    //  proj_helper_ = new ProjectionHelper(data.stereoCameraParameters);

    //print camera info
    printf("extrinsic param\n");
    printf(" %g, %g, %g, %g, \n %g, %g, %g, %g\n %g, %g, %g, %g\n", 
      data.stereoCameraParameters.extrinsics.r11, data.stereoCameraParameters.extrinsics.r12,  data.stereoCameraParameters.extrinsics.r13, data.stereoCameraParameters.extrinsics.t1, 
      data.stereoCameraParameters.extrinsics.r21, data.stereoCameraParameters.extrinsics.r22,  data.stereoCameraParameters.extrinsics.r23, data.stereoCameraParameters.extrinsics.t2,
      data.stereoCameraParameters.extrinsics.r31, data.stereoCameraParameters.extrinsics.r32,  data.stereoCameraParameters.extrinsics.r33,
      data.stereoCameraParameters.extrinsics.t3);


    //Eigen::Matrix3f R;// = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    extrinsics_ << data.stereoCameraParameters.extrinsics.r11, data.stereoCameraParameters.extrinsics.r12,  data.stereoCameraParameters.extrinsics.r13, data.stereoCameraParameters.extrinsics.t1,
      data.stereoCameraParameters.extrinsics.r21, data.stereoCameraParameters.extrinsics.r22,  data.stereoCameraParameters.extrinsics.r23, data.stereoCameraParameters.extrinsics.t2,
      data.stereoCameraParameters.extrinsics.r31, data.stereoCameraParameters.extrinsics.r32,  data.stereoCameraParameters.extrinsics.r33, data.stereoCameraParameters.extrinsics.t3,
      0.0f, 0.0f, 0.0f, 1.0f;
    //extrinsics_.transposeInPlace();
    printf("end of extrinsics\n");
    printf("%g, %g, %g,\n%g, %g, %g\n", data.stereoCameraParameters.depthIntrinsics.fx, 0.0f, data.stereoCameraParameters.depthIntrinsics.cx, 
      0.0f, data.stereoCameraParameters.depthIntrinsics.fy, data.stereoCameraParameters.depthIntrinsics.cy);
    //Eigen::Vector3f t;
    //t << data.stereoCameraParameters.extrinsics.t1, data.stereoCameraParameters.extrinsics.t2,  data.stereoCameraParameters.extrinsics.t3;
    intrinsics_ << data.stereoCameraParameters.depthIntrinsics.fx, 0.0f, data.stereoCameraParameters.depthIntrinsics.cx, 
      0.0f, data.stereoCameraParameters.depthIntrinsics.fy, data.stereoCameraParameters.depthIntrinsics.cy,
      0.0f, 0.0f, 1.0f;
    //intrinsics_.transposeInPlace();
    printf("end of intrinsics\n");

    //Eigen::Vector3f t = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);
    //Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);
    //Eigen::Vector3f c = - R * t;
    //pose_ = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

    int32_t w, h;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);
    //printf("Z# width %d height %d\n",w,h);

    depth_width_ = w;
    depth_height_ = h;
    if (depth_buffer_size_ < w * h)
    {
      depth_buffer_size_ = w * h;
      depth_buffer_.reset (new float [depth_buffer_size_]);
      //printf("reset depth buffer\n");
      lockedDepthImage_.setDimension(w, h);
    }

    std::memcpy(depth_buffer_.get(), data.depthMapFloatingPoint, depth_buffer_size_*sizeof(float));

    //printf("depth copy data\n");

    cloud_ = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA> (w, h));
    cloud_->is_dense = false;
    if (cloud_->points.size() != w*h)
      cloud_->points.resize(w * h);

    //printf("cloud reset\n");
    unsigned char* rgb_image_data = rgb_array_.get();
    float* depth_image_data = depth_buffer_.get();
    Vertex p3DPoints[1];
    Point2D p2DPoints[1];

    const float nan_value = std::numeric_limits<float>::quiet_NaN ();

    for (int j = 0, k = 0; j < h; j++)
    {
      for (int i = 0; i < w; i++, k++)
      {
        cloud_->points[k].x = -data.verticesFloatingPoint[k].x;
        cloud_->points[k].y = data.verticesFloatingPoint[k].y;
        if (data.verticesFloatingPoint[k].z == 32001)
        {
          cloud_->points[k].z = 0;
        }
        else
        {
          cloud_->points[k].z = data.verticesFloatingPoint[k].z;
        }
        // Saturated pixels on depthMapFloatingPoint have -1 value, but on openni are NaN
        if (data.depthMapFloatingPoint[k] < 0.0)
        {
          depth_image_data[k] = nan_value;
        }


        // unsigned char r = 255;
        // unsigned char g = 255;
        // unsigned char b = 255;

        // p3DPoints[0] = data.vertices[k];
        // g_pProjHelper->get2DCoordinates(p3DPoints, p2DPoints, 2, CAMERA_PLANE_COLOR);
        // int x_pos = (int)p2DPoints[0].x;
        // int y_pos = (int)p2DPoints[0].y;
        // if (y_pos >= 0 && y_pos <= image_height_ && x_pos >= 0 && x_pos <= image_width_)
        // {
        //   // Within bounds: depth fov is significantly wider than color's
        //   // one, so there are black points in the borders of the pointcloud
        //   unsigned char* rgb_ptr = rgb_image_data + 3 * (y_pos*image_width_+x_pos);
        //   cloud_->points[k].b = rgb_ptr[0];
        //   cloud_->points[k].g = rgb_ptr[1];
        //   cloud_->points[k].r = rgb_ptr[2];
        // }
        // else
        // {
        //   cloud_->points[k].b = nan_value;
        //   cloud_->points[k].g = nan_value;
        //   cloud_->points[k].r = nan_value;          
        // }
      }
    }

    //saveDepthImage(filename, depth_image_data, depth_width_, depth_height_);
    /*
    std::string filename = "depth.dat";
    std::ofstream ofs;
    ofs.open(filename.c_str());
    for (int j = 0, k = 0; j < h; j++)
    {
      for (int i = 0; i < w; i++, k++)
      {
        ofs << cloud_->points[k].z;//depth_image_data[k];
        ofs << " ";
      }
      ofs << "\n";
    }
    ofs.close();
    */

    has_depth_data_ = true;
  }

  g_dFrames++;

  //printf("new depth done\n");
}

void pcl::SKGrabber::newDepthSampleCallback(DepthNode node, DepthNode::NewSampleReceivedData data)
{
  getInstance().onNewDepthSample(node, data);
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

/*----------------------------------------------------------------------------*/
void pcl::SKGrabber::configureDepthNode()
{
    g_dnode.newSampleReceivedEvent().connect(newDepthSampleCallback);

    DepthNode::Configuration config = g_dnode.getConfiguration();
    config.frameFormat = FRAME_FORMAT_QVGA;
    config.framerate = 25;
    config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = true;
    
    g_dnode.setEnableVertices(true);
    g_dnode.setEnableVerticesFloatingPoint(true);
    g_dnode.setEnableDepthMapFloatingPoint(true);

    try 
    {
        g_context.requestControl(g_dnode,0);

        g_dnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("TimeoutException\n");
    }

}

/*----------------------------------------------------------------------------*/
void pcl::SKGrabber::configureColorNode()
{
    // connect new color sample handler
    g_cnode.newSampleReceivedEvent().connect(newColorSampleCallback);

    ColorNode::Configuration config = g_cnode.getConfiguration();
    config.frameFormat = FRAME_FORMAT_VGA;
    config.compression = COMPRESSION_TYPE_MJPEG;
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
    config.framerate = 25;

    g_cnode.setEnableColorMap(true);

    try 
    {
        g_context.requestControl(g_cnode,0);

        g_cnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("TimeoutException\n");
    }
}

/*----------------------------------------------------------------------------*/
void pcl::SKGrabber::configureNode(Node node)
{
    if ((node.is<DepthNode>())&&(!g_dnode.isSet()))
    {
        g_dnode = node.as<DepthNode>();
        configureDepthNode();
        g_context.registerNode(node);
    }

    if ((node.is<ColorNode>())&&(!g_cnode.isSet()))
    {
        g_cnode = node.as<ColorNode>();
        configureColorNode();
        g_context.registerNode(node);
    }

    // if ((node.is<AudioNode>())&&(!g_anode.isSet()))
    // {
    //     g_anode = node.as<AudioNode>();
    //     configureAudioNode();
    //     g_context.registerNode(node);
    // }
}

/*----------------------------------------------------------------------------*/
void pcl::SKGrabber::onNodeConnected(Device device, Device::NodeAddedData data)
{
    configureNode(data.node);
}

void pcl::SKGrabber::nodeConnectedCallback(Device device, Device::NodeAddedData data)
{
    getInstance().onNodeConnected(device, data);
}


/*----------------------------------------------------------------------------*/
void pcl::SKGrabber::onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
    // if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == g_anode))
    //     g_anode.unset();
    if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
        g_cnode.unset();
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
        g_dnode.unset();
    printf("Node disconnected\n");
}

void pcl::SKGrabber::nodeDisconnectedCallback(Device device, Device::NodeRemovedData data)
{
    getInstance().onNodeDisconnected(device, data);
}

/*----------------------------------------------------------------------------*/
void pcl::SKGrabber::onDeviceConnected(Context context, Context::DeviceAddedData data)
{
    if (!g_bDeviceFound)
    {
        data.device.nodeAddedEvent().connect(&nodeConnectedCallback);
        data.device.nodeRemovedEvent().connect(&nodeDisconnectedCallback);
        g_bDeviceFound = true;
    }
}

void pcl::SKGrabber::deviceConnectedCallback(Context context, Context::DeviceAddedData data)
{
    getInstance().onDeviceConnected(context, data);
}

/*----------------------------------------------------------------------------*/
void pcl::SKGrabber::onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
    g_bDeviceFound = false;
    printf("Device disconnected\n");
}

void pcl::SKGrabber::deviceDisconnectedCallback(Context context, Context::DeviceRemovedData data)
{
    getInstance().onDeviceDisconnected(context, data);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pcl::SKGrabber::skrun()
{
  g_context.run();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::SKGrabber::init ()
{
  // enable rgb and 3d data and initialize
  //pp_.EnableImage (PXCImage::COLOR_FORMAT_RGB32);
  //pp_.EnableImage (PXCImage::COLOR_FORMAT_VERTICES);
  //pp_.Init ();

  g_context = Context::create("localhost");

  g_context.deviceAddedEvent().connect(deviceConnectedCallback);
  g_context.deviceRemovedEvent().connect(deviceDisconnectedCallback);

  // Get the list of currently connected devices
  std::vector<Device> da = g_context.getDevices();

  // We are only interested in the first device
  if (da.size() >= 1)
  {
    g_bDeviceFound = true;

    da[0].nodeAddedEvent().connect(nodeConnectedCallback);
    da[0].nodeRemovedEvent().connect(nodeDisconnectedCallback);

    std::vector<Node> na = da[0].getNodes();
    
    printf("Found %u nodes\n",na.size());
    
    for (int n = 0; n < (int)na.size();n++)
      configureNode(na[n]);
  }

  g_context.startNodes();

  skrun_thread_ = boost::thread (&pcl::SKGrabber::skrun, this);

  //g_context.run();


  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::SKGrabber::close ()
{
  //pp_.Close ();
  g_context.stopNodes();

  if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
  if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
  if (g_anode.isSet()) g_context.unregisterNode(g_anode);

  if (g_pProjHelper)
    delete g_pProjHelper;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::SKGrabber::start ()
{
  running_ = true;
  printf("before creating thread\n");
  grabber_thread_ = boost::thread (&pcl::SKGrabber::processGrabbing, this); 
  printf("after creating thread\n");

  init ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::SKGrabber::stop ()
{
  running_ = false;
  grabber_thread_.join ();

  close ();
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
  fps_mutex_.lock ();
  float fps = fps_;
  fps_mutex_.unlock ();

  return (fps);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::SKGrabber::processGrabbing ()
{
  printf("start grabbing...\n");
  pcl::StopWatch stop_watch;
  std::queue<double> capture_time_queue;
  double total_time = 0.0f;

  stop_watch.reset ();

  //bool continue_grabbing = true;
  while (running_) 
  {
    // boost::mutex::scoped_lock lock(data_ready_mutex_);
    // has data that haven't been fetched
    printf(".");
    if (has_depth_data_ == false || has_color_data_ == false) {
      boost::this_thread::sleep (boost::posix_time::milliseconds (10));
      continue;
    }

    //printf("\ngrabbing\n");
    // publish
    int w = depth_width_;
    int h = depth_height_;

    {
      boost::mutex::scoped_lock lock(data_ready_mutex_);

      if (num_slots<sig_cb_sk_camera_pose> () > 0)
      {
        pose_signal_->operator() (intrinsics_, extrinsics_);
      }

      if (num_slots<sig_cb_sk_color_image> () > 0 && !lockedRGBImage_.hasData())
      {
        lockedRGBImage_.setData(rgb_array_.get());
        color_image_signal_->operator() (lockedRGBImage_);
      }

      if (num_slots<sig_cb_sk_depth_image> () > 0 && !lockedDepthImage_.hasData())
      {
        lockedDepthImage_.setData(depth_buffer_.get());
        depth_image_signal_->operator() (lockedDepthImage_);
      }

      if (num_slots<sig_cb_sk_point_cloud_rgba> () > 0)
      {
        point_cloud_rgba_signal_->operator() (cloud_);
      }

      if (num_slots<sig_cb_sk_point_cloud_rgb> () > 0)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
        cloud_->resize (w*h);
        cloud_->width = w;
        cloud_->height = h;
        cloud_->is_dense = false;

        pcl::copyPointCloud<pcl::PointXYZRGBA, pcl::PointXYZRGB> (*cloud_, *tmp_cloud);

        point_cloud_rgb_signal_->operator() (tmp_cloud);
      }
      if (num_slots<sig_cb_sk_point_cloud> () > 0)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ> ());
        cloud_->resize (w*h);
        cloud_->width = w;
        cloud_->height = h;
        cloud_->is_dense = false;

        pcl::copyPointCloud<pcl::PointXYZRGBA, pcl::PointXYZ> (*cloud_, *cloud_tmp);

        point_cloud_signal_->operator() (cloud_tmp);
      }


      const double capture_time = stop_watch.getTimeSeconds ();
      total_time += capture_time;

      capture_time_queue.push (capture_time);

      if (capture_time_queue.size () >= 30)
      {
        double removed_time = capture_time_queue.front ();
        capture_time_queue.pop ();

        total_time -= removed_time;
      }

      has_color_data_ = false;
      has_depth_data_ = false;

    }
    {
      fps_mutex_.lock ();
      fps_ = static_cast<float> (total_time / capture_time_queue.size ());
      fps_mutex_.unlock ();

    }
    printf("end grabbing\n");
  }
}

#endif
