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

#ifndef __PCL_IO_SK_GRABBER__
#define __PCL_IO_SK_GRABBER__

#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/common/synchronizer.h>
#include <pcl/point_cloud.h>

#include <boost/thread.hpp>  

//#include "pxcsmartptr.h"
//#include "pxcsession.h"
//#include "util_capture.h"
//#include "util_pipeline.h"
//#include "util_pipeline_raw.h"
//#include "util_render.h"
//#include <util_capture.h>
//#include <pxcprojection.h>
//#include <pxcmetadata.h>

#include <string>
#include <deque>
#include <DepthSense.hxx>


namespace pcl
{
  struct PointXYZ;
  struct PointXYZRGB;
  struct PointXYZRGBA;
  struct PointXYZI;
  template <typename T> class PointCloud;


  template <typename T, bool rgb=false>
  struct LockedImage
  {
    LockedImage(int c, int r) : width_(c), height_(r)
    {
      length_ = rgb ? c*r*3 : c*r;
    }
    
    LockedImage()
    {
      width_ = 0;
      height_ = 0;
      length_ = 0;
    }

    void setDimension(int c, int r)
    {
      width_ = c;
      height_ = r;
      int newlength = rgb ? c*r*3 : c*r;
      if (length_ != newlength)
      {
        length_ = newlength;
        image_array_.reset();
      }
    }

    void setData(T* data)
    {
      boost::mutex::scoped_lock lock(data_access_mutex_);
      if (!image_array_)
      {
        image_array_.reset(new T[length_]);
      }
      T* buffer = image_array_.get();
      memcpy(buffer, data, length_*sizeof(T));
    }

    bool hasData()
    {
      boost::mutex::scoped_lock lock(data_access_mutex_);
      return image_array_;      
    }

    boost::shared_array<T> getData()
    {
       boost::mutex::scoped_lock lock(data_access_mutex_);
       boost::shared_array<T> temp_array;
       temp_array.swap (image_array_); //here we set image_array_ to null

       return temp_array;
    }

    int width_;
    int height_;
    int length_;
    mutable boost::mutex data_access_mutex_;
    boost::shared_array<T> image_array_;
  };

  typedef LockedImage<unsigned char, true> LockedRGBImage;
  typedef LockedImage<float, false> LockedDepthImage;


  /** \brief Grabber for SK devices
    * \author Stefan Holzer <holzers@in.tum.de>
    * \author Yongqiang Tan
    * \ingroup io
    */
  class PCL_EXPORTS SKGrabber : public Grabber
  {
    public:

      /** \brief Supported modes for grabbing from a PXC device. */
      typedef enum
      {
        SK_Default_Mode = 0, 
      } Mode;

      //typedef pcl::gpu::PixelRGB  PixelRGB;

      //define callback signature typedefs
      typedef void (sig_cb_sk_point_cloud) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
      typedef void (sig_cb_sk_point_cloud_rgb) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&);
      typedef void (sig_cb_sk_point_cloud_rgba) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&);
      typedef void (sig_cb_sk_point_cloud_i) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);
      typedef void (sig_cb_sk_color_image) (LockedRGBImage &);
      typedef void (sig_cb_sk_depth_image) (LockedDepthImage &);
      typedef void (sig_cb_sk_camera_pose) (const Eigen::Matrix3f &, const Eigen::Matrix4f &);
      //typedef void (sig_cb_sk_depth_image) (const boost::shared_array<float> &);

    public:

      /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
      virtual ~SKGrabber () throw ();

      /** \brief Start the data acquisition. */
      virtual void
      start ();

      /** \brief Stop the data acquisition. */
      virtual void
      stop ();

      /** \brief Check if the data acquisition is still running. */
      virtual bool
      isRunning () const;

      /** \brief Returns the name of the grabber. */
      virtual std::string
      getName () const;

      /** \brief Obtain the number of frames per second (FPS). */
      virtual float 
      getFramesPerSecond () const;

      virtual void 
      onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data);

      virtual void 
      onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);

      static SKGrabber& getInstance() {
        static SKGrabber instance;
        return instance;
      }

      static void nodeConnectedCallback(DepthSense::Device device, DepthSense::Device::NodeAddedData data);
      static void nodeDisconnectedCallback(DepthSense::Device device, DepthSense::Device::NodeRemovedData data);
      static void deviceConnectedCallback(DepthSense::Context context, DepthSense::Context::DeviceAddedData data);
      static void deviceDisconnectedCallback(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data);
      static void newColorSampleCallback(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data);
      static void newDepthSampleCallback(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);

    protected:

      /** \brief Initializes the PXC grabber and the grabbing pipeline. */
      bool
      init ();

      /** \brief Closes the grabbing pipeline. */
      void
      close ();

      /** \brief Continously asks for data from the device and publishes it if available. */
      void
      processGrabbing ();

      void skrun();


      void configureColorNode();
      void configureDepthNode();
      void configureNode(DepthSense::Node);

      void onNodeConnected(DepthSense::Device device, DepthSense::Device::NodeAddedData data);
      void onNodeDisconnected(DepthSense::Device device, DepthSense::Device::NodeRemovedData data);
      void onDeviceConnected(DepthSense::Context context, DepthSense::Context::DeviceAddedData data);
      void onDeviceDisconnected(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data);

      // signals to indicate whether new clouds are available
      boost::signals2::signal<sig_cb_sk_point_cloud>* point_cloud_signal_;
      //boost::signals2::signal<sig_cb_fotonic_point_cloud_i>* point_cloud_i_signal_;
      boost::signals2::signal<sig_cb_sk_point_cloud_rgb>* point_cloud_rgb_signal_;
      boost::signals2::signal<sig_cb_sk_point_cloud_rgba>* point_cloud_rgba_signal_;

      boost::signals2::signal<sig_cb_sk_color_image>* color_image_signal_;
      boost::signals2::signal<sig_cb_sk_depth_image>* depth_image_signal_;
      boost::signals2::signal<sig_cb_sk_camera_pose>* pose_signal_;

    protected:
      /** \brief Constructor */
      SKGrabber ();
      SKGrabber (SKGrabber const &);
      SKGrabber& operator= (SKGrabber const &);

      // utiliy object for accessing PXC camera
      //UtilPipeline pp_;
      // indicates whether grabbing is running
      int image_width_, image_height_;
      int depth_width_, depth_height_;
      mutable bool running_;

      // TODO condition variable may be better for synchronizing
      mutable bool has_depth_data_;
      mutable bool has_color_data_;
      mutable boost::mutex data_ready_mutex_;


      // FPS computation
      mutable float fps_;
      mutable boost::mutex fps_mutex_;

      mutable unsigned int rgb_array_size_;
      mutable unsigned int depth_buffer_size_;
      mutable boost::shared_array<unsigned char> rgb_array_;
      mutable boost::shared_array<float> depth_buffer_;

      // thread where the grabbing takes place
      boost::thread grabber_thread_;
      boost::thread skrun_thread_;

      LockedRGBImage lockedRGBImage_;
      LockedDepthImage lockedDepthImage_;

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;

      Eigen::Matrix3f intrinsics_;
      Eigen::Matrix4f extrinsics_;

      //static SKGrabber* instance;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace pcl
#endif // __PCL_IO_SK_GRABBER__
#endif // HAVE_SOFTKINETIC
