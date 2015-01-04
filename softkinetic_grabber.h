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

#include <boost/thread.hpp>  
#include "softkinetic_image.h"
#include "softkinetic_device.h"

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
      typedef void (sig_cb_sk_point_cloud) (pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
      //typedef void (sig_cb_sk_point_cloud_rgb) (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);
      typedef void (sig_cb_sk_point_cloud_rgba) (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr );
      //typedef void (sig_cb_sk_point_cloud_i) (pcl::PointCloud<pcl::PointXYZI>::ConstPtr &);
      typedef void (sig_cb_sk_color_image) (boost::shared_ptr<softkinetic_wrapper::Image> &);
      typedef void (sig_cb_sk_depth_image) (boost::shared_ptr<softkinetic_wrapper::DepthImage> &);
      //typedef void (sig_cb_sk_camera_pose) (const std::pair<Eigen::Matrix3f, Eigen::Matrix4f> &);
      //typedef void (sig_cb_sk_depth_image) (const boost::shared_array<float> &);

    public:
      SKGrabber ();

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

      static SKGrabber& getInstance() {
        static SKGrabber instance;
        return instance;
      }

      /** \brief RGB image callback. */
      virtual void
      imageCallback (boost::shared_ptr<softkinetic_wrapper::Image> image, void* cookie);

      /** \brief Depth image callback. */
      virtual void
      depthCallback (boost::shared_ptr<softkinetic_wrapper::DepthImage> depth_image, void* cookie);

      Eigen::Matrix3f getIntrinsicParameters() const { return SoftKineticDevice::getInstance().getIntrinsicParameters(); }
      Eigen::Matrix4f getExtrinsicParameters() const { return SoftKineticDevice::getInstance().getExtrinsicParameters(); }

      void cloudCallback (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, void*);
      void cloudRGBACallback (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, void*);
    protected:

      /** \brief Continously asks for data from the device and publishes it if available. */

      // signals to indicate whether new clouds are available
      boost::signals2::signal<sig_cb_sk_point_cloud>* point_cloud_signal_;
      //boost::signals2::signal<sig_cb_fotonic_point_cloud_i>* point_cloud_i_signal_;
      //boost::signals2::signal<sig_cb_sk_point_cloud_rgb>* point_cloud_rgb_signal_;
      boost::signals2::signal<sig_cb_sk_point_cloud_rgba>* point_cloud_rgba_signal_;

      boost::signals2::signal<sig_cb_sk_color_image>* color_image_signal_;
      boost::signals2::signal<sig_cb_sk_depth_image>* depth_image_signal_;
      //boost::signals2::signal<sig_cb_sk_camera_pose>* pose_signal_;

    protected:
      /** \brief Constructor */
      SKGrabber (SKGrabber const &);
      SKGrabber& operator= (SKGrabber const &);


      pcl::SoftKineticDevice::CallbackHandle depth_callback_handle_;
      pcl::SoftKineticDevice::CallbackHandle image_callback_handle_;
      pcl::SoftKineticDevice::CallbackHandle cloud_callback_handle_;
      pcl::SoftKineticDevice::CallbackHandle cloud_rgba_callback_handle_;

      mutable bool running_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace pcl
#endif // __PCL_IO_SK_GRABBER__
#endif // HAVE_SOFTKINETIC
