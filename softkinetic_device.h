#ifndef __SOFTKINETIC_DEVICE__
#define __SOFTKINETIC_DEVICE__

#include <pcl/pcl_macros.h>
#include <DepthSense.hxx>
#include "softkinetic_image.h"
#include <pcl/io/boost.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/thread.hpp>

//namespace ds = DepthSense;


namespace pcl {

class PCL_EXPORTS SoftKineticDevice
{
  //struct PointXYZ;
  //struct PointXYZRGB;
  //struct PointXYZRGBA;
  //struct PointXYZI;
  //template <typename T> class PointCloud;

public:
  typedef boost::function<void(boost::shared_ptr<softkinetic_wrapper::Image>, void* cookie) > ImageCallbackFunction;
  typedef boost::function<void(boost::shared_ptr<softkinetic_wrapper::DepthImage>, void* cookie) > DepthImageCallbackFunction;
  typedef boost::function<void(pcl::PointCloud<pcl::PointXYZ>::ConstPtr, void* cookie) > CloudCallbackFunction;
  typedef boost::function<void(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr, void* cookie) > CloudRGBACallbackFunction;  

  //typedef boost::function<void(const std::pair<Eigen::Matrix3f, Eigen::Matrix4f void* cookie) > CameraPoseCallbackFunction;
  typedef unsigned CallbackHandle;

public:
  SoftKineticDevice();
  ~SoftKineticDevice() throw ();

  inline static SoftKineticDevice& getInstance();


  static void nodeConnectedCallback(DepthSense::Device device, DepthSense::Device::NodeAddedData data);
  static void nodeDisconnectedCallback(DepthSense::Device device, DepthSense::Device::NodeRemovedData data);
  static void deviceConnectedCallback(DepthSense::Context context, DepthSense::Context::DeviceAddedData data);
  static void deviceDisconnectedCallback(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data);
  static void newColorSampleCallback(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data);
  static void newDepthSampleCallback(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);

  /** \brief Start the data acquisition. */
  virtual void
  start ();

  /** \brief Stop the data acquisition. */
  virtual void
  stop ();

  float
  getFramesPerSecond () const;

  CallbackHandle 
  registerImageCallback (const ImageCallbackFunction& callback, void* custom_data) throw ();

  bool 
  unregisterImageCallback (const SoftKineticDevice::CallbackHandle& callbackHandle) throw ();

  /** \brief registers a callback function for the image stream with an optional user defined parameter.
    *        This version is used to register a member function of any class.
    *        The callback will always be called with a new image and the user data "cookie".
    * \param[in] callback the user callback to be called if a new image is available
    * \param instance
    * \param[in] cookie the cookie that needs to be passed to the callback together with the new image.
    * \return a callback handler that can be used to remove the user callback from list of image-stream callbacks.
    */
  template<typename T> CallbackHandle 
  registerImageCallback (void (T::*callback)(boost::shared_ptr<softkinetic_wrapper::Image>, void* cookie), T& instance, void* cookie = NULL) throw ();

  CallbackHandle 
  registerDepthCallback (const DepthImageCallbackFunction& callback, void* custom_data) throw ();

  bool 
  unregisterDepthCallback (const SoftKineticDevice::CallbackHandle& callbackHandle) throw ();

  /** \brief registers a callback function for the depth stream with an optional user defined parameter.
    *        This version is used to register a member function of any class.
    *        The callback will always be called with a new depth image and the user data "cookie".
    * \param[in] callback the user callback to be called if a new depth image is available
    * \param instance
    * \param[in] cookie the cookie that needs to be passed to the callback together with the new depth image.
    * \return a callback handler that can be used to remove the user callback from list of depth-stream callbacks.
    */
  template<typename T> CallbackHandle 
  registerDepthCallback (void (T::*callback)(boost::shared_ptr<softkinetic_wrapper::DepthImage>, void* cookie), T& instance, void* cookie = NULL) throw ();


  CallbackHandle 
  registerCloudCallback (const CloudCallbackFunction& callback, void* custom_data) throw ();

  bool 
  unregisterCloudCallback (const SoftKineticDevice::CallbackHandle& callbackHandle) throw ();

  /** \brief registers a callback function for the image stream with an optional user defined parameter.
    *        This version is used to register a member function of any class.
    *        The callback will always be called with a new image and the user data "cookie".
    * \param[in] callback the user callback to be called if a new image is available
    * \param instance
    * \param[in] cookie the cookie that needs to be passed to the callback together with the new image.
    * \return a callback handler that can be used to remove the user callback from list of image-stream callbacks.
    */
  template<typename T> CallbackHandle 
  registerCloudCallback (void (T::*callback)(pcl::PointCloud<pcl::PointXYZ>::ConstPtr, void* cookie), T& instance, void* cookie = NULL) throw ();

  Eigen::Matrix3f getIntrinsicParameters() const { return intrinsics_; }
  Eigen::Matrix4f getExtrinsicParameters() const { return extrinsics_; }

protected:
  /** \brief Initializes the PXC grabber and the grabbing pipeline. */
  bool
  init ();

  /** \brief Closes the grabbing pipeline. */
  void
  close ();

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

  virtual void 
  onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data);

  virtual void 
  onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);


  /** \brief The global context used. */
  mutable DepthSense::Context context_;

  // FPS computation
  mutable float fps_;
  mutable boost::mutex fps_mutex_;

  int image_width_, image_height_;
  int depth_width_, depth_height_;
  mutable bool running_;

  // TODO condition variable may be better for synchronizing
  mutable bool has_depth_data_;
  mutable bool has_color_data_;
  mutable boost::mutex data_ready_mutex_;

  mutable unsigned int rgb_array_size_;
  mutable unsigned int depth_buffer_size_;

  typedef boost::function<void(boost::shared_ptr<softkinetic_wrapper::Image>) > ActualImageCallbackFunction;
  typedef boost::function<void(boost::shared_ptr<softkinetic_wrapper::DepthImage>) > ActualDepthImageCallbackFunction;
  typedef boost::function<void(pcl::PointCloud<pcl::PointXYZ>::ConstPtr) > ActualCloudCallbackFunction;

  std::map<CallbackHandle, ActualImageCallbackFunction> image_callback_;
  std::map<CallbackHandle, ActualDepthImageCallbackFunction> depth_callback_;
  std::map<CallbackHandle, ActualCloudCallbackFunction> cloud_callback_;


  CallbackHandle image_callback_handle_counter_;
  CallbackHandle depth_callback_handle_counter_;
  CallbackHandle cloud_callback_handle_counter_;

  // thread where the grabbing takes place
  boost::thread grabber_thread_;
  boost::thread skrun_thread_;

  boost::shared_ptr<softkinetic_wrapper::Image> rgb_image_;
  boost::shared_ptr<softkinetic_wrapper::DepthImage> depth_image_;

  PointCloud<PointXYZRGBA>::Ptr cloud_;

  Eigen::Matrix3f intrinsics_;
  Eigen::Matrix4f extrinsics_;

};

SoftKineticDevice& SoftKineticDevice::getInstance()
{
  static SoftKineticDevice device;
  return device;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T> SoftKineticDevice::CallbackHandle
SoftKineticDevice::registerImageCallback (void (T::*callback)(boost::shared_ptr<softkinetic_wrapper::Image>, void* cookie), T& instance, void* custom_data) throw ()
{
  image_callback_[image_callback_handle_counter_] = boost::bind (callback, boost::ref (instance), _1, custom_data);
  return (image_callback_handle_counter_++);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T> SoftKineticDevice::CallbackHandle
SoftKineticDevice::registerDepthCallback (void (T::*callback)(boost::shared_ptr<softkinetic_wrapper::DepthImage>, void* cookie), T& instance, void* custom_data) throw ()
{
  depth_callback_[depth_callback_handle_counter_] = boost::bind ( callback,  boost::ref (instance), _1, custom_data);
  return (depth_callback_handle_counter_++);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T> SoftKineticDevice::CallbackHandle
SoftKineticDevice::registerCloudCallback (void (T::*callback)(pcl::PointCloud<pcl::PointXYZ>::ConstPtr, void* cookie), T& instance, void* custom_data) throw ()
{
  cloud_callback_[cloud_callback_handle_counter_] = boost::bind ( callback,  boost::ref (instance), _1, custom_data);
  return (cloud_callback_handle_counter_++);
}


}

#endif