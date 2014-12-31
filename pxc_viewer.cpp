#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/filters/filter.h>
#include <boost/shared_array.hpp>

//#include <pcl/io/pxc_grabber.h>
#include "softkinetic_grabber.h"
#include "softkinetic_image.h"

/*
#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
     static unsigned count = 0;\
     static double last = pcl::getTime ();\
     double now = pcl::getTime (); \
     ++count; \
     if (now - last >= 1.0) \
     { \
       std::cout << "Average framerate("<< _WHAT_ << "): " <<
double(count)/double(now - last) << " Hz" <<  std::endl; \
       count = 0; \
       last = now; \
     } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif
*/
using namespace pcl::console;
using namespace boost::filesystem;

template<typename PointType>
class PXCGrabFrame
{
   typedef pcl::PointCloud<PointType> Cloud;
   typedef typename Cloud::ConstPtr CloudConstPtr;
   typedef typename Cloud::Ptr CloudPtr;
   public:
     PXCGrabFrame (pcl::SKGrabber& grabber)
     : visualizer_ (new pcl::visualization::PCLVisualizer ("PXC Viewer"))
     , quit_ (false)
     , continuous_ (false)
     , grabber_ (grabber)
     , save_ (false)
     , counter_ (0)
     {
       visualizer_->setBackgroundColor (0, 0, 0);
       visualizer_->addCoordinateSystem (1.0, "global");
       visualizer_->initCameraParameters ();
       visualizer_->setPosition (0, 500);
       visualizer_->setSize (320, 240);
       visualizer_->setCameraClipDistances (-0.01, -10.0);

       color_viewer_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);
       depth_viewer_ = pcl::visualization::ImageViewer::Ptr(new pcl::visualization::ImageViewer);
       color_viewer_->setPosition(0, 0);
       color_viewer_->setSize(640, 480);
       depth_viewer_->setPosition(640,0);
       depth_viewer_->setSize(320, 240);

       intrinsics_ = Eigen::Matrix3f::Identity();
       extrinsics_ = Eigen::Matrix4f::Identity();
     }

     void
     cloud_cb_ (const CloudConstPtr& cloud)
     {
       if (quit_)
         return;

       boost::mutex::scoped_lock lock (cloud_mutex_);
       cloud_ = cloud;
     }

     void
     color_cb_ (softkinetic_wrapper::ImageRGB24& color_image)
     {
       if (quit_)
         return;

       //boost::mutex::scoped_lock lock (cloud_mutex_);
       // TODO
       int len = color_image.getWidth()*color_image.getHeight()*3;
       unsigned char* data = new unsigned char[len];
       color_image_.reset(data);
       color_image.fillRGB(color_image.getWidth(), color_image.getHeight(), data);
     }

     void
     depth_cb_ (softkinetic_wrapper::DepthImage& depth_image)
     {
       if (quit_)
         return;

       //boost::mutex::scoped_lock lock (cloud_mutex_);
       float* data = new float[depth_image.getWidth()*depth_image.getHeight()];
       depth_image_.reset(data);
       depth_image.fillDepthImage(depth_image.getWidth(), depth_image.getHeight(), data);
       //depth_image_ = depth_image.getData();
     }

     void camera_cb_(const Eigen::Matrix3f& intrinsics, const Eigen::Matrix4f& extrinsics)
     {
        if (quit_)
          return;

       boost::mutex::scoped_lock lock (cloud_mutex_);
       intrinsics_ = intrinsics;
       extrinsics_ = extrinsics;

       //printf("before camera callback\n");
       //visualizer_->setCameraParameters(intrinsics, extrinsics);
       //printf("after camera callback\n");
     }

     void
     keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
     {
       if (event.keyUp ())
       {
         switch (event.getKeyCode ())
         {
           case 27:
           case 'Q':
           case 'q': printf("quit\n"); quit_ = true; visualizer_->close ();
             break;
           case 'S':
           case 's': printf("save\n"); save_ = true;
             break;
           case ' ': continuous_ = !continuous_;
             break;
         }
       }
     }

     CloudConstPtr
     getLatestCloud ()
     {
       //lock while we swap our cloud and reset it.
       boost::mutex::scoped_lock lock(cloud_mutex_);
       CloudConstPtr temp_cloud;
       temp_cloud.swap (cloud_); //here we set cloud_ to null, so that
       //it is safe to set it again from our
       //callback
       return (temp_cloud);
     }

     boost::shared_array<unsigned char>
     getLatestColorImage ()
     {
       //lock while we swap our cloud and reset it.
       boost::mutex::scoped_lock lock(cloud_mutex_);
       boost::shared_array<unsigned char> temp_color;
       temp_color.swap (color_image_); //here we set cloud_ to null, so that
       //it is safe to set it again from our
       //callback
       return (temp_color);
     }

     boost::shared_array<float>
     getLatestDepthImage ()
     {
       //lock while we swap our cloud and reset it.
       boost::mutex::scoped_lock lock(cloud_mutex_);
       boost::shared_array<float> temp_depth;
       temp_depth.swap (depth_image_); //here we set cloud_ to null, so that
       //it is safe to set it again from our
       //callback
       return (temp_depth);
     }

     void saveCamera(pcl::visualization::Camera& camera)
     {
       std::ofstream of("camera.txt");
       // Eigen::Matrix4d proj;
       // camera.computeProjectionMatrix (proj);

       // for (int i = 0; i < 4; i++) {
       //   for (int j = 0; j < 4; j++) {
       //     of << proj(i, j) << "  ";
       //   }
       //   of << std::endl;
       // }

       of << "focal" << std::endl;
       of << camera.focal[0] << "  " << camera.focal[1] << "  " << camera.focal[2] << std::endl;
       of << "pos" << std::endl;
       of << camera.pos[0] << "  " << camera.pos[1] << "  " << camera.pos[2] << std::endl;
       of << "viewup" << std::endl;
       of << camera.view[0] << "  " << camera.view[1] << "  " << camera.view[2] << std::endl;
       of << "clip" << std::endl;
       of << camera.clip[0] << "  " << camera.clip[1] << std::endl;
       of << "fovy" << std::endl;
       of << camera.fovy << std::endl;
       of << "window_size" << std::endl;
       of << camera.window_size[0] << "  " << camera.window_size[1] << std::endl;
       of << "window_pos" << std::endl;
       of << camera.window_pos[0] << "  " << camera.window_pos[1] << std::endl;

       of.close();
     }
     void
     run ()
     {
       // register the keyboard and mouse callback for the visualizer
       visualizer_->registerKeyboardCallback(&PXCGrabFrame::keyboard_callback,*this);
       color_viewer_->registerKeyboardCallback(&PXCGrabFrame::keyboard_callback,*this);
       depth_viewer_->registerKeyboardCallback(&PXCGrabFrame::keyboard_callback,*this);

       // make callback function from member function
       boost::function<void (const CloudConstPtr&)> f = boost::bind(&PXCGrabFrame::cloud_cb_, this, _1); 

       // connect callback function for desired signal. In this case its a point cloud with color values
       boost::signals2::connection c = grabber_.registerCallback (f);

       boost::function<void (softkinetic_wrapper::ImageRGB24&)> cf = boost::bind(&PXCGrabFrame::color_cb_, this, _1); 
       grabber_.registerCallback (cf);
       boost::function<void (softkinetic_wrapper::DepthImage&)> df = boost::bind(&PXCGrabFrame::depth_cb_, this, _1); 
       grabber_.registerCallback (df);
       boost::function<void (const Eigen::Matrix3f&, const Eigen::Matrix4f&)> pf = boost::bind(&PXCGrabFrame::camera_cb_, this, _1, _2); 
       grabber_.registerCallback (pf);

       // start receiving point clouds
       printf("before run::start\n");
       grabber_.start ();
       printf("after run::start\n");

       // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
       while (!visualizer_->wasStopped())
       {
         boost::this_thread::sleep (boost::posix_time::microseconds (1000));

         if (color_image_) {
           boost::shared_array<unsigned char> color_image = getLatestColorImage();
           if (color_image)
            color_viewer_->showRGBImage(reinterpret_cast<unsigned char*> (&color_image[0]), 640, 480);
         }
         if (depth_image_) {
           boost::shared_array<float> depth_image = getLatestDepthImage();
           if (depth_image)
            depth_viewer_->showFloatImage(reinterpret_cast<float*> (&depth_image[0]), 320, 240);
         }

         //visualizer_->spinOnce ();

         if (cloud_)
         {
           CloudConstPtr cloud = getLatestCloud ();
           if (!cloud)
           {
              //printf("Cloud is null\n");
           }

           //printf("grabbed an cloud\n");
           CloudPtr filtered_cloud (new Cloud ());
           filtered_cloud->reserve (cloud->size ());
           for (int point_id = 0; point_id < cloud->size (); ++point_id)
           {
             const PointType & p = cloud->points[point_id];

             if (pcl::isFinite (p))
             {
               filtered_cloud->push_back (p);
             }
           }

           
           if (!visualizer_->updatePointCloud (filtered_cloud, "PXCCloud"))
           {
           	 //visualizer_->removeAllPointClouds ();
             visualizer_->addPointCloud (filtered_cloud, "PXCCloud");
             //visualizer_->resetCameraViewpoint ("PXCCloud");
             visualizer_->setCameraParameters(intrinsics_, extrinsics_);
             visualizer_->setCameraClipDistances (-0.01, -10);
             //visualizer_->setCameraFieldOfView (1.125); //(0.8575);
             pcl::visualization::Camera camera;
             visualizer_->getCameraParameters(camera);
             saveCamera(camera);
             //printf("set camera parameters ");
             //printf("%g, %g\n", intrinsics_(0,0), intrinsics_(0,1));
             //boost::this_thread::sleep (boost::posix_time::microseconds (1000000));
             //visualizer_->spinOnce();
             //visualizer_->updateCamera();
             //visualizer_->setCameraPosition(0, 0, 0, 0, 0, 5, 0, 1, 0);

           }
           visualizer_->setCameraParameters (intrinsics_, extrinsics_);
           visualizer_->setCameraClipDistances (-0.01, -10.0);
           //visualizer_->setCameraFieldOfView (0.8575);
           visualizer_->spinOnce ();

           //visualizer_->setCameraParameters(intrinsics_, extrinsics_);
          
           if (save_)
           {
             std::stringstream cloud_name;
             cloud_name << counter_ << ".pcd";
             //pcl::io::savePCDFileBinaryCompressed (cloud_name.str().c_str (), *cloud);
             pcl::io::savePCDFile (cloud_name.str().c_str (), *cloud);
             ++counter_;
             save_ = false;
           }
         }
       }

       //while (!quit_)
         //boost::this_thread::sleep (boost::posix_time::seconds (1));

       // stop the grabber
       grabber_.stop ();
     }

     boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer_;
     pcl::visualization::ImageViewer::Ptr color_viewer_;
     pcl::visualization::ImageViewer::Ptr depth_viewer_;

     boost::shared_array<unsigned char> color_image_;
     boost::shared_array<float> depth_image_;

     bool quit_;
     bool continuous_;
     CloudConstPtr cloud_;
     mutable boost::mutex cloud_mutex_;
     pcl::SKGrabber& grabber_;

     Eigen::Matrix3f intrinsics_;
     Eigen::Matrix4f extrinsics_;

     bool save_;
     int counter_;
};

void
usage (char ** argv)
{
   std::cout << "usage: " << argv[0] << " <filename> <options>\n\n";

   print_info ("Options are:\n");
   print_info ("  -XYZ  = store just a XYZ cloud\n");
}

int
main (int argc, char** argv)
{
   std::string arg;
   if (argc > 1)
     arg = std::string (argv[1]);

   if (arg == "--help" || arg == "-h")
   {
     usage (argv);
     return 1;
   }

   bool xyz = false;
   if (argc > 1)
   {
     xyz = find_switch (argc, argv, "-XYZ");
   }

   pcl::SKGrabber& grabber = pcl::SKGrabber::getInstance();
   if (xyz)
   {
     PXCGrabFrame<pcl::PointXYZ> grab_frame (grabber);
     printf("before main::run\n");
     grab_frame.run ();
     printf("after main::run\n");
   }
   else
   {
     PXCGrabFrame<pcl::PointXYZRGBA> grab_frame (grabber);
     grab_frame.run ();
   }
   return (0);
} 
