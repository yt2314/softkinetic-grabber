#ifndef __PCL_IO_SOFTKINETIC_WRAPPER__
#define __PCL_IO_SOFTKINETIC_WRAPPER__

#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>
#include <boost/shared_array.hpp>

namespace softkinetic_wrapper
{
  class PCL_EXPORTS DepthImage
  {
    public:
      typedef boost::shared_ptr<DepthImage> Ptr;
      typedef boost::shared_ptr<const DepthImage> ConstPtr;

      //inline DepthImage (unsigned width, unsigned height, unsigned short*);
      DepthImage (unsigned widht, unsigned height, const float*);
      virtual ~DepthImage() {}

      unsigned getWidth() { return width_; }
      unsigned getHeight() { return height_; }
      float* getDepthMap() { return raw_.get(); }

      virtual void fillDepthImage(unsigned width, unsigned height, float* depth, unsigned line_step=0) const;
      virtual void fillDepthImageRaw(unsigned width, unsigned height, unsigned short* depth, unsigned line_step = 0) const;
    private:
      boost::shared_array<float> raw_;
      unsigned width_;
      unsigned height_;
  };

  class PCL_EXPORTS Image
  {
  public:
    typedef boost::shared_ptr<Image> Ptr;
    typedef boost::shared_ptr<const Image> ConstPtr;

    virtual void fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer,
                      unsigned rgb_line_step = 0) const = 0;
    virtual unsigned char* getRGB() const = 0;
    virtual unsigned getWidth () const = 0;
    virtual unsigned getHeight () const = 0;
  };

  class PCL_EXPORTS ImageRGB24  : public Image
  {
    public:
      ImageRGB24 (unsigned width, unsigned height, const unsigned char* image);
      virtual ~ImageRGB24 () {}
      virtual void fillRGB (unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step = 0) const;

      virtual unsigned getWidth() const { return width_; }
      virtual unsigned getHeight() const { return height_; }
      unsigned char* getRGB() const { return raw_.get(); }

    private:
      boost::shared_array<unsigned char> raw_;
      unsigned width_;
      unsigned height_;
  };

}




#endif
