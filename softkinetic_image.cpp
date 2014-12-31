#include "softkinetic_image.h"

#include <string.h>

namespace softkinetic_wrapper
{

  DepthImage::DepthImage (unsigned width, unsigned height, const float* depth)
  {
    width_ = width;
    height_= height;
    int len = width*height;
    float* buffer = new float[len];
    raw_.reset(buffer);
    memcpy(buffer, depth, len*sizeof(float));
  }

  // line_step is the line stripe for depth_buffer in bytes
  void DepthImage::fillDepthImage(unsigned width, unsigned height, float* depth_buffer, unsigned line_step) const
  {
    if (line_step == 0)
      line_step = width * static_cast<unsigned> (sizeof (float));

    if (line_step == width * static_cast<unsigned> (sizeof (float)))
    {
      memcpy(depth_buffer, raw_.get(), width*height*sizeof(float));
      return;
    }
    float* srcptr = raw_.get();
    for (int i = 0; i < height; i++) 
    {
      float* rowptr = reinterpret_cast<float*> (reinterpret_cast<unsigned char*>(depth_buffer) + i*line_step );
      memcpy(rowptr, srcptr + i*width, width*sizeof(float));
    }
  }

  // line_step is the line stripe for depth_buffer in bytes
  void DepthImage::fillDepthImageRaw(unsigned width, unsigned height, unsigned short* depth_buffer, unsigned line_step) const
  {
    if (line_step == 0)
      line_step = width * static_cast<unsigned> (sizeof (unsigned short));

    float* srcptr = raw_.get();
    for (int r = 0, cnt = 0; r < height; r++)
    {
      unsigned short* rowptr = reinterpret_cast<unsigned short*> (reinterpret_cast<unsigned char*>(depth_buffer) + r*line_step );
      for (int c = 0; c < width; c++, cnt++)
        rowptr[c] = srcptr[cnt]*1000;
    }
  }

  ImageRGB24::ImageRGB24(unsigned width, unsigned height, const unsigned char* image)
  {
    width_ = width;
    height_= height;
    int len = width*height*3;
    unsigned char* buffer = new unsigned char[len];
    raw_.reset(buffer);
    memcpy(buffer, image, len*sizeof(unsigned char));
  }

  void ImageRGB24::fillRGB(unsigned width, unsigned height, unsigned char* rgb_buffer, unsigned rgb_line_step) const
  {
    const unsigned char* data = raw_.get();
    if (width == width_ && height == height_)
    {
      unsigned line_size = width * 3;
      if (rgb_line_step == 0 || rgb_line_step == line_size)
      {
        memcpy (rgb_buffer, data, width*height*3);
      }
      else // line by line
      {
        unsigned char* rgb_line = rgb_buffer;
        const unsigned char* src_line = static_cast<const unsigned char*> (data);
        for (unsigned yIdx = 0; yIdx < height; ++yIdx, rgb_line += rgb_line_step, src_line += line_size)
        {
          memcpy (rgb_line, src_line, line_size);
        }
      }
    }
    else if (width_ % width == 0 && height_ % height == 0) // downsamplig
    {
      unsigned src_step = width_ / width*3;
      unsigned src_skip = (height_ / height - 1) * width_*3;

      if (rgb_line_step == 0)
        rgb_line_step = width * 3;

      unsigned dst_skip = rgb_line_step - width * 3; // skip of padding values in bytes

      //XnRGB24Pixel* dst_line = reinterpret_cast<XnRGB24Pixel*> (rgb_buffer);
      //const XnRGB24Pixel* src_line = data;
      unsigned char* dst_line = rgb_buffer;
      const unsigned char* src_line = data;

      for (unsigned yIdx = 0; yIdx < height; ++yIdx, src_line += src_skip)
      {
        for (unsigned xIdx = 0; xIdx < width; ++xIdx, src_line += src_step, dst_line = dst_line+3)
        {
          dst_line[0] = src_line[0];
          dst_line[1] = src_line[1];
          dst_line[2] = src_line[2];
        }

        if (dst_skip != 0)
        {
          //unsigned char* temp = reinterpret_cast <unsigned char*> (dst_line);
          dst_line = (dst_line + dst_skip);
        }
      }
    }
    
  }



}

