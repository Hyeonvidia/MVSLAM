#pragma once
// Owning copy of an ImageView (the API view is only valid during track()). Used to carry the raw
// left/right/depth images of a keyframe to the mapper worker. No OpenCV dependency.
#include <cstring>
#include <vector>

#include "mvslam/api/types.hpp"

namespace mvslam::core {

struct ImageBuffer {
  std::vector<std::uint8_t> data;
  int width = 0, height = 0, stride = 0;
  PixelFormat format = PixelFormat::Gray8;

  static int bytesPerPixel(PixelFormat f) {
    switch (f) {
      case PixelFormat::Gray8: return 1;
      case PixelFormat::Rgb8: case PixelFormat::Bgr8: return 3;
      case PixelFormat::Depth16: return 2;
      case PixelFormat::DepthF32: return 4;
    }
    return 1;
  }
  static ImageBuffer copy(const ImageView& v) {
    ImageBuffer b;
    b.width = v.width; b.height = v.height; b.format = v.format;
    b.stride = v.width * bytesPerPixel(v.format);  // repack tightly
    b.data.resize(static_cast<std::size_t>(b.stride) * static_cast<std::size_t>(b.height));
    for (int r = 0; r < v.height; ++r)
      std::memcpy(b.data.data() + static_cast<std::size_t>(r) * b.stride, v.data + static_cast<std::size_t>(r) * v.stride, static_cast<std::size_t>(b.stride));
    return b;
  }
  bool empty() const { return data.empty(); }
  ImageView view() const { return ImageView{data.data(), width, height, stride, format}; }
};

}  // namespace mvslam::core
