#include <vw/Image.h> 
#include <vw/Stereo.h> 
#include <vw/FileIO.h> 

using namespace vw;


// Pixel accessor function
class MaxFilter : public ReturnFixedType<PixelGray<float> > {
  int m_kern_size;
  int m_threshold;

public:
  MaxFilter(int kern_size, int threshold) : m_kern_size(kern_size), m_threshold(threshold) {} 

  BBox2i work_area() const { return BBox2i(-m_kern_size,-m_kern_size,
                                           m_kern_size*2,m_kern_size*2); }

  template <class PixelAccessorT>
  typename boost::remove_reference<typename PixelAccessorT::result_type>::type
  operator()(PixelAccessorT acc) const {
    
    float max_val = ScalarTypeLimits<float>::lowest();
    
    PixelAccessorT row_acc = acc;
    row_acc.advance(-m_kern_size, -m_kern_size/2);
    
    for (int j = -m_kern_size/2; j < -m_kern_size/2+m_kern_size; ++j) {
      PixelAccessorT col_acc = row_acc;
      for (int i = -m_kern_size; i < -m_kern_size/2+m_kern_size; ++i) {
        if (*col_acc > max_val && i*i+j*j <= pow(m_kern_size/2,2)) 
          max_val = *col_acc;
        col_acc.next_col();
      }
      row_acc.next_row();
    }
    
    if (max_val > m_threshold) 
      return max_val;
    else 
      return *acc;
  }
};


// ------------------------------------------------------------------

class Histogram {
  std::vector<double> m_bins;
  int m_nbins;
  long int m_nvalid;
  
public:
  template <class ViewT>
  Histogram(ImageViewBase<ViewT> const& view, int nbins = 256) {

    TerminalProgressCallback callback(InfoMessage, "\t--> Building Histogram: ");
    m_nbins = nbins;
    m_nvalid = 0;
    m_bins.resize(nbins);
    for (unsigned i = 0; i < m_nbins; ++i) 
      m_bins[i] = 0.0;

    // Determine the standard range of pixel values.  FIXME: this is
    // hardwired for now for floating point images between [0, 1].
    float lo = 0.0;
    float hi = 1.0;

    // Fill the bins using the pixel data in the image.
    // 
    // FIXME: This is hardwired for PixelGray<float> images for now!
    for (unsigned j = 0; j < view.impl().rows(); ++j) {
      callback.report_progress(float(j) / view.impl().rows());
      for (unsigned i = 0; i < view.impl().cols(); ++i) {
        if ( is_valid( view.impl()(i,j) ) ) {
          float val = view.impl()(i,j)[0];
          int idx = floor(val * (m_nbins-1));
          if (idx < 0) idx = 0;
          if (idx > 255) idx = 255;

          m_bins[idx]++;
          m_nvalid++;
        }
      }
    }
    
    // Normalize the bins
    for (unsigned i = 0; i < m_nbins; ++i) {
      m_bins[i] /= (double)m_nvalid;
      //      std::cout << (float(i)/m_nbins) << " : " << m_bins[i] << "\n";
    }
    callback.report_finished();

  }

  float find_threshold(float percentile) {
    float accum = 0;
    for (unsigned i = 0; i < m_nbins; ++i) {
      accum += m_bins[i];
      if (accum >= percentile)
        return float(i) / m_nbins;
    }

    // Should not be reached
    return 1.0;
  }
                     

  
};

// ------------------------------------------------------------------

/// DisparityTransform image transform functor
///
/// Transform points in an image based on offset values in a disparity
/// map. 
class DisparityTransform : public TransformBase<DisparityTransform> {
  ImageViewRef<PixelDisparity<float> > m_offset_image;
public:
  template <class OffsetImageT>
  DisparityTransform(ImageViewBase<OffsetImageT> const& offset_image)
    : m_offset_image(offset_image.impl()) {}
  
  inline Vector2 reverse(const Vector2 &p) const {
    int32 x = (int32) p.x(), y = (int32) p.y();
    // VW_DEBUG_ASSERT(x>=0 && y>=0 && x<m_offset_image.cols() && y<m_offset_image.rows(),
    //                 LogicErr() << "Point offest transform: exceeded lookup table dimensions.");
    PixelDisparity<float> offset = m_offset_image(x,y);
    if (offset.missing()) 
      return Vector2(-1, p.y());  // Missing pixels return a value
    // outside of the bounds of the
    // original image.  Therefore, edge
    // extension will determine what value
    // missing pixels take on.
    else
      return Vector2(p.x() + offset.h(), p.y() + offset.v());
  }
};

// ------------------------------------------------------------------

void flood(ImageView<PixelGray<float> > &in, int i, int j, int &num_pixels) {

  if (i >=0 && j >= 0 && i < in.cols() && j < in.rows() && in(i,j) == 1 ) {
    num_pixels++;
    in (i,j) = -1; // Mark as visited
    
    // Explore the 8 connected neighborhood
    if (num_pixels < 500) {
      flood(in, i+1, j-1, num_pixels);
      flood(in, i  , j-1, num_pixels);
      flood(in, i-1, j-1, num_pixels);
      flood(in, i+1, j  , num_pixels);
      flood(in, i-1, j  , num_pixels);
      flood(in, i+1, j+1, num_pixels);
      flood(in, i  , j+1, num_pixels);
      flood(in, i-1, j+1, num_pixels);
    }
  }
}

void fill(ImageView<PixelGray<float> > const&in, int i, int j, int value) {
  if (i >=0 && j >= 0 && i < in.cols() && j < in.rows() && in(i,j) == -1 ) {
    in(i,j) = value;
    
    // Explore the 8 connected neighborhood
    fill(in, i+1, j-1, value);
    fill(in, i  , j-1, value);
    fill(in, i-1, j-1, value);
    fill(in, i+1, j  , value);
    fill(in, i-1, j  , value);
    fill(in, i+1, j+1, value);
    fill(in, i  , j+1, value);
    fill(in, i-1, j+1, value);
  }
}

// Region size determination (implemented as a recursive flood fill)
template <class ViewT>
ImageView<PixelGray<float> > region_size_image(ImageViewBase<ViewT> const& in) {

  ImageView<PixelGray<float> > out = in.impl();

  TerminalProgressCallback callback(InfoMessage, "\t--> Computing region sizes: ");
  for (unsigned j=0; j < out.rows(); ++j) {
    callback.report_progress(float(j) / out.rows());
    for (unsigned i=0; i < out.cols(); ++i) {
      int num_pixels = 0;
      flood(out, i, j, num_pixels);
      fill(out, i, j, num_pixels);
    }
  }
  callback.report_finished();

  return out;
}



// ------------------------------------------------------------------


void photometric_outlier_rejection(std::string out_prefix, int kernel_size) {

  // DiskImageView<PixelGray<float> > dust_mask(out_prefix + "-DustMask.tif");
  // ImageView<PixelGray<float> > out = region_size_image(dust_mask);
  // std::cout << out.cols() << "  " << out.rows() << "   " << out.planes() << "   " << out.channels() << "\n";
  // write_image(out_prefix + "-DustMaskFixed.tif",
  //             out,
  //             TerminalProgressCallback(InfoMessage, "\t--> Writing region size image: "));

  // ImageView<PixelGray<float> > padded_dust_mask = per_pixel_accessor_filter(out,
  //                                                                           MaxFilter(kernel_size, 100));
  // ImageView<PixelGray<float> > padded_dust_mask2 = per_pixel_accessor_filter(padded_dust_mask,
  //                                                                            MaxFilter(kernel_size, 10));

  // write_image(out_prefix + "-PaddedDustMaskFixed.tif",
  //             padded_dust_mask2,
  //             TerminalProgressCallback(InfoMessage, "\t--> Writing padded dust mask: "));

  // exit(0);

  
  // Open the necessary files
  DiskImageView<PixelGray<float> > right_disk_image(out_prefix + "-R.tif");
  DiskImageView<PixelDisparity<float> > disparity_disk_image(out_prefix + "-F.exr");

  std::cout << "Dust-based outlier detection (warning: tested with Apollo imagery only!!)\n";
  DisparityTransform trans(disparity_disk_image);
  write_image(out_prefix + "-R-reproj.tif", 
              transform(right_disk_image, trans, ZeroEdgeExtension()), 
              TerminalProgressCallback(InfoMessage, "\t--> Reprojecting Right Image: "));
  
  DiskImageView<PixelGray<float> > left_disk_image(out_prefix + "-L.tif");
  DiskImageView<PixelGray<float> > right_reproj_image(out_prefix + "-R-reproj.tif");
  ImageViewRef<PixelMask<PixelGray<float> > > left = create_mask(left_disk_image);
  ImageViewRef<PixelMask<PixelGray<float> > > right = create_mask(right_reproj_image, 0);

  ImageView<PixelGray<float> > diff = abs(left_disk_image-right_reproj_image);
  ImageView<PixelMask<PixelGray<float> > > masked_diff = copy_mask(diff, right);
  Histogram hist(masked_diff);
  float thresh = hist.find_threshold(0.9999);
  std::cout << "\t--> Using Threshold: " << thresh << "\n";

  ImageViewRef<PixelGray<float> > dust_mask = threshold(apply_mask(masked_diff,0),thresh,0.0,1.0);
  
  // Compute region sizes
  ImageView<PixelGray<float> > out = region_size_image(dust_mask);
  write_image(out_prefix + "-RegionSize.tif",
              out,
              TerminalProgressCallback(InfoMessage, "\t--> Writing region size image: "));

  // The first padding takes care of the large dust monsters, adding extra padding around them.
  ImageView<PixelGray<float> > padded_dust_mask = per_pixel_accessor_filter(out,
                                                                            MaxFilter(kernel_size*1.5, 50));
  // The first padding takes care of the smaller dust spots.
  ImageView<PixelGray<float> > padded_dust_mask2 = 1.0 - clamp(per_pixel_accessor_filter(padded_dust_mask,
                                                                                         MaxFilter(kernel_size*1.5, 0)));

  DiskImageView<PixelGray<float> > input_mask_image(out_prefix + "-NurbsMask.tif");
  ImageViewRef<PixelGray<float> > final_mask = apply_mask(copy_mask(input_mask_image, 
                                                                    create_mask(padded_dust_mask2, 0)));


  write_image(out_prefix + "-DustDiff.tif",
              apply_mask(masked_diff, 0),
              TerminalProgressCallback(InfoMessage, "\t--> Writing dust diff: "));

  write_image(out_prefix + "-DustMask.tif",
              dust_mask,
              TerminalProgressCallback(InfoMessage, "\t--> Writing dust mask: "));

  write_image(out_prefix + "-PaddedDustMask.tif",
              padded_dust_mask2,
              TerminalProgressCallback(InfoMessage, "\t--> Writing padded dust mask: "));

  write_image(out_prefix + "-NurbsDustMask.tif",
              final_mask,
              TerminalProgressCallback(InfoMessage, "\t--> Writing dust NURBS mask: "));

}
