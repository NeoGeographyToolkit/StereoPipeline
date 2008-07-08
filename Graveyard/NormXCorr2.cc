//
// Copyright (C) 2006 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//

#include <iostream>

#define MINDOUBLE -1e16 // Very small number
#define MAXDOUBLE 1e16  // Very large number

using namespace std;

// Include VXL libraries
#include <iostream>
#include <vxl_config.h>
#include <vil/vil_rgb.h>
#include <vil/vil_load.h>
#include <vil/vil_copy.h>
#include <vil/vil_save.h>
#include <vil/vil_fill.h>
#include <vil/vil_clamp.h>
#include <vil/vil_transpose.h>
#include <vil/vil_math.h>
#include <vil/vil_print.h>
#include <vil/algo/vil_convolve_1d.h>
#include <vil/algo/vil_convolve_2d.h>
#include <vil/algo/vil_correlate_2d.h>
#include <vil/algo/vil_fft.h>
#include <vil/algo/vil_gauss_reduce.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_matrix.h>
#include <vnl/algo/vnl_matrix_inverse.h>

// VisionWorkbench includes
#include "NormXCorr2.h"

  static const double K_conv2 = 2.7e-8; /* K_conv2 is empirically
					   calculated in MATLAB's
					   normxcorr2 routine */
  static const double K_fft = 3.3e-7; /* K_fft is empirically calculated
					 in MATLAB'snormxcorr2
					 routine */

  enum { ROW_CUMSUM = 1, COL_CUMSUM = 2 };

  double
  MaxPixel(vil_image_view<float> image)
  {
    unsigned int ni = image.ni();
    unsigned int nj = image.nj();
    unsigned int nplanes = image.nplanes();
    double value = MINDOUBLE;		   // Start with a very low value.

    float* plane = image.top_left_ptr();

    for (unsigned int p = 0; p < nplanes; ++p, plane += image.planestep())
      {
        float* row = plane;
        for (unsigned int j = 0; j < nj; ++j, row += image.jstep())
          {
            float* pixel = row;
            for (unsigned int i = 0; i < ni; ++i, pixel += image.istep())
              {
                if (*pixel > value)
                  {
                    value = *pixel;
                  }
              }
          }
      }
    
    return value;
  }

  double
  MinPixel(vil_image_view<float> image)
  {
    unsigned int ni = image.ni();
    unsigned int nj = image.nj();
    unsigned int nplanes = image.nplanes();
    double value = MAXDOUBLE;		   // Start with a very high value.

    float* plane = image.top_left_ptr();

    for (unsigned int p = 0; p < nplanes; ++p, plane += image.planestep())
    {
      float* row = plane;
      for (unsigned int j = 0; j < nj; ++j, row += image.jstep())
        {
          float* pixel = row;
          for (unsigned int i = 0; i < ni; ++i, pixel += image.istep())
            {
              if (*pixel < value)
                {
                  value = *pixel;
                }
	}
        }
    }

    return value;
  }

  // These are disabled to avoid compiler warnings until we get 
  // around to using them.
#if 0
  static double
  time_conv2(vil_image_view<float> image, vil_image_view<float> kernel)
  {
    return K_conv2 * image.ni() * image.nj() * kernel.ni() * kernel.nj();
  }

  static double
  time_fft(vil_image_view<float> image, vil_image_view<float> kernel)
  {
    double R = image.ni() + kernel.ni() - 1;
    double S = image.nj() + kernel.nj() - 1;

    double Tr = K_fft * R * log(R);
    double Ts;
  
    if ( S == R ) {
      Ts = Tr;
    } else {
      Ts = K_fft * S * log(S);
    }
    return S * Tr + R * Ts;
  }
#endif

  /* 
   * Rotates the image by a fixed amount of 180 degrees.
   * 
   * This is useful when correlating images, since correlation is
   * equivelent to flipping the image in both dimensions and running a
   * convolution.
   */
  template<class T> 
  static void
  rot180(vil_image_view<T> image)
  {
    unsigned int i,j;
    vil_image_view<T> result(image.ni(), image.nj(), image.nplanes());

    for (i = 0 ; i < image.ni() ; i++) {
      for (j = 0 ; j < image.nj() ; j++) {
	result(result.ni() - 1 - i, result.nj() - 1 - j) = image(i,j);
      }
    }
    vil_copy_deep(result, image);
  }


  /*
   * Compute the standard deviation of the pixels in an image.
   */
  template<class imT, class sumT>
  inline void
  image_std_dev(sumT& std_dev, vil_image_view<imT> im, unsigned p)
  {
    sumT mean, sum_sq;
    vil_math_mean(mean, im, p);

    const imT* row = im.top_left_ptr()+p*im.planestep();
    ptrdiff_t istep = im.istep(),jstep=im.jstep();
    const imT* row_end = row + im.nj()*jstep;
    int row_len = im.ni()*im.istep();
    sum_sq = 0;
    for (;row!=row_end;row+=jstep) {
      const imT* v_end = row + row_len;
      for (const imT* v = row;v!=v_end;v+=istep) { 
	sum_sq += (sumT(*v) - mean)*(sumT(*v)-mean); 
      }
    }
    std_dev = sqrt(sum_sq / ((im.ni()*im.nj()) - 1));
  }

  /*
   * Calculates the cumulative sum of the elements of A, either across
   * the rows direction (direction = ROW_CUMSUM) or down the columns
   * (direction = COL_CUMSUM).
   */

  static void
  cumsum(vil_image_view<float> image, unsigned int direction)
  {
    unsigned int i,j;

    if (direction == ROW_CUMSUM) {
      vnl_vector<double> sum(image.ni(), 0.0);

      for (j = 0; j < image.nj(); j++) {
	for (i = 0; i < image.ni(); i++) {
	  sum(i) += image(i,j);
	  image(i,j) = sum(i);
	}
      }
    } else { /* direction == COL_CUMSUM */
      vnl_vector<double> sum(image.nj(), 0.0);

      for (i = 0; i < image.ni(); i++) {
	for (j = 0; j < image.nj(); j++) {
	  sum(j) += image(i,j);
	  image(i,j) = sum(j);
	}
      }
    }
  }


  static vil_image_view<float>
  local_sum(vil_image_view<float> image, 
	    vil_image_view<float> kernel)
  {
    unsigned int i,j;
    vil_image_view<float> tempImg1(image.ni() + 2 * kernel.ni() - 1,
				    image.nj() + 2 * kernel.nj() - 1,
				    image.nplanes());
    vil_image_view<float> tempImg2(image.ni() + 2 * kernel.ni() - 1,
				    image.nj() + kernel.nj() - 1,
				    image.nplanes());
    vil_image_view<float> dest(image.ni() + kernel.ni() - 1,
				image.nj() + kernel.nj() - 1,
				image.nplanes());

    /* 
     * Fill outImg with zeros around the edges, 
     * and place the input image at the center of B.
     */
    vil_fill(tempImg1, 0.0f);
    for ( i = 0; i < image.ni(); i++) {
      for ( j = 0; j < image.nj(); j++) {
        tempImg1(kernel.ni() + i, kernel.nj() + j) = image(i,j);
      }
    }
    
    /*
     * Find the cumulative sum across the rows.
     */
    cumsum(tempImg1, ROW_CUMSUM);

    /* 
     * In matlab, the following loop would look something like:
     * tempImg2 = tempImg1(1+k_i:end, :) - tempImg1(1:end-k_i,:)
     */
    for ( i = 0; i < tempImg2.ni(); i++) {
      for ( j = 0; j < tempImg2.nj(); j++) {
	tempImg2(i,j) = tempImg1(i, kernel.nj() + j) - tempImg1(i,j);
      }
    }

    /*
     * Find the cumulative sum down the columns.
     */
    cumsum(tempImg2, COL_CUMSUM);

    /* 
     * In matlab, the following loop would look something like:
     * dest = tempImg2(:,1+k_j:end) - tempImg1(:,1:end-k_j)
     */
    for ( i = 0; i < dest.ni(); i++) {
      for ( j = 0; j < dest.nj(); j++) {
	dest(i,j) = tempImg2(kernel.ni() + i, j) - tempImg2(i,j);
      }
    }

    return dest;
  }

  /* 
   * The correlation can be computed in the frequency domain as:
   * 
   * ifft( fft(rot180(kernel)) * fft(image) ) 
   */
  static vil_image_view<float>
  freqxcorr2(vil_image_view<float> src, vil_image_view<float> kernel)
  {
    //  unsigned int out_i = src.ni() + kernel.ni() - 1;
    //  unsigned int out_j = src.nj() + kernel.nj() - 1;
    vil_image_view< std::complex<float> > Fa;
    vil_image_view< std::complex<float> > Fb;
    vil_image_view< std::complex<float> > Fproduct;
    vil_image_view<float> temp;
  
    /* 
     * First, we must determine the size of the FFT buffers, which are
     * required to be a power of 2 in length and width.
     */
    double fft_i = (double) src.ni();               
    double fft_j = (double) src.nj();
    fft_i = pow(2, ceil( log(fft_i) / log(2.0) ) );     
    fft_j = pow(2, ceil( log(fft_j) / log(2.0) ) );

    // For debugging
    //   std::cout << "Original image dimensions: (" << src.ni() << ", "
    // 	   << src.nj() << ")\n";
    //   std::cout << "FFT Canvas dimensions: (" << fft_i << ", " << fft_j
    // 	   << ")\n";

    Fa.set_size((unsigned int)fft_i, (unsigned int)fft_j);
    Fb.set_size((unsigned int)fft_i, (unsigned int)fft_j);
  
    /*
     * Create a complex image using the kernel, padding it with zeros up
     * to out_i and out_j.  The kernel is flipped before hand, so that
     * the correlation is computed rather than a straight convolution.
     */
    rot180(kernel);
    temp = vil_view_real_part(Fa);
    vil_fill(temp, 0.0f);
    vil_copy_to_window(kernel, temp, 0, 0);
    temp = vil_view_imag_part(Fa);
    vil_fill(temp, 0.0f);
    rot180(kernel);

    /* 
     * Create a complex pixel image using the source image, padding with
     * zeros as necessary. 
     */
    temp = vil_view_real_part(Fb);
    vil_fill(temp, 0.0f);
    vil_copy_to_window(src, temp, 0, 0);
    temp = vil_view_imag_part(Fb);
    vil_fill(temp, 0.0f);
  
    /* 
     * 2005-Sept-09
     * 
     * Lesson of the day: vil_fft_bwd does what I would have
     * called the "forward" fft; that is, fft(a).   I find this counter-
     * intuitive, but hey, that's how things go sometimes.
     */
    vil_fft_2d_bwd(Fb);
    vil_fft_2d_bwd(Fa);

    vil_math_image_product(Fa, Fb, Fproduct);
    vil_fft_2d_fwd(Fproduct);


    /*
     * Now we can trim the edges off of the correlation image, and 
     * return this as the result. 
     */
    temp = vil_view_real_part(Fproduct);
    vil_image_view<float> result(src.ni() + 1 - kernel.ni(),
				  src.nj() + 1 - kernel.nj(),
				  src.nplanes());
    unsigned int i, j;

    for (i = 0; i < result.ni(); i++) {
      for (j = 0; j < result.nj(); j++) {
	result(i,j) = temp(i + kernel.ni()-1, j + kernel.nj()-1);
      }
    }
    return result;
  }
   
  /*
   * Chooses the fastest correlation method based on the size of the
   * images being correlated.
   * 
   * Options are: (1) a image-domain (convolution) based correlation
   * method or (2) a frequency domain (fft) based method.
   *
   * Returns: A (image_x + 1 - kernel_x) by (image_y + 1 - kernel_y)
   * image of un-normalized correlation values.  The pixel in the upper
   * left of the correlated image is the correlation value corresponding
   * to when the center of the kernel is placed at (kernel_w/2,
   * kernel_h/2) on the reference image (the first position where the
   * kernel completely overlaps with the reference image.)
   */

 vil_image_view<float>
 xcorr2_fast(vil_image_view<float> image, vil_image_view<float> kernel)
  {
    /*
     * First, determine whether it will be faster to run the correlation in 
     * the time domain or the frequency domain.
     */
//     double conv_time = time_conv2(image, kernel); /* 1 time domain convolution */
//     double fft_time = 3 * time_fft(image, kernel); /* 2 fft plus 1 inverse fft */

    vil_image_view<float> cross_corr;

    printf("Running Fast Normalized Cross-Correlation.\n");
    // BUG: For now, we are forced to use image-domain correlation, because the FFT based 
    // correlation isn't working correctly.  At some point we should fix this, since thi FFT
    // based correlation would be much faster for large images.
    //     if (conv_time < fft_time) {
    if (1) {
      printf("\tUsing image-domain correlation method.\n\n");
      vil_correlate_2d(image, cross_corr, kernel, double());
    } else {
      printf("\tUsing frequency-domain correlation method.\n\n");
      cross_corr = freqxcorr2(image, kernel);
    }

    return cross_corr;
  }

  /*
   * normxcorr2()
   * 
   * author : mbroxton@email.arc.nasa.gov
   * 
   * This routine computes the normalized cross correlation between an
   * image and a kernel image (The kernel image must be smaller than the
   * full image) in the same manner as the normxcorr2 function in
   * MATLAB.  This code is adapted from the MATLAB normxcorr2 code,
   * which is itself based on a method which is described at
   * http://www.idiom.com/~zilla/Papers/nvisionInterface/nip.html.
   *
   * Rather than computing the normal sum of square differences
   * correlation, this function computes instead normalizes this value
   * by the autocorrelation of the image patch and the autocorrelation
   * of the kernel.  This eliminates most of the dependence of the
   * correlation on uniform intensity changes across the image.
   */

  // Note: a copy of a local variable is returned. This seems to be ok
  // since the vil_image_view destructor<> does nothing. However, the
  // destructor is virtual so it could be overriden. A little
  // dangerous. -- LJE

  vil_image_view<float>
  NormalizedCrossCorrelation2D(vil_image_view<float> &kernel,
			       vil_image_view<float> &image)
  {
    unsigned int i,j;
    vil_image_view<float> temp1;
    vil_image_view<float> temp2;


    /* Ensure that the kernel and image have at least 2 elements each */
    assert(kernel.ni() > 2 || kernel.nj() > 2);
    assert(image.ni() > 2 || image.nj() > 2);

    /* Ensure that the kernel is smaller than the image. */
    assert(kernel.ni() < image.ni());
    assert(kernel.nj() < image.nj());

    /* Ensure that the kernel and image both are greyscale (only one
       image plane) */
    assert(kernel.nplanes() == 1 && image.nplanes() == 1);

    /* 
     * Run the un-normalized correlation algorithm.
     */
    vil_image_view<float> cross_corr = xcorr2_fast(image, kernel);

    /*
     * Precompute the normalization factors.
     */
    vil_image_view<float> local_sum_A = local_sum(image, kernel);
    vil_math_image_product(image, image, temp1);
    vil_image_view<float> local_sum_A2 = local_sum(temp1, kernel);

    /*
     * Compute the first half of the denominator, which is
     *  the same thing as this string of MATLAB commands:
     *
     * denom_A = sqrt( ( local_sum_A2 - (local_sum_A.^2)/mn ) / (mn - 1) )
     */
    double mn = (double)kernel.ni() * (double)kernel.nj();
    vil_math_image_product(local_sum_A, local_sum_A, temp1);
    vil_math_scale_values(temp1, 1.0 / mn);
    vil_math_image_difference(local_sum_A2, temp1, temp2);
    vil_math_scale_values(temp2, 1.0 / (mn - 1.0));
    vil_math_sqrt(temp2);
  
    /*
     * Compute the second half of the denominator, which is
     *  the same thing as this string of MATLAB commands:
     *
     * denom_T = std(kernel(:))
     */
    double k_std, k_mean;
    vil_math_mean(k_mean, kernel, 0);
    image_std_dev(k_std, kernel, 0);

    /*
     * Combine the two to create the final denominator.
     *
     * denom = denom_T * denom_A
     */
    vil_math_scale_values(temp2, k_std);

    vil_image_view<float> denom(image.ni() + 1 - kernel.ni(),
				 image.nj() + 1 - kernel.nj(),
				 image.nplanes());

    for (i = 0; i < denom.ni(); i++) {
      for (j = 0; j < denom.nj(); j++) {
	denom(i,j) = temp2(i + kernel.ni()-1, j + kernel.nj()-1);
      }
    }
  
    /*
     * Now, create the numerator.
     * 
     * numerator = (cross_corr - local_sum_A(kernel(:))/mn) / (mn - 1) 
     */

    temp1.set_size(image.ni() + 1 - kernel.ni(),
		   image.nj() + 1 - kernel.nj(),
		   image.nplanes());
    for (i = 0; i < temp1.ni(); i++) {
      for (j = 0; j < temp1.nj(); j++) {
	temp1(i,j) = local_sum_A(i + kernel.ni()-1, j + kernel.nj()-1);
	temp1(i,j) *= k_mean;
      }
    }
  
    vil_math_image_difference(cross_corr, temp1, temp2);
    vil_math_scale_values(temp2, mn - 1);

    /* 
     * Finally fill in the cross correlation with the final quotient.
     *
     * C(i_nonzero) = numerator(i_nonzero ./ denom(i_nonzero)
     */
    vil_fill(cross_corr, 0.0f);
    for (i = 0; i < cross_corr.ni(); i++) {
      for (j = 0; j < cross_corr.nj(); j++) {
	if(denom(i,j) != 0) {
	  cross_corr(i,j) = temp2(i,j) / denom(i,j);
	} 
      }
    }

    return cross_corr;
  }
