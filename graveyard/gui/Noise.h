// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __NOISE_H__
#define __NOISE_H__

#include <vw/Image.h>
#include <cstdlib> 
#include <cmath>

static double randdouble() { return rand()/(double(RAND_MAX)+1); }

static double normrand(double stddev) {
  double u1, u2, v1, v2, s, x;
  do {
    u1 = randdouble();
    u2 = randdouble();
    v1 = 2 * u1 - 1;
    v2 = 2 * u2 - 1;
    s = v1 * v1 + v2 * v2;
  } while (s >= 1);
  x = sqrt(-2 * log(s) / 2) * v1;
  return stddev * x;
}

namespace vw {
  struct SaltPepperNoiseFunctor : UnaryReturnSameType {
    private:
      const double m_fraction;
    public:
      SaltPepperNoiseFunctor(double const& fraction) : m_fraction(fraction) {}

    template <class ArgT>
    typename result<SaltPepperNoiseFunctor(ArgT)>::type
    inline operator()(ArgT const& arg) const {
      double d = randdouble();
      if (d < m_fraction / 2) {
        return ChannelRange<ArgT>::min();
      }
      else if (d < m_fraction) {
        return ChannelRange<ArgT>::max();
      }
      else {
        return arg;
      }
    }
  };

  template <class ImageT>
  inline UnaryPerPixelView<ImageT, SaltPepperNoiseFunctor> salt_pepper_noise(ImageViewBase<ImageT> const& image, double frac) {
    return UnaryPerPixelView<ImageT, SaltPepperNoiseFunctor>(image.impl(), SaltPepperNoiseFunctor(frac));
  }

  struct GaussianNoiseFunctor : UnaryReturnSameType {
    private:
      const double m_stddev;
    public:
      GaussianNoiseFunctor(double const& stddev) : m_stddev(stddev) {}

    template <class ArgT>
    typename result<GaussianNoiseFunctor(ArgT)>::type
    inline operator()(ArgT const& arg) const {
      double d = normrand(m_stddev);

      //Make sure no overflow happens when adding the noise
      if ((d < 0) && (vw::ChannelRange<ArgT>::min() - d > arg)) {
        return ChannelRange<ArgT>::min();
      }
      else if ((d > 0) && (vw::ChannelRange<ArgT>::max() - d < arg)) {
        return ChannelRange<ArgT>::max();
      }
      else {
        return arg + d;
      }
      return 0; // never reached
    }
  };

  template <class ImageT>
  inline UnaryPerPixelView<ImageT, GaussianNoiseFunctor> gaussian_noise(ImageViewBase<ImageT> const& image, double stddev) {
    return UnaryPerPixelView<ImageT, GaussianNoiseFunctor>(image.impl(), GaussianNoiseFunctor(stddev));
  }
} // namespace vw

#endif // __NOISE_H__
