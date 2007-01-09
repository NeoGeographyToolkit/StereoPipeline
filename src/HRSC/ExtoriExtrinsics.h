/// \file ExtoriExtrinsics.h
/// 
/// Utilities for describing extrinsic camera parameters based on an
/// HRSC extori file.
///
/// 
#ifndef __EXTORI_EXTRINSICS_H__
#define __EXTORI_EXTRINSICS_H__

#include <vw/Core/Exception.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>

#include <algorithm>

  class ExtoriPositionInterpolation {
    std::vector<double> m_line_times;
    std::vector<vw::Vector3> m_positions;

  public:
    ExtoriPositionInterpolation(std::vector<double> &line_times,
                                std::vector<vw::Vector3> &positions) : 
      m_line_times(line_times), m_positions(positions) {}

    vw::Vector3 operator()(double t) const {
      std::vector<double>::const_iterator lower_bound_iter = std::lower_bound(m_line_times.begin(), m_line_times.end(), t);
      int lower_bound_position = &(*lower_bound_iter) - &(m_line_times[0]);
      if (lower_bound_iter+1 == m_line_times.end()) {
        return m_positions[lower_bound_position];
      } else if (lower_bound_iter != m_line_times.end()) {
        double norm_t = (t - *lower_bound_iter) / ((*lower_bound_iter+1)-*lower_bound_iter);
        return m_positions[lower_bound_position] + (m_positions[lower_bound_position+1] - m_positions[lower_bound_position]) * norm_t;
      } else {
        throw vw::ArgumentErr() << "ExtoriPositionInterpolation: requested a position out of range (t=" << t << "  max_t=" << m_line_times[m_line_times.size()-1]<< ".";
      }
    }

  };


  /// Performs interpolation between sparse pose data points using the
  /// spherical linear interpolation algorithm.
  class ExtoriPoseInterpolation {
    std::vector<double> m_line_times;
    std::vector<vw::Quaternion<double> > m_pose_samples;

    static vw::Quaternion<double> slerp(double alpha, vw::Quaternion<double> const& a, vw::Quaternion<double> const& b, int spin) {
      const double SLERP_EPSILON = 1.0E-6; 	        // a tiny number
      double beta;			// complementary interp parameter 
      double theta;			// angle between A and B 
      double sin_t, cos_t;		// sine, cosine of theta 
      double phi;			// theta plus spins 
      int bflip;			// use negation of B? 
      
      // cosine theta = dot product of A and B 
      cos_t = a(1)*b(1) + a(2)*b(2) + a(3)*b(3) + a(0)*b(0);
      
      // if B is on opposite hemisphere from A, use -B instead 
      if (cos_t < 0.0) {
        cos_t = -cos_t;
        bflip = true;
      } else {
        bflip = false;
      }
      
      // if B is (within precision limits) the same as A,
      // just linear interpolate between A and B.
      // Can't do spins, since we don't know what direction to spin.
      if (1.0 - cos_t < SLERP_EPSILON) {
        beta = 1.0 - alpha;
      } else {				/* normal case */
        theta = acos(cos_t);
        phi = theta + spin * M_PI;
        sin_t = sin(theta);
        beta = sin(theta - alpha*phi) / sin_t;
        alpha = sin(alpha*phi) / sin_t;
      }
      
      if (bflip)
        alpha = -alpha;
      
      // interpolate 
      vw::Quaternion<double> q;
      q(1) = beta*a(1) + alpha*b(1);
      q(2) = beta*a(2) + alpha*b(2);
      q(3) = beta*a(3) + alpha*b(3);
      q(0) = beta*a(0) + alpha*b(0);
      return q;
    }

  public:
    ExtoriPoseInterpolation(std::vector<double> line_times, 
                            std::vector<vw::Quaternion<double> > const& pose_samples) :
      m_line_times(line_times), m_pose_samples(pose_samples) {}
    
    /// Compute the pose at a given time t.  The pose will be an interpolated value
    vw::Quaternion<double> operator()(double t) const {
      std::vector<double>::const_iterator lower_bound_iter = std::lower_bound(m_line_times.begin(), m_line_times.end(), t);
      int lower_bound_position = &(*lower_bound_iter) - &(m_line_times[0]);
      if (lower_bound_iter+1 == m_line_times.end()) {
        return m_pose_samples[lower_bound_position];
      } else if (lower_bound_iter != m_line_times.end() ) {
        double norm_t = t - *lower_bound_iter;

        vw::Quaternion<double> a = m_pose_samples[lower_bound_position];
        vw::Quaternion<double> b = m_pose_samples[lower_bound_position+1];
        vw::Quaternion<double> result = this->slerp(norm_t,a,b,0);

        std::cout << "***> " << *lower_bound_iter << " ("<< lower_bound_position << ")   " << a << "   " << b << "   " << result << "\n";

        return result;

      } else {
        throw vw::ArgumentErr() << "ExtoriPoseInterpolation: requested a pose out of time range.";
      }


    }

  };



#endif
