// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


#ifndef __ASP_CORE_GAUSSIAN_CLUSTERING_H__
#define __ASP_CORE_GAUSSIAN_CLUSTERING_H__

#include <vw/Core.h>
#include <vw/Math.h>
#include <vector>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

namespace asp {

  // Generic clustering tool (mulitple dimensions and multiple clusters)
  template <class ContainerT, size_t DimensionsT>
  std::vector<std::pair<vw::Vector<double>,vw::Vector<double> > > // for every
  // cluster,
  // mean and
  // variance
  gaussian_clustering( typename ContainerT::iterator begin,
                       typename ContainerT::iterator end,
                       size_t clusters = 2) {
    namespace ba = boost::accumulators;
    using namespace vw;
    typedef ba::accumulator_set<double, ba::stats<ba::tag::variance> > acc_set;
    typedef Vector<double, DimensionsT> VectorD;
    typedef typename ContainerT::iterator IterT;

    std::vector<math::CDFAccumulator<double> > cdf( DimensionsT );
    size_t num_samples = 0;

    // Working out best spot to place seeds for clusters
    for ( IterT i = begin; i != end; i++ ) {
      for ( size_t d = 0; d < DimensionsT; d++ ) {
        cdf[d]( (*i)[d] );
      }
      num_samples++;
    }
    for ( size_t d = 0; d < DimensionsT; d++ )
      cdf[d].update();

    // Start clusters
    std::vector<VectorD > means( clusters ), variances( clusters );
    for ( size_t c = 0; c < clusters; c++ ) {
      double probability = double( 1 + c ) / double( 1 + clusters );
      for ( size_t d = 0; d < DimensionsT; d++ ) {
        means[c][d] = cdf[d].quantile( probability );
      }
      VW_OUT( DebugMessage, "asp" ) << "Cluster " << c << " mean seeded with:\n"
                                    << means[c] << std::endl;
    }

    { // Prime the variance by sorting the samples
      std::vector<acc_set> acc( clusters * DimensionsT );

      // Bin the samples based on their distance from mean
      std::vector<double> distance( clusters );
      for ( IterT i = begin; i != end; i++ ) {
        VectorD s;
        for ( size_t d = 0; d < DimensionsT; d++ )
          s[d] = (*i)[d];

        for ( size_t c = 0; c < clusters; c++ )
          distance[c] = norm_2( s - means[c] );

        size_t min_index =
          std::distance(distance.begin(),
                        std::min_element( distance.begin(), distance.end() ));

        for ( size_t d = 0; d < DimensionsT; d++ )
          acc[ min_index * DimensionsT + d ]( s[d] );
      }

      // Pull mean an variance from accumulators
      for ( size_t c = 0; c < clusters; c++ ) {
        for ( size_t d = 0; d < DimensionsT; d++ ) {
          means[c][d] = ba::mean( acc[ c * DimensionsT + d ] );
          variances[c][d] = std::max(ba::variance( acc[ c * DimensionsT + d ] ),
                                     std::numeric_limits<double>::epsilon());
        }
        VW_OUT( DebugMessage, "asp" ) << "Cluster " << c << " seeded with:\n"
                                      << means[c] << " " << variances[c]
                                      << " " << ba::count( acc[c*DimensionsT] ) << std::endl;
      }
    }

    // Iterate and refine mean and variance estimates
    ssize_t change = 10, previous_set0_size = 0;
    while ( change ) {
      // Precalulate the the scalar on the normal distribution
      std::vector<VectorD > normal_scalar( clusters );
      for ( size_t c = 0; c < clusters; c++ )
        normal_scalar[c] = elem_quot( 1.0, sqrt( 2.0 * M_PI * variances[c] ) );

      std::vector<acc_set> acc( clusters * DimensionsT );

      // Bin the samples based on their probabilities
      std::vector<double> probability( clusters );
      for ( IterT i = begin; i != end; i++ ) {
        VectorD s;
        for ( size_t d = 0; d < DimensionsT; d++ )
          s[d] = (*i)[d];

        for ( size_t c = 0; c < clusters; c++ )
          probability[c] =
            prod(elem_prod(normal_scalar[c], exp( elem_quot(-elem_prod( s-means[c], s-means[c] ), ( 2 * variances[c] ) ) ) ) );

        size_t max_index =
          std::distance(probability.begin(),
                        std::max_element( probability.begin(), probability.end() ));

        for ( size_t d = 0; d < DimensionsT; d++ )
          acc[ max_index * DimensionsT + d ]( s[d] );
      }

      // Pull mean and variance from accumulators
      for ( size_t c = 0; c < clusters; c++ ) {
        for ( size_t d = 0; d < DimensionsT; d++ ) {
          means[c][d] = ba::mean( acc[ c * DimensionsT + d ] );
          variances[c][d] = std::max(ba::variance( acc[ c * DimensionsT + d ] ),
                                     std::numeric_limits<double>::epsilon());
        }
        VW_OUT( DebugMessage, "asp" ) << "Cluster " << c << " updated:\n"
                                      << means[c] << " " << variances[c] << " "
                                      << ba::count(acc[c * DimensionsT]) << std::endl;
      }

      // Update change indicators
      ssize_t count = ba::count( acc[0] );
      change = previous_set0_size - count;
      previous_set0_size = count;
    }

    std::vector<std::pair<Vector<double>, Vector<double> > > results( clusters );
    for ( size_t c = 0; c < clusters; c++ ) {
      results[c].first = means[c];
      results[c].second = variances[c];
    }
    return results;
  }

  // Scalar Version
  template <class ContainerT>
  std::vector<std::pair<vw::Vector<double>,vw::Vector<double> > > // for every
  // cluster,
  // mean and
  // variance
  gaussian_clustering( typename ContainerT::iterator begin,
                       typename ContainerT::iterator end,
                       size_t clusters = 2) {
    namespace ba = boost::accumulators;
    using namespace vw;
    typedef ba::accumulator_set<double, ba::stats<ba::tag::variance> > acc_set;
    typedef double VectorD;
    typedef typename ContainerT::iterator IterT;

    math::CDFAccumulator<double> cdf;
    size_t num_samples = 0;

    // Working out best spot to place seeds for clusters
    for ( IterT i = begin; i != end; i++ ) {
      cdf( (*i) );
      num_samples++;
    }
    cdf.update();

    // Start clusters
    std::vector<VectorD > means( clusters ), variances( clusters );
    for ( size_t c = 0; c < clusters; c++ ) {
      double probability = double( 1 + c ) / double( 1 + clusters );
      means[c] = cdf.quantile( probability );
      VW_OUT( DebugMessage, "asp" ) << "PreSeed Mean " << c << " = " << means[c] << "\n";
    }

    { // Prime the variance by sorting the samples
      std::vector<acc_set> acc( clusters );

      // Bin the samples based on their distance from mean
      std::vector<double> distance( clusters );
      for ( IterT i = begin; i != end; i++ ) {
        VectorD s = (*i);

        for ( size_t c = 0; c < clusters; c++ )
          distance[c] = fabs(s - means[c]);

        size_t min_index =
          std::distance(distance.begin(),
                        std::min_element( distance.begin(), distance.end() ));

        acc[ min_index ]( s );
      }

      // Pull mean an variance from accumulators
      for ( size_t c = 0; c < clusters; c++ ) {
        means[c] = ba::mean( acc[ c ] );
        variances[c] = std::max(ba::variance( acc[ c ] ),
                                std::numeric_limits<double>::epsilon());
        VW_OUT( DebugMessage, "asp" ) << "Cluster " << c << " seeded with:\n"
                                      << means[c] << " " << variances[c] << std::endl;
      }
    }

    // Iterate and refine mean and variance estimates
    ssize_t change = 10, previous_set0_size = 0;
    while ( change ) {
      // Precalulate the the scalar on the normal distribution
      std::vector<VectorD > normal_scalar( clusters );
      for ( size_t c = 0; c < clusters; c++ )
        normal_scalar[c] = 1.0 /sqrt( 2.0 * M_PI * variances[c] );

      std::vector<acc_set> acc( clusters );

      // Bin the samples based on their probabilities
      std::vector<double> probability( clusters );
      for ( IterT i = begin; i != end; i++ ) {
        VectorD s = (*i);

        for ( size_t c = 0; c < clusters; c++ )
          probability[c] =
            normal_scalar[c] * exp( -pow(s-means[c], 2) / ( 2 * variances[c] ) );

        size_t max_index =
          std::distance(probability.begin(),
                        std::max_element( probability.begin(), probability.end() ));

        acc[ max_index ]( s );
      }

      // Pull mean and variance from accumulators
      for ( size_t c = 0; c < clusters; c++ ) {
        means[c] = ba::mean( acc[ c ] );
        variances[c] = std::max(ba::variance( acc[ c ] ),
                                std::numeric_limits<double>::epsilon());
        VW_OUT( DebugMessage, "asp" ) << "Cluster " << c << " updated:\n"
                                      << means[c] << " " << variances[c]
                                      << " " << ba::count( acc[c] ) << std::endl;
      }

      // Update change indicators
      ssize_t count = ba::count( acc[0] );
      change = previous_set0_size - count;
      previous_set0_size = count;
    }

    std::vector<std::pair<Vector<double>, Vector<double> > > results( clusters );
    for ( size_t c = 0; c < clusters; c++ ) {
      results[c].first = Vector<double,1>(means[c]);
      results[c].second = Vector<double,1>(variances[c]);
    }

    return results;
  }
}

#endif//__ASP_CORE_GAUSSIAN_CLUSTERING_H__
