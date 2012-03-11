// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <test/Helpers.h>
#include <boost/assign/std/vector.hpp>
#include <asp/Core/GaussianClustering.h>

using namespace vw;
using namespace asp;
using namespace boost::assign;

typedef std::vector<std::pair<vw::Vector<double>, vw::Vector<double> > > Clusters;

TEST( GaussianClustering, ClusterOf2_1D ) {
  std::vector<double> samples;
  samples += 1.2,2.2,3,1.9,2.5,1.784;
  samples += 11.1,12,10.9,11.4,12.1,11.782;

  Clusters c =
    gaussian_clustering<std::vector<double> >( samples.begin(), samples.end(), 2 );
  EXPECT_EQ( c.size(), 2 );
  EXPECT_NEAR( 2, c[0].first[0], 1 );
  EXPECT_NEAR( 11, c[1].first[0], 1 );
  EXPECT_NEAR( 1, c[0].second[0], 1 );
  EXPECT_NEAR( 1, c[1].second[0], 1 );
}

TEST( GaussianClustering, ClusterOf2_2D ) {
  std::vector<Vector2> samples;
  samples += Vector2(-3.2,20.1), Vector2(-2.1,18.6), Vector2(-3.7,21.6),
    Vector2(-2.98,20.7), Vector2(-3.8,19.1),Vector2(-2.4,22.1);
  samples += Vector2(42.2,40.6), Vector2(41.7,41.1), Vector2(41.3,40.1),
    Vector2(43.1,41.5), Vector2(43.5,39.3), Vector2(41.5,41.3);

  Clusters c =
    gaussian_clustering<std::vector<Vector2>, 2>( samples.begin(), samples.end(), 2 );
  EXPECT_EQ( c.size(), 2 );
  EXPECT_VECTOR_NEAR( Vector2(-3,20), c[0].first, 1 );
  EXPECT_VECTOR_NEAR( Vector2(1,1), c[0].second, 1 );
  EXPECT_VECTOR_NEAR( Vector2(42,40), c[1].first, 1 );
  EXPECT_VECTOR_NEAR( Vector2(1,1), c[1].second, 1 );
}

TEST( GaussianClustering, ClusterOf3_1D ) {
  std::vector<double> samples;
  samples += 2.84, 1.62, 2.49, 2.94, 1.66, 1.51;
  samples += 6.92, 6.68, 5.57, 6.56, 8.08, 6.21, 7.33;
  samples += 28.79, 21.23, 24.928, 23.85, 22.24, 24.87, 21.20, 27.14, 24.93, 28.48;

  Clusters c =
    gaussian_clustering<std::vector<double> >( samples.begin(), samples.end(), 3 );
  EXPECT_EQ( 3, c.size() );
  EXPECT_NEAR( 2, c[0].first[0], 1 );
  EXPECT_NEAR( 7, c[1].first[0], 1 );
  EXPECT_NEAR( 25, c[2].first[0], 2 );
  EXPECT_NEAR( 1, c[0].second[0], 1 );
  EXPECT_NEAR( 1, c[1].second[0], 1 );
  EXPECT_NEAR( 5, c[2].second[0], 2 );
}
