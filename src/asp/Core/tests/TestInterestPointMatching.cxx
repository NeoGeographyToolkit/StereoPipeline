// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <test/Helpers.h>
#include <asp/Core/InterestPointMatching.h>
#include <vw/Camera/PinholeModel.h>

using namespace vw;
using namespace asp;

TEST( InterestPointMatching, DatumIntersection ) {

  // Make a synthetic camera (Parameters selected to mimic a DG like camera)
  camera::PinholeModel model( Vector3(-414653.934175,-2305310.05912,-6759174.5439),
                              Quat(-0.0794638597818,-0.0396316037899,-0.40945443655,-0.907998840691).rotation_matrix(),
                              1.65e6, 1.65e6, 17500, 17500,
                              Vector3(1,0,0), Vector3(0,1,0), Vector3(0,0,1),
                              camera::NullLensDistortion() );

  // Project back a forth against a flattened sphere
  cartography::Datum datum("WGS84");
  for ( size_t i = 0; i < 35000; i+= 2000 ) {
    for ( size_t j = 0; j < 35000; j+= 2000 ) {
      Vector2 meas( i, j );

      Vector3 pos = datum_intersection( datum, &model, meas );
      Vector3 geo = datum.cartesian_to_geodetic( pos );

      // Verify that the geodetic height is zero (we are on the sphere).
      EXPECT_NEAR( 0, geo.z(), 1e-3 );

      Vector2 ret = model.point_to_pixel( pos );
      EXPECT_VECTOR_NEAR( meas, ret, 1e-3 );
    }
  }

  // Project back and forth against a true sphere
  datum.set_semi_minor_axis( datum.semi_major_axis() );
  for ( size_t i = 0; i < 35000; i+= 2000 ) {
    for ( size_t j = 0; j < 35000; j+= 2000 ) {
      Vector2 meas( i, j );

      Vector3 pos = datum_intersection( datum, &model, meas );
      Vector3 geo = datum.cartesian_to_geodetic( pos );

      // Verify that the geodetic height is zero (we are on the sphere).
      EXPECT_NEAR( 0, geo.z(), 1e-3 );

      Vector2 ret = model.point_to_pixel( pos );
      EXPECT_VECTOR_NEAR( meas, ret, 1e-3 );
    }
  }

}
