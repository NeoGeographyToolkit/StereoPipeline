// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <asp/Sessions/RPC/RPCModel.h>

#include <vw/Math/Vector.h>
#include <vw/FileIO/GdalIO.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/Cartography.h>

using namespace vw;

namespace asp {

  void RPCModel::initialize( DiskImageResourceGDAL* resource ) {
    // Extract Datum (by means of GeoReference)
    cartography::GeoReference georef;
    cartography::read_georeference( georef, *resource );
    m_datum = georef.datum();

    // Extract RPC Info
    boost::shared_ptr<GDALDataset> dataset = resource->get_dataset_ptr();
    if ( !dataset )
      vw_throw( NotFoundErr() << "RPCModel: Could not read data. No file has been opened." );

    GDALRPCInfo gdal_rpc;
    if ( !GDALExtractRPCInfo( dataset->GetMetadata("RPC"),
                              &gdal_rpc) )
      vw_throw( NotFoundErr() << "RPCModel: GDAL resource appears not to have RPC metadata." );

    // Copy information over to our data structures.
    m_lonlatheight_offset = Vector3(gdal_rpc.dfLONG_OFF,
                                    gdal_rpc.dfLAT_OFF,
                                    gdal_rpc.dfHEIGHT_OFF);
    m_lonlatheight_scale = Vector3(gdal_rpc.dfLONG_SCALE,
                                   gdal_rpc.dfLAT_SCALE,
                                   gdal_rpc.dfHEIGHT_SCALE);
    m_xy_offset = Vector2(gdal_rpc.dfSAMP_OFF,   gdal_rpc.dfLINE_OFF);
    m_xy_scale  = Vector2(gdal_rpc.dfSAMP_SCALE, gdal_rpc.dfLINE_SCALE);

    m_line_num_coeff =   CoeffVec(gdal_rpc.adfLINE_NUM_COEFF);
    m_line_den_coeff =   CoeffVec(gdal_rpc.adfLINE_DEN_COEFF);
    m_sample_num_coeff = CoeffVec(gdal_rpc.adfSAMP_NUM_COEFF);
    m_sample_den_coeff = CoeffVec(gdal_rpc.adfSAMP_DEN_COEFF);
  }

  RPCModel::RPCModel( std::string const& filename ) {
    boost::scoped_ptr<DiskImageResourceGDAL> s_ptr( new DiskImageResourceGDAL(filename) );
    initialize( s_ptr.get() );
  }

  RPCModel::RPCModel( DiskImageResourceGDAL* resource  ) {
    initialize( resource );
  }

  RPCModel::RPCModel( cartography::Datum const& datum,
                      Vector<double,20> const& line_num_coeff,
                      Vector<double,20> const& line_den_coeff,
                      Vector<double,20> const& samp_num_coeff,
                      Vector<double,20> const& samp_den_coeff,
                      Vector2 const& xy_offset,
                      Vector2 const& xy_scale,
                      Vector3 const& lonlatheight_offset,
                      Vector3 const& lonlatheight_scale ) :
    m_datum(datum), m_line_num_coeff(line_num_coeff),
    m_line_den_coeff(line_den_coeff), m_sample_num_coeff(samp_num_coeff),
    m_sample_den_coeff(samp_den_coeff), m_xy_offset(xy_offset),
    m_xy_scale(xy_scale), m_lonlatheight_offset(lonlatheight_offset),
    m_lonlatheight_scale(lonlatheight_scale) {}

  // All of these implementations are largely inspired by the GDAL
  // code. We don't use the GDAL code unfortunately because they don't
  // make that part of the API available. However I believe this is a
  // safe reinterpretation that is safe to distribute.
  Vector2 RPCModel::point_to_pixel( Vector3 const& point ) const {

    Vector3 geodetic = m_datum.cartesian_to_geodetic( point );
    return geodetic_to_pixel( geodetic );
  }

  Vector2 RPCModel::geodetic_to_pixel( Vector3 const& geodetic ) const {
    CoeffVec term =
      calculate_terms( elem_quot(geodetic - m_lonlatheight_offset,
                                 m_lonlatheight_scale) );

    Vector2 normalized_proj( dot_prod(term,m_sample_num_coeff) /
                             dot_prod(term,m_sample_den_coeff),
                             dot_prod(term,m_line_num_coeff) /
                             dot_prod(term,m_line_den_coeff) );

    return elem_prod( normalized_proj, m_xy_scale ) + m_xy_offset;
  }

}
