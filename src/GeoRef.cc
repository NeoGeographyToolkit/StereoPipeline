
/* Functions for dealing with georeferenced image files
 * 
 * We make use of the GDAL library for most of our 
 * georeferncing, projections, etc.
 */

#include "GeoRef.h"
#include "DEM.h" // For bBox stuff
#include <vw/Image/Manipulation.h>
#include <vw/Image/Algorithms.h>

// GDAL includes 
#include "gdal.h"
#include "gdal_priv.h"
#include "cpl_string.h"
#include "ogr_spatialref.h"
#include "ogr_api.h"


using namespace vw;
using namespace std;

/* 
 * Converts spherical coordinates to a planetographic, east-positive 
 * coordinate system with sea level define by a bi-axial elliptical 
 * areoid with radii defined according to the IAU2000 standard.
 */
void sphericalToPlanetographic(ImageView<double> pointCloud)
{
  OGRSpatialReference sourceSRS, targetSRS;
  OGRCoordinateTransformation *coordinateTransform;
  double x, y, z;
  int badpoints = 0;

  /* 
   * Create a set of spatial references and define a coordinate
   * transformation between the two reference frames.
   */
  sourceSRS.SetGeogCS( "Mars IAU2000 east/planetocentric spherical coordinate system",
		       "MARS IAU2000 Spherical Areoid",
		       "Mars IAU2000 Spherical Areoid", 
		       MOLA_PEDR_EQUATORIAL_RADIUS, 0,
		       "Meridian", 0.0,                      // No offset of the meridian
		       SRS_UA_DEGREE, 0.0174532925199433 );  // pi/180
  
  //   char *SRS_string = NULL;
  //   sourceSRS.exportToWkt( &SRS_string );
  //   printf( "Source:  %s\n", SRS_string );

  double invFlattening = 1.0 / ( 1.0 - IAU2000_MARS_POLAR_RADIUS / 
				 IAU2000_MARS_EQUATORIAL_RADIUS);
  targetSRS.SetGeogCS( "Mars IAU2000 east/planetocentric elliptical coordinate system",
		       "Mars IAU2000 Elliptical Areoid",
		       "Mars IAU2000 Elliptical Areoid", 
		       IAU2000_MARS_EQUATORIAL_RADIUS, invFlattening,
		       "Meridian", 0.0,
		       SRS_UA_DEGREE, 0.0174532925199433 );

  //   targetSRS.exportToWkt( &SRS_string );
  //   printf( "Target:  %s\n", SRS_string );

  /* 
   * Project the destination coordinates using a cylindrical, 
   * equal area porjection. 
   */
  coordinateTransform = OGRCreateCoordinateTransformation( &sourceSRS,
							   &targetSRS );

  if( coordinateTransform == NULL) 
    throw GeoRefErr() << "GDAL coordinate transformation failed!";

  /*
   * Map (x, y, z) to spherical coordinates
   */
  for (unsigned int i = 0; i < pointCloud.cols(); i++) {
    for (unsigned int j = 0; j < pointCloud.rows(); j++) {      
      if (pointCloud(i,j,0) != MISSING_PIXEL) {

	/* 
	 * Copy the points into temporary memory so that they can be 
	 * reprojected using GDAL.
	 *
	 * We want elevations not raddii, so we subtract off the source
	 * sphere radius (MOLA_PEDR_EQUATORIAL_RADIUS)
	 */
	x = pointCloud(i,j,0);
	y = pointCloud(i,j,1);
	z = pointCloud(i,j,2) - MOLA_PEDR_EQUATORIAL_RADIUS;

	if (!coordinateTransform->Transform( 1, &x, &y, &z ))
	  badpoints++;
	
	// Note: for planetographic coords, longitude is defined
	// positive west! I.e., lonPlanetographic = (360.0 - lonPlanetocentric);
	
	/* 
	 * Copy the points back.
	 */
	pointCloud(i,j,0) = x;  
	pointCloud(i,j,1) = y;
	pointCloud(i,j,2) = z;
      } 
    }
  }
}


static inline double
CalcEllipsoidRadius(double lat, double polarRadius, double equatorialRadius)
{
  double a = equatorialRadius;
  double b = polarRadius;
  double t = atan((a/b) * tan(lat * M_PI / 180.0));
  // For an ellipse in a 2D (x, y) plane:
  // x = a * cos(t) and y = b * sin(t), but on the circle:
  // y/x = tan(lat), so:
  // y/x = (b * sin(t))/(a * cos(t)) = (b/a) * tan(t) = tan(lat), and:
  // t = arctan((a/b) * tan(theta))
  double x = a * cos(t);
  double y = b * sin(t);
  double radius = sqrt(x*x + y*y);

  return radius;
}

void sphericalToPlanetocentric(ImageView<double> pointCloud) 
{
  for (unsigned int i = 0; i < pointCloud.cols(); i++) {
    for (unsigned int j = 0; j < pointCloud.rows(); j++) {
      
      if (pointCloud(i,j,0) != MISSING_PIXEL) {
        double radius = CalcEllipsoidRadius(pointCloud(i,j,1),
                                            IAU2000_MARS_POLAR_RADIUS,
                                            IAU2000_MARS_EQUATORIAL_RADIUS);
        pointCloud(i,j,2) -= radius;
      }
    }     
  }
}

/* 
 * Projection adapted from 
 * http://www.progonos.com/furuti/MapProj/Normal/CartHow/HowSanson/howSanson.html
 * 
 * Where lambda is longitudue and phi is latitude:
 * 
 * x = (lambda - lamda_0) * cos ( phi ) + lambda_0
 * y =  phi
 *
 */
void projectToSinusoidal(ImageView<double> pointCloud) 
{
  enum { LON = 0, LAT = 1 };

  /* 
   * First we must compute the center longitude 
   * of the DEM so that we can subtract it off for 
   * the sinusoidal projection
   *
   * Note -- this may fail if you are processing a mesh that 
   * crosses the prime meridian.
   */
  BBox3d bBox = FindBBox(pointCloud);
  double lon_0 = bBox.xMin + ((bBox.xMax - bBox.xMin) / 2.0);

  printf("\tApplying sinusoidal projection.\n");
  for (unsigned int i = 0; i < pointCloud.cols(); i++) {
    for (unsigned int j = 0; j < pointCloud.rows(); j++) {
      if (pointCloud(i,j,0) != MISSING_PIXEL) {
	pointCloud(i,j,LON) = (pointCloud(i,j,LON) - lon_0)
	                       * cos(pointCloud(i,j,LAT) * M_PI / 180.0);
	pointCloud(i,j,LON) += lon_0;
      }
    }
  }
}


void MapToSphericalCoordinates(ImageView<double> pointCloud) {

  float x,y,z;
  double radius, lat, lon;
  double sinLat, cosLat;

  // Map cartesian (x, y, z) to spherical (lon, lat, radius)
  for (unsigned int i = 0; i < pointCloud.cols(); i++) {
    for (unsigned int j = 0; j < pointCloud.rows(); j++) {
      if (pointCloud(i,j,0) != MISSING_PIXEL) {

	x = pointCloud(i,j,0);
	y = pointCloud(i,j,1);
	z = pointCloud(i,j,2);

	radius = sqrt(x*x + y*y + z*z);
	sinLat = (z / radius);
      
	// The following assumes latitude is measured from the equatorial
	// plane with north positive. This is different than normal
	// spherical coordinate conversion where the equivalent angle is
	// measured from the positive z axis.
	if (sinLat > 1.0)
	  sinLat = 1.0;
	else if (sinLat < -1.0)
	  sinLat = -1.0;
	cosLat = sqrt(1.0 - sinLat * sinLat);
	
	lat = asin(sinLat);
	
	//   sin(lon) = (y / (radius * cosLat));
	//   cos(lon) = (x / (radius * cosLat));
	// so
	//  tan(lon) = y/x
	lon = atan2(y, x);
	
	pointCloud(i,j,0) = lon * 180.0 / M_PI;
        pointCloud(i,j,1) = lat * 180.0 / M_PI;
	pointCloud(i,j,2) = radius; 
      }
    }
  }
}


void MapToAreoid(ImageView<double> pointCloud,
		 CoordType coordType) {
  switch (coordType)
  {
  case ePlanetoGraphic:
    sphericalToPlanetographic(pointCloud);
    break;
  case ePlanetoCentric:
    sphericalToPlanetocentric(pointCloud);
    break;
  case eSinusoidal:
    // Subtract off the ellipsoid
    sphericalToPlanetocentric(pointCloud);

    // Convert to sinusoidal projection
    //
    // Turned off at Malin's request -mbroxton
    projectToSinusoidal(pointCloud);
    break;
  case eSpherical:
    break;
  }
}

// Template specializations for selecting the GDAL data type fors
// various VW channel formats.
template <class T> struct GDALChannelTraits { static const GDALDataType type = 0; };
template<> struct GDALChannelTraits<vw::uint8 > { static const GDALDataType type = GDT_Byte;    };
template<> struct GDALChannelTraits<vw::uint16> { static const GDALDataType type = GDT_UInt16;  };
template<> struct GDALChannelTraits<vw::int16 > { static const GDALDataType type = GDT_Int16;   };
template<> struct GDALChannelTraits<vw::uint32> { static const GDALDataType type = GDT_UInt32;  };
template<> struct GDALChannelTraits<vw::int32 > { static const GDALDataType type = GDT_Int32;   };
template<> struct GDALChannelTraits<float     > { static const GDALDataType type = GDT_Float32; };
template<> struct GDALChannelTraits<double    > { static const GDALDataType type = GDT_Float64; };

// Function for writing georeferenced files to disk.
//
// Arguments:
//
//   image    -- the raster data to save to disk
//   filename -- the file name
//   fileFormat -- A string specifying the format of the files.  See 
//               the comment at the top of this file for a partial list
//               of supported formats.
//   transform -- An affine transform relates pixel coordinates (positive, 
//               integer pairs) to coordinates in the spatial reference 
//               frame.  This include a translation to fix the position of 
//               the upper left pixel, and a scaling factor to describe the
//               ratio of pixels to units of length (or degrees or whatever).
//               Rotation may also be included in this representation by 
//               changing the values of the off-diagonal entries.  
//
template <class PixelT>
void write_georef_file(vw::ImageView<PixelT> &input_image, std::string filename, 
                       std::string fileFormat, vw::Matrix<double> transform,
                       double centerLongitude) {
  // Use Vision Workbench pixel traits to infer the type of the
  // channel values and then convert this image to a multiplane image
  // if possible.
  typedef typename PixelChannelType<PixelT>::type channel_type;
  vw::ImageView<channel_type> image = vw::channels_to_planes(input_image);

  // Register the various file types with GDAL 
  GDALAllRegister();  
 
  // Open the appropriate GDAL I/O driver, depending on the fileFormat
  // argument specified by the user.
  GDALDriver *poDriver = GetGDALDriverManager()->GetDriverByName(fileFormat.c_str());  
  if( poDriver == NULL )
    throw vw::IOErr() << "Error opening selected GDAL file I/O driver.";
  
  char** papszMetadata = poDriver->GetMetadata();
  if( !CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATE, FALSE ) )
    throw vw::IOErr() << "Selected GDAL driver not supported.\n";

  char **options = NULL;
  GDALDataType pixFmt = GDALChannelTraits<channel_type>::type;
  GDALDataset *dataset = poDriver->Create( filename.c_str(), image.cols(), image.rows(), image.planes(), pixFmt, options );
  
  double adfGeoTransform[6] = { transform(0,2), transform(0,0), transform(0,1), transform(1,2), transform(1,0), transform(1,1) };
  dataset->SetGeoTransform( adfGeoTransform );

  // Set up the spatial reference for this image.
  OGRSpatialReference oSRS;
  double invFlattening = 1.0 / ( 1.0 - IAU2000_MARS_POLAR_RADIUS / 
				 IAU2000_MARS_EQUATORIAL_RADIUS);
  //  oSRS.SetProjCS( "Mars Sinusoidal Projected Coordinate System" );
  oSRS.SetGeogCS( "Mars IAU2000 east/planetocentric elliptical coordinate system",
                  "Mars IAU2000 Elliptical Areoid",
                  "Mars IAU2000 Elliptical Areoid", 
                  IAU2000_MARS_EQUATORIAL_RADIUS, invFlattening,
                  "Meridian", 0.0,
                  SRS_UA_DEGREE, 0.0174532925199433 );
  //  oSRS.SetSinusoidal( centerLongitude , 0, 0);
  
  char *wellKnownTextStr = NULL;
  oSRS.exportToWkt( &wellKnownTextStr );
  dataset->SetProjection( wellKnownTextStr );
  
  // For debugging:
  //   printf("%s\n", wellKnownTextStr);
  //   char *proj4Str = NULL;
  //   oSRS.exportToProj4( &proj4Str );
  //   printf("%s\n", proj4Str);
  CPLFree( wellKnownTextStr );
  
  // Write the data to the selected raster band. 
  for (unsigned int p = 0; p < image.planes(); p++) {
    ImageView<channel_type> plane = vw::select_plane(image, p);
    vil_image_view<channel_type> temp_plane = plane.vil_view();

    printf("\tWriting geo-referenced file %s (%d x %d) with %d band(s).\n",
	   filename.c_str(), image.cols(), image.rows(), image.planes());

    GDALRasterBand *band = dataset->GetRasterBand(p+1);
    band->RasterIO( GF_Write, 0, 0, image.cols(), image.rows(), 
                    temp_plane.top_left_ptr(), image.cols(), image.rows(), 
                    pixFmt, 0, 0 );
  }  
  // Close the file 
  delete dataset;

  printf("\tWrote geo-referenced file %s (%d x %d) with %d band(s).\n",
	 filename.c_str(), image.cols(), image.rows(), image.planes());
}

// Template instantiations
template void write_georef_file(vw::ImageView<vw::uint8> &image,
                                std::string filename, std::string fileFormat,
                                vw::Matrix<double> transform, double centerLongitude);
template void write_georef_file(vw::ImageView<vw::int16> &image,
				std::string filename, std::string fileFormat,
				vw::Matrix<double> transform, double centerLongitude);

template void write_georef_file(vw::ImageView<vw::uint16> &image,
				std::string filename, std::string fileFormat,
				vw::Matrix<double> transform, double centerLongitude);

template void write_georef_file(vw::ImageView<float> &image,
				std::string filename, std::string fileFormat,
				vw::Matrix<double> transform, double centerLongitude);

template void write_georef_file(vw::ImageView<double> &image,
				std::string filename, std::string fileFormat,
				vw::Matrix<double> transform, double centerLongitude);



