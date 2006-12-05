/* For a list of supported georefernced formats, refer to
 * /irg/packages/src/gdal-1.3.1/frmts/formats_list.html
 *
 * Here is an abreviated list of formats that are supported for both
 * reading and writing: The string format codes appear to the right.
 * This format code is passed to the fileFormate argument of
 * write_georef_file() and read_georef_file().
 *
 *  Arc/Info ASCII Grid 	                 AAIGrid 
 *  Windows Device Independent Bitmap (.bmp) 	 BMP
 *  VTP Binary Terrain Format (.bt) 	         BT
 *  ERMapper Compressed Wavelets (.ecw) 	 ECW
 *  ENVI .hdr Labelled Raster 	                 ENVI
 *  FITS (.fits) 	                         FITS 
 *  Graphics Interchange Format (.gif) 	         GIF 	 
 *  Arc/Info Binary Grid (.adf) 	         GIO
 *  TIFF / GeoTIFF (.tif) 	                 GTiff
 *  Hierarchical Data Format Release 4 (HDF4) 	 HDF4
 *  Erdas Imagine (.img) 	                 HFA
 *  Atlantis MFF2e 	                         HKV
 *  Image Display and Analysis (WinDisp) 	 IDA
 *  ILWIS Raster Map (.mpr,.mpl) 	         ILWIS
 *  JPEG JFIF (.jpg) 	                         JPEG
 *  JPEG2000 (.jp2, .j2k) 	                 JPEG2000 
 *  JPEG2000 (.jp2, .j2k) 	                 JP2KAK
 *  JPEG2000 (.jp2, .j2k) 	                 JP2ECW
 *  JPEG2000 (.jp2, .j2k) 	                 JP2MrSID
 *  In Memory Raster 	                         MEM 	 
 *  Atlantis MFF 	                         MFF
 *  NITF 	                                 NITF
 *  NetCDF 	                                 netCDF
 *  PCI .aux Labelled 	                         PAux
 *  PCI Geomatics Database File 	         PCIDSK
 *  Portable Network Graphics (.png) 	         PNG
 *  PCRaster (.map) 	                         PCRaster
 *  Netpbm (.ppm,.pgm) 	                         PNM
 *  Raster Matrix Format (*.rsw, .mtw) 	         RMF
 *  X11 Pixmap (.xpm) 	                         XPM
 *
 * Note that many more file formats are available for reading (but not
 * writing.  Also, certain file formats have file size limits.
 * Consult the web page above for more details.
 */

#ifndef __GEOREF_H__
#define __GEOREF_H__

#include <vw/Image/ImageView.h>
#include <vw/Math/Matrix.h>
#include <vw/Image/PixelTypes.h>
#include "stereo.h"

enum CoordType { ePlanetoGraphic, ePlanetoCentric, eSinusoidal, eSpherical };

/* Define an exception for the georef module. */
VW_DEFINE_EXCEPTION(GeoRefErr, vw::Exception);

void MapToAreoid(vw::ImageView<double> pointCloud,
		 CoordType coordType = ePlanetoCentric);
void MapToSphericalCoordinates(vw::ImageView<double> pointCloud);

template<class PixelT>
void write_georef_file(vw::ImageView<PixelT> &input_image, std::string filename, 
                       std::string fileFormat, vw::Matrix<double> transform,
                       double centerLongitude);


#endif // __GEOREF_H__
