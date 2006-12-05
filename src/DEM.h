#ifndef __DEM_H__
#define __DEM_H__

#include "stereo.h"
#include <vw/Image/ImageView.h>
#include <vw/Image/PixelTypes.h>

// Structs
typedef struct
{
  double xMin, yMin, zMin;
  double xMax, yMax, zMax;
} BBox3d;			// 3D bounding box


BBox3d FindBBox(vw::ImageView<double> pointCloud);

void WriteDEM(vw::ImageView<double> pointCloud,
              vw::ImageView<double> texture,
              std::string filename_prefix,
              double dem_spacing);

void WriteDRG(vw::ImageView<double> pointCloud,
              vw::ImageView<vw::PixelGray<double> > texture,
              std::string filename_prefix,
              double dem_spacing);

#endif /* __DEM_H__ */
