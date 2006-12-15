#ifndef __DEM_H__
#define __DEM_H__

#include <vw/Image/ImageView.h>
#include <vw/Math/BBox.h>
#include <vw/Cartography/GeoReference.h>
#include <string>

void write_GMT_script(std::string const prefix,
                      int width, int height, double min_value, double max_value,
                      vw::cartography::GeoReference const& georef);

void write_ENVI_header(std::string header_name, 
                       int width, int height, 
                       double pixelScaling, vw::BBox3 bBox);

#endif // __DEM_H__ 
