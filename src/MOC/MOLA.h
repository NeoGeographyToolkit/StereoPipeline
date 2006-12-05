#ifndef __MOC_MOLA_H__
#define __MOC_MOLA_H__

#include "MOC/Metadata.h"
#include <vw/Image/ImageView.h>
#include <vw/Math/Vector.h>

void do_mola_comparison(vw::ImageView<vw::Vector3> const& moc_point_image, 
                        MOCImageMetadata const& moc_metadata, 
                        std::string const& output_prefix);

std::vector<vw::Vector2> mola_track(MOCImageMetadata const& moc_metadata,
                        std::string const& output_prefix);

std::vector<vw::Vector2> synthetic_track(vw::ImageView<vw::Vector3> const& moc_point_image, 
                                         MOCImageMetadata const& moc_metadata,
                                         std::string const& output_prefix);


#endif
