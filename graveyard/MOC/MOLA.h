// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file MOLA.h
///

#ifndef __MOC_MOLA_H__
#define __MOC_MOLA_H__

#include <asp/Sessions/MOC/Metadata.h>
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
