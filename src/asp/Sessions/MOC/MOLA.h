// __BEGIN_LICENSE__
//
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
//
// Copyright 2008 Carnegie Mellon University. All rights reserved.
//
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
//
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
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
