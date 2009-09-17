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

/// \file stereo.cc
///
#include <asp/asp_config.h>

#include <asp/Core/BaseEquation.h>
#include <asp/Core/BlobIndexThreaded.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/CameraAdjust.h>
#include <asp/Core/ControlNetworkLoader.h>
#if defined(ASP_HAVE_PKG_SPICE) && ASP_HAVE_PKG_SPICE == 1
#include <asp/Core/DiskImageResourceDDD.h>
#endif
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/Core/DiskImageResourceIsis.h>
#endif
#include <asp/Core/Equation.h>
#include <asp/Core/InpaintView.h>
#include <asp/Core/IsisAdjustCameraModel.h>
#include <asp/Core/IsisCameraModel.h>
#include <asp/Core/OrthoRasterizer.h>
#include <asp/Core/PolyEquation.h>
#include <asp/Core/RPNEquation.h>
#include <asp/Core/SoftwareRenderer.h>
#include <asp/Core/SparseView.h>
#include <asp/Core/SpiceUtilities.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/TabulatedDataReader.h>
#include <asp/Core/nff_terrain.h>
