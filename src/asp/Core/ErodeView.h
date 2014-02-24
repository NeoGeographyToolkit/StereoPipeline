// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file ErodeView.h
///

#ifndef __ASP_CORE_ERODE_VIEW_H__
#define __ASP_CORE_ERODE_VIEW_H__

// Standard
#include <vector>

// VW
#include <vw/Image/Algorithms.h>
#include <vw/Image/ImageViewBase.h>

// ASP
#include <asp/Core/BlobIndexThreaded.h>

// Erode View
// This takes in an image and invalidates spots based on the blobs detected
// by blobindexthreaded
template <class ViewT>
class ErodeView : public vw::ImageViewBase<ErodeView<ViewT> > {

  vw::ImageViewBase<ViewT> const& m_child;
  std::vector<vw::BBox2i>  m_bboxes;
  std::vector<blob::BlobCompressed> m_blobs;

 public:
  typedef typename ViewT::pixel_type pixel_type;
  typedef typename ViewT::pixel_type result_type; // Have to copy
  typedef vw::ProceduralPixelAccessor<ErodeView<ViewT> > pixel_accessor;

  ErodeView( vw::ImageViewBase<ViewT> const& image,
             BlobIndexThreaded const& bindex ) :
  m_child(image), m_bboxes(bindex.num_blobs()), m_blobs(bindex.num_blobs()) {
    std::copy(bindex.begin(),bindex.end(),m_blobs.begin());
    std::copy(bindex.bbox_begin(),bindex.bbox_end(),m_bboxes.begin());
  }

  inline vw::int32 cols() const { return m_child.impl().cols(); }
  inline vw::int32 rows() const { return m_child.impl().rows(); }
  inline vw::int32 planes() const { return 1; } // Not allowed .

  inline pixel_accessor origin() const { return pixel_accessor(*this,0,0); }

  inline result_type operator()( vw::int32 i, vw::int32 j, vw::int32 /*p*/=0 ) const {
    // Ideally this should be an R-tree.
    // - Searching through all bounding boxes for each pixel will be
    //    very slow for rasterizing.

    vw::Vector2i lookup(i,j); // The requested pixel location
    std::vector<blob::BlobCompressed>::const_iterator blob = m_blobs.begin();
    for ( std::vector<vw::BBox2i>::const_iterator bbox = m_bboxes.begin();
          bbox != m_bboxes.end(); bbox++ ) {  // Loop over all blob bounding boxes
      if ( bbox->contains(lookup) ) {         // Check if this blob's bounding box contains this pixel
        // Determing now if the compressed blob really does contain this point
        vw::Vector2i local = lookup - bbox->min();
        typedef std::list<vw::int32>::const_iterator inner_iter;
        for ( inner_iter start = blob->start(local.y()).begin(), // Loop over all segments on this row of the blob
                         end   = blob->end(local.y()).begin();
              start != blob->start(local.y()).end();
              start++, end++ ) {
          if ( local.x() >= *start && local.x() < *end )        // Check if the pixel is contained in this row segment
            return result_type(); // zero or invalid
        }
      }
      blob++;
    }
    return m_child.impl()(i,j); // This pixel is not in a blob so call the underlying image to get the pixel value
  }

  typedef ErodeView<ViewT> prerasterize_type;
  inline prerasterize_type prerasterize( vw::BBox2i const& /*bbox*/ ) const { return *this; }
  template <class DestT>
  inline void rasterize( DestT const& dest, vw::BBox2i const& bbox ) const {
    vw::rasterize( prerasterize(bbox), dest, bbox );
  }

};

#endif//__ERODEVIEW_H__
