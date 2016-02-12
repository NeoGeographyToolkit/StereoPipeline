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

//TODO: This could be moved somewhere (maybe to crop()) and used in other locations.

/// Rasterize a selected portion of the input image, then wrap it so
///   that it looks like the full image for pixel location purposes.
/// - Don't request any pixels outside the bounding box or else something bad will happen!
/// - This function is useful in prerasterize functions to rasterize only a portion of
///     an image but make it look like the entire image.
template <class ImageT>
inline vw::CropView<vw::ImageView<typename ImageT::pixel_type> >
          makeHiddenCroppedView( ImageT &imageToCrop, vw::BBox2i const& bbox )
{
  // - First crop wraps a portion of input image in the bounding box.
  // - The ImageView forces rasterization of the cropped input image into a buffer.
  // - Second crop fakes the coordinates so it looks like it is the full size image.
  return crop(vw::ImageView<typename ImageT::pixel_type>(crop(imageToCrop, bbox)),
              -bbox.min().x(), -bbox.min().y(), imageToCrop.cols(), imageToCrop.rows());
}



// ErodeView
/**
    This class is similar to SparseView but instead of storing replacement
    image data it just stores a sparse list of invalid pixels.

    Each time there is a pixel query the list is consulted to see if the pixel is valid.
    If the pixel is not on the list, the value from the underlying image is used.
*/
template <class ImageT>
class ErodeView : public vw::ImageViewBase< ErodeView<ImageT> >
{
public: // Definitions

  // Pixel types
  typedef typename ImageT::pixel_type                     pixel_type;
  typedef pixel_type                                      result_type;
  typedef vw::ProceduralPixelAccessor<ErodeView<ImageT> > pixel_accessor;

  // Rasterization types
  typedef typename vw::CropView<vw::ImageView<typename ImageT::pixel_type> > inner_pre_type;
  typedef          ErodeView<inner_pre_type>                                 prerasterize_type;

public: // Functions

  /// Construct class from image and blobs to be masked
  ErodeView( vw::ImageViewBase<ImageT>       const& under_image,
             std::list<blob::BlobCompressed> const& blobList);

  /// Construct class from image and blobs to be masked
  ErodeView( vw::ImageViewBase<ImageT> const& under_image,
             BlobIndexThreaded         const& blobIndex);

  /// Construct class from as image and another instance of this class.
  /// - The blobs from the other instance are applied to the new image.
  template <class T>
  ErodeView( vw::ImageViewBase<ImageT> const& under_image,
             ErodeView<T>              const& parentInstance);


  // Return size information
  inline vw::int32 cols  () const { return m_under_image.cols(); } ///< Get the number of rows
  inline vw::int32 rows  () const { return m_under_image.rows(); } ///< Get the number of columns
  inline vw::int32 planes() const { return 1; }                    ///< Get the number of planes TODO!

  /// Return an iterator to the first pixel
  inline pixel_accessor origin() const { return pixel_accessor(*this,0,0); }

  /// Return the value for the given pixel.
  inline result_type operator()( vw::int32 i, vw::int32 j, vw::int32 p=0 ) const;


  // These are the functions to add invalid pixels to the mask
  void absorb(blob::BlobCompressed            const& newBlob);   ///< Add a single blob to the sparse mask
  void absorb(std::list<blob::BlobCompressed> const& blobList);  ///< Adds a new set of blobs to the sparse mask
  void absorb(BlobIndexThreaded               const& blobIndex); ///< Adds a new set of blobs to the sparse mask



  /// Generate a new ErodeView of the cropped, rasterized underlying image.
  inline prerasterize_type prerasterize( vw::BBox2i const& bbox ) const;

  /// Rasterize all pixels in bbox to the output destination.
  template <class DestT>
  inline void rasterize( DestT const& dest, vw::BBox2i const& bbox ) const;

  /// Write out the sparse container contents showing which pixels are masked.
  void print_structure() const;

private: // Definitions

  /// Allow different instantiations of ErodeView to access each other's private stuff
  template <typename>
  friend class ErodeView;

  // The key in our map is the index marking the end of the
  // vector. This somewhat confusing method is to allow us better use of
  // the container's search method.
  // - The first  element is the last  masked entry in the segment
  // - The second element is the first masked entry in the segment
  typedef std::map<vw::int32,vw::int32> map_type; ///< Sparse container

private: // Variables

  // It would be nice to have a shared class to represent a sparse image
  mutable boost::shared_ptr<std::vector<map_type> > m_data; ///< One map for each row of the image
  ImageT  m_under_image;    ///< The underlying image data

private: // Functions

  /// Convenience functions for accessing m_data through the shared pointer
  const map_type& blobRow(const vw::int32 row) const {return (*m_data)[row];}
        map_type& blobRow(const vw::int32 row)       {return (*m_data)[row];}


}; // End class ErodeView

/// Convenience wrapper to construct from a list of BlobCompressed
template <class SourceT>
inline ErodeView<SourceT> applyErodeView( vw::ImageViewBase<SourceT>      const& src,
                                          std::list<blob::BlobCompressed> const& blobList) {
  return ErodeView<SourceT>(src, blobList);
}

/// Convenience wrapper to construct from BlobIndexThreaded
template <class SourceT>
inline ErodeView<SourceT> applyErodeView( vw::ImageViewBase<SourceT> const& src,
                                          BlobIndexThreaded          const& bindex) {
  return ErodeView<SourceT>(src, bindex);
}

// ======================================================================================================
// -- Function definitions


template <class ImageT>
ErodeView<ImageT>::ErodeView( vw::ImageViewBase<ImageT>       const& under_image,
                              std::list<blob::BlobCompressed> const& blobList)
    : m_under_image(under_image.impl())
{
  // Create new array for storing blob information, then import.
  m_data.reset( new std::vector<map_type>(under_image.impl().rows()) );
  absorb(blobList);
}

template <class ImageT>
ErodeView<ImageT>::ErodeView( vw::ImageViewBase<ImageT> const& under_image,
                              BlobIndexThreaded         const& blobIndex)
    : m_under_image(under_image.impl())
{
  // Create new array for storing blob information, then import.
  m_data.reset( new std::vector<map_type>(under_image.impl().rows()) );
  absorb(blobIndex);
}


template <class ImageT>
template <class T>
ErodeView<ImageT>::ErodeView( vw::ImageViewBase<ImageT> const& under_image,
                              ErodeView<T>              const& parentInstance)
    : m_data(parentInstance.m_data), m_under_image(under_image.impl())
{
}


template <class ImageT>
typename ErodeView<ImageT>::result_type ErodeView<ImageT>::operator()( vw::int32 i, vw::int32 j, vw::int32 p) const
{

  // Get next container entry after column 'i'.  Since each container is labeled
  //   with the last column in that section, this finds the section which might contain 'i'.
  typename map_type::const_iterator it;
  it = blobRow(j).lower_bound(i);

  if ( it != blobRow(j).end() )  // If a segment possibly containing 'i' was found
  {
    vw::int32 startIndex = it->second; // Get the starting index of this section
    if ( i < startIndex )
    {
      return m_under_image( i, j, p ); // The segment does not contain the requested pixel, use the underlying image
    }
    else
    {
      return result_type(); // The segment contains the pixel so return a zero or invalid pixel
    }
  }

  return m_under_image( i, j, p ); // No segment contains the requested pixel, use the underlying image
}

template <class ImageT>
void ErodeView<ImageT>::absorb(blob::BlobCompressed const& newBlob)
{
  // All pixels stored in the blob are offset this much from global image coordinates
  const int xOffset = newBlob.min()[0];
  const int yOffset = newBlob.min()[1];

  // Loop through all rows contained by the blob
  const int numRows = newBlob.num_rows();
  for (int row=0; row<numRows; ++row)
  {
    // Verify that we have storage for this row
    const int fullImageRow = row + yOffset;
    if (m_data->size() <= static_cast<size_t>(fullImageRow))
    {
      vw_throw( vw::InputErr() << "\nInput blob row " << fullImageRow
                               << " is greater than underlying image height "
                               << m_data->size() << "!\n" );
    }

    // Loop through all segments for this row
    std::list<vw::int32>::const_iterator iterStart, iterEnd;
    for (iterStart =  newBlob.start(row).begin(), iterEnd = newBlob.end(row).begin();
         iterStart != newBlob.start(row).end();
         ++iterStart, ++iterEnd)
    {
      //TODO: Existing blob code lists the end as ONE AFTER the blob
      //     ---> Need to modify this code so that it stores them the same way!
      // These are the starting and stopping columns of the current segment
      int segmentStart = *iterStart + xOffset;
      int segmentEnd   = *iterEnd   + xOffset - 1;

      // Find the first segment ending after the start of this segment.
      // - Need to use start-1 to catch any segment ending adjacent to the new start.
      // - This is the first possible segment this one will need to interact with.
      map_type::iterator priorEndIter = blobRow(fullImageRow).lower_bound(segmentStart-1);

      // Find the first segment ending after the end of this segment.
      // - This is the last possible segment that this one will need to interact with.
      map_type::iterator nextEndIter  = blobRow(fullImageRow).lower_bound(segmentEnd);

      // Get an iterator to the next element to NOT be processed
      map_type::iterator stopIter = blobRow(fullImageRow).end();
      if (nextEndIter != blobRow(fullImageRow).end())
      {
        stopIter = nextEndIter;
        ++stopIter;
      }

      // Loop from first possible segment to last possible segment (inclusive)
      bool skipNewSegment = false;
      map_type::iterator i=priorEndIter;
      while (i!= stopIter)
      {

        // Get the start and stop columns of the old segment
        int oldSegmentStart = i->second;
        int oldSegmentEnd   = i->first;

        // Handle case where there is no segment overlap or adjacency
        // - In many applications all segments will trigger this condition
        if ( (oldSegmentEnd < (segmentStart-1)) || (oldSegmentStart > (segmentEnd+1)) )
        {
          // Move on to the next existing segment
          ++i;
          continue;
        }

        // Handle case where existing segment completely contains the new segment
        if ( (oldSegmentStart <= segmentStart) && (oldSegmentEnd >= segmentEnd) )
        {
          // Quit this loop and move on to the next new segment
          skipNewSegment = true;
          break;
        }

        // Handle case where new segment completely contains the old segment
        if ( (oldSegmentStart >= segmentStart) && (oldSegmentEnd <= segmentEnd) )
        {
          // Remove this existing segment and move on to the next one
          map_type::iterator tempIter = i;
          ++i;
          blobRow(fullImageRow).erase(tempIter);
          continue;
        }

        // Handle other overlap cases
        if ( ( (oldSegmentEnd   >= (segmentStart-1)) && (oldSegmentEnd   <= (segmentEnd+1)) ) ||
             ( (oldSegmentStart >= (segmentStart-1)) && (oldSegmentStart <= (segmentEnd+1)) )  )
        {
          // Expand the new segment so it fully contains the old segment
          if (oldSegmentStart < segmentStart)
            segmentStart = oldSegmentStart;
          if (oldSegmentEnd > segmentEnd)
            segmentEnd = oldSegmentEnd;

          // Remove this existing segment and move on to the next one
          map_type::iterator tempIter = i;
          ++i;
          blobRow(fullImageRow).erase(tempIter);
          continue;
        }

        vw_throw( vw::LogicErr() << "\nFailed to compare row segments!\n" );

       } // End of loop through possible overlapping segments

       // If it turned out the new segment is redundant, move on to the next one.
       if (skipNewSegment)
         continue;

       // If we got to here we are finally ready to add the new segment!
       // - Remember that we used the end of the segment as the key.
       std::pair<vw::int32,vw::int32> newSegment(segmentEnd, segmentStart);
       blobRow(fullImageRow).insert(newSegment);

    } // End loop through new segments in this row

  } // End row loop

  // The entire blob should now be added!

} // End function "void absorb(const vw::blob::BlobCompressed &newBlob)"

template <class ImageT>
void ErodeView<ImageT>::absorb(const std::list<blob::BlobCompressed> &blobList)
{
  // Loop through all the blobs in the blob index and absorb them one at a time
  std::list<blob::BlobCompressed>::const_iterator iter;
  for (iter=blobList.begin(); iter!=blobList.end(); ++iter)
    absorb(*iter);
}

template <class ImageT>
void ErodeView<ImageT>::absorb(const BlobIndexThreaded &blobIndex)
{
  // Loop through all the blobs in the blob index and absorb them one at a time
  BlobIndexThreaded::const_blob_iterator iter;
  for (iter=blobIndex.begin(); iter!=blobIndex.end(); ++iter)
    absorb(*iter);
}

template <class ImageT>
typename ErodeView<ImageT>::prerasterize_type ErodeView<ImageT>::prerasterize( vw::BBox2i const& bbox ) const
{
  // Rasterize only the required portion of the input image, then wrap it so
  //  that it looks like the full image for coordinate access purposes.
  inner_pre_type preraster = makeHiddenCroppedView(m_under_image, bbox);

  // Create a new ErodeView to wrap the cropped rasterized image
  ErodeView<inner_pre_type> croppedBlobMasked( preraster, *this );
  return croppedBlobMasked;
}

template <class ImageT>
template <class DestT>
void ErodeView<ImageT>::rasterize( DestT const& dest, vw::BBox2i const& bbox ) const
{
  /// Call the default rasterization function on the prerasterized image section.
  vw::rasterize( prerasterize(bbox), dest, bbox );
}

template <class ImageT>
void ErodeView<ImageT>::print_structure() const
{
  using namespace vw;
  vw_out() << "ErodeView Structure:\n";
  for ( uint32 r = 0; r < m_data->size(); ++r )  // For each row
  {
    vw_out() << r << " | ";
    // For each data segment
    for ( typename map_type::const_iterator it = blobRow(r).begin(); it != blobRow(r).end(); ++it )
    {
      vw_out() << "(" << it->second << "->" << it->first << ")";
    }
    vw_out() << "\n";
  }
}





#endif//__ERODEVIEW_H__
