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


/// \file stereo_fltr.cc
///
//#define USE_GRAPHICS

#include <asp/Tools/stereo.h>
#include <vw/Stereo/DisparityMap.h>

#include <asp/Core/BlobIndexThreaded.h>
#include <asp/Core/InpaintView.h>
#include <asp/Core/ErodeView.h>
#include <asp/Core/ThreadedEdgeMask.h>

using namespace vw;
using namespace asp;

namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}


//=====================================================================================================
//====================================================================================================
//====================================================================================================


//TODO: Move this to its own file!
namespace asp {


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
  return crop(ImageView<typename ImageT::pixel_type>(crop(imageToCrop, bbox)),
              -bbox.min().x(), -bbox.min().y(), imageToCrop.cols(), imageToCrop.rows());
}



// BlobMaskView
/*
    This class is similar to SparseView but instead of storing replacement
    image data it just stores a sparse list of invalid pixels.

    Each time there is a pixel query the list is consulted to see if the pixel is valid.
    If the pixel is not on the list, the value from the underlying image is used.

    TODO: Move function definitions out of the header
*/

template <class ImageT>
class BlobMaskView : public vw::ImageViewBase< BlobMaskView<ImageT> >
{
public: // Definitions

  // Pixel types
  typedef typename ImageT::pixel_type                        pixel_type;
  typedef pixel_type                                         result_type;
  typedef vw::ProceduralPixelAccessor<BlobMaskView<ImageT> > pixel_accessor;

  // Rasterization types
  typedef typename vw::CropView<vw::ImageView<typename ImageT::pixel_type> > inner_pre_type;
  typedef          BlobMaskView<inner_pre_type>                              prerasterize_type;

public: // Functions

  /// Construct class from image and blobs to be masked
  BlobMaskView( vw::ImageViewBase<ImageT>       const& under_image,
                std::list<blob::BlobCompressed> const& blobList)
      : m_under_image(under_image.impl())
  {
    // Create new array for storing blob information, then import.
    m_data.reset( new std::vector<map_type>(under_image.impl().rows()) );
    absorb(blobList);
  }

  /// Construct class from image and blobs to be masked
  BlobMaskView( vw::ImageViewBase<ImageT> const& under_image,
                BlobIndexThreaded         const& blobIndex)
      : m_under_image(under_image.impl())
  {
    // Create new array for storing blob information, then import.
    m_data.reset( new std::vector<map_type>(under_image.impl().rows()) );
    absorb(blobIndex);
  }

  /// Construct class from as image and another instance of this class.
  /// - The blobs from the other instance are applied to the new image.
  template <class T>
  BlobMaskView( vw::ImageViewBase<ImageT> const& under_image,
                BlobMaskView<T>           const& parentInstance)
        : m_data(parentInstance.m_data), m_under_image(under_image.impl())
  {
  }

  // Return size information
  inline vw::int32 cols  () const { return m_under_image.cols(); }
  inline vw::int32 rows  () const { return m_under_image.rows(); }
  inline vw::int32 planes() const { return 1; }

  /// Return an iterator to the first pixel
  inline pixel_accessor origin() const { return pixel_accessor(*this,0,0); }

  /// Return the value for the given pixel.
  inline result_type operator()( vw::int32 i, vw::int32 j, vw::int32 p=0 ) const
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
        //if ((j == 747) && (i >= 9167) && (i <= 9178))
        //  printf("Pixel %d, %d NOT masked out by: %d - %d\n", i, j, startIndex, it->first);
        return m_under_image( i, j, p ); // The segment does not contain the requested pixel, use the underlying image
      }
      else
      {
        //if ((j == 747) && (i >= 9167) && (i <= 9178))
        //  printf("Pixel %d, %d is masked out by: %d - %d\n", i, j, startIndex, it->first);
        return result_type(); // The segment contains the pixel so return a zero or invalid pixel
      }
    }

    return m_under_image( i, j, p ); // No segment contains the requested pixel, use the underlying image
  }

  // Add a single blob to the sparse mask
  void absorb(blob::BlobCompressed const& newBlob)
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
        vw_throw( InputErr() << "\nInput blob row " << fullImageRow
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

          /*if (fullImageRow == 747)
          {
            printf("------------------------------------------\n");
            printf("Old: %d to %d\n", oldSegmentStart, oldSegmentEnd);
            printf("New: %d to %d\n", segmentStart,    segmentEnd);
          }*/

          // Handle case where there is no segment overlap or adjacency
          // - In many applications all segments will trigger this condition
          if ( (oldSegmentEnd < (segmentStart-1)) || (oldSegmentStart > (segmentEnd+1)) )
          {
            //if (fullImageRow == 747)
            //  printf("No segment overlap\n");
            // Move on to the next existing segment
            ++i;
            continue;
          }

          // Handle case where existing segment completely contains the new segment
          if ( (oldSegmentStart <= segmentStart) && (oldSegmentEnd >= segmentEnd) )
          {
            if (fullImageRow == 747)
              printf("New is contained\n");
            // Quit this loop and move on to the next new segment
            skipNewSegment = true;
            break;
          }

          // Handle case where new segment completely contains the old segment
          if ( (oldSegmentStart >= segmentStart) && (oldSegmentEnd <= segmentEnd) )
          {
            //if (fullImageRow == 747)
            //  printf("Old is contained\n");
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
            //if (fullImageRow == 747)
            //  printf("Generic overlap!\n");

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

          vw_throw( LogicErr() << "\nFailed to compare row segments!\n" );

         } // End of loop through possible overlapping segments

         // If it turned out the new segment is redundant, move on to the next one.
         if (skipNewSegment)
           continue;

         // If we got to here we are finally ready to add the new segment!
         // - Remember that we used the end of the segment as the key.
         std::pair<vw::int32,vw::int32> newSegment(segmentEnd, segmentStart);
         blobRow(fullImageRow).insert(newSegment);

      } // End loop through new segments in this row

      /*if (fullImageRow == 747)
      {
        printf("Row contents: \n");
        map_type::iterator iter;
        for (iter=blobRow(fullImageRow).begin();
            iter!=blobRow(fullImageRow).end(); ++iter)
        {
          printf("Seg: %d to %d\n", iter->second, iter->first);
        }
      }

      if (fullImageRow == 747)
        printf("Row %d contains %d segments.\n", fullImageRow, blobRow(fullImageRow).size());
      */

    } // End row loop

    // The entire blob should now be added!

  } // End function "void absorb(const vw::blob::BlobCompressed &newBlob)"

  /// Adds a new set of blobs to the sparse mask
  void absorb(const std::list<blob::BlobCompressed> &blobList)
  {
    // Loop through all the blobs in the blob index and absorb them one at a time
    std::list<blob::BlobCompressed>::const_iterator iter;
    for (iter=blobList.begin(); iter!=blobList.end(); ++iter)
      absorb(*iter);
  }

  /// Adds a new set of blobs to the sparse mask
  void absorb(const BlobIndexThreaded &blobIndex)
  {
    // Loop through all the blobs in the blob index and absorb them one at a time
    BlobIndexThreaded::const_blob_iterator iter;
    for (iter=blobIndex.begin(); iter!=blobIndex.end(); ++iter)
      absorb(*iter);
  }

  /// Generate a new BlobMaskView of the cropped, rasterized underlying image.
  inline prerasterize_type prerasterize( vw::BBox2i const& bbox ) const
  {
    //std::cout << "prerasterizing " << bbox << std::endl;

    // Rasterize only the required portion of the input image, then wrap it so
    //  that it looks like the full image for coordinate access purposes.
    inner_pre_type preraster = makeHiddenCroppedView(m_under_image, bbox);

    // Create a new BlobMaskView to wrap the cropped rasterized image
    BlobMaskView<inner_pre_type> croppedBlobMasked( preraster, *this );
    return croppedBlobMasked;
  }

  /// Rasterize all pixels in bbox to the output destination.
  template <class DestT>
  inline void rasterize( DestT const& dest, vw::BBox2i const& bbox ) const
  {
    /// Call the default rasterization function on the prerasterized image section.
    vw::rasterize( prerasterize(bbox), dest, bbox );
  }


  /// Write out the sparse container contents showing which pixels are masked.
  void print_structure() const
  {
    using namespace vw;
    vw_out() << "BlobMaskView Structure:\n";
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

private: // Definitions

  // Allow different instantiations of BlobMaskView to access each other's private stuff
  template <typename>
  friend class BlobMaskView;

  // The key in our map is the index marking the end of the
  // vector. This somewhat confusing method is to allow us better use of
  // the container's search method.
  // - The first element is the last masked entry in the segment
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


}; // End class BlobMaskView

/// Convenience wrapper to construct from list of BlobCompressed
template <class SourceT>
inline BlobMaskView<SourceT> blobMask( vw::ImageViewBase<SourceT>      const& src,
                                       std::list<blob::BlobCompressed> const& blobList) {
  return BlobMaskView<SourceT>(src, blobList);
}

/// Convenience wrapper to construct from BlobIndexThreaded
template <class SourceT>
inline BlobMaskView<SourceT> blobMask( vw::ImageViewBase<SourceT> const& src,
                                       BlobIndexThreaded          const& bindex) {
  return BlobMaskView<SourceT>(src, bindex);
}

} // end namespace asp






//====================================================================================================
//====================================================================================================
//====================================================================================================


// Run several cleanup passes with desired cleanup mode.
template <class ViewT>
struct MultipleDisparityCleanUp {
  typedef ImageViewRef< typename ViewT::pixel_type > result_type;
  
  inline result_type operator()( ImageViewBase<ViewT> const& input, int N) {

    result_type out = input;
    for (int i = 0; i < N; i++){
      int mode = stereo_settings().filter_mode;
      if (mode == 1){
        out = stereo::disparity_cleanup_using_mean
          (out.impl(),
           stereo_settings().rm_half_kernel.x(),
           stereo_settings().rm_half_kernel.y(),
           stereo_settings().max_mean_diff);
      }else if (mode == 2){
        out = stereo::disparity_cleanup_using_thresh
          (out.impl(),
           stereo_settings().rm_half_kernel.x(),
           stereo_settings().rm_half_kernel.y(),
           stereo_settings().rm_threshold,
           stereo_settings().rm_min_matches/100.0);
      }else
        vw_throw( ArgumentErr() << "\nExpecting value of 1 or 2 for filter-mode. "
                  << "Got: " << mode << "\n" );
    }

    return out;
  }
};

template <class ImageT>
void write_good_pixel_and_filtered( ImageViewBase<ImageT> const& inputview,
                                    Options const& opt ) {
  { // Write Good Pixel Map
    // Sub-sampling so that the user can actually view it.
    float sub_scale =
      float( std::min( inputview.impl().cols(),
                       inputview.impl().rows() ) ) / 2048.0;

    asp::block_write_gdal_image( opt.out_prefix + "-GoodPixelMap.tif",
                                 subsample(
                                           apply_mask(
                                                      copy_mask(stereo::missing_pixel_image(inputview.impl()),
                                                                create_mask(DiskImageView<vw::uint8>(opt.out_prefix+"-lMask.tif"), 0)
                                                               )
                                                     ),
                                           sub_scale < 1 ? 1 : sub_scale
                                          ),
                                 opt, TerminalProgressCallback
                                 ("asp", "\t--> Good Pxl Map: ") );
  }
  
  // TODO: Add a variable to control these!
  int  SMALL_BLOB_PIXEL_CUTOFF = stereo_settings().erode_max_size;
  bool REMOVE_SMALL_BLOBS      = SMALL_BLOB_PIXEL_CUTOFF > 0;

  // Fill holes
  if(!stereo_settings().disable_fill_holes)
  {
    // Generate a list of blobs below a maximum size
    // - This requires the entire input image to be read in
    //    and produces a single blob list for the entire image.
    vw_out() << "\t--> Filling holes with Inpainting method.\n";
    BlobIndexThreaded smallHoleIndex( invert_mask( inputview.impl() ),
                                      stereo_settings().fill_hole_max_size );
    vw_out() << "\t    * Identified " << smallHoleIndex.num_blobs() << " holes\n";
    bool use_grassfire = true;
    typename ImageT::pixel_type default_inpaint_val;


    if (!REMOVE_SMALL_BLOBS) // Skip small blob removal
    {
      // Write out the image to disk, filling in the blobs in the process
      asp::block_write_gdal_image( opt.out_prefix + "-F.tif",
                                   inpaint(inputview.impl(), smallHoleIndex,
                                           use_grassfire, default_inpaint_val),
                                   opt, TerminalProgressCallback
                                   ("asp","\t--> Filtering: ") );
    }
    else // Add small blob removal step
    {
      // Get a list of small blobs, almost identical to how the fill holes
      vw_out() << "\t--> Removing small blobs.\n";
      BlobIndexThreaded smallBlobIndex( inputview.impl(), SMALL_BLOB_PIXEL_CUTOFF );
      vw_out() << "\t    * Identified " << smallBlobIndex.num_blobs() << " small blobs\n";


      std::list<blob::BlobCompressed> fullBlobList;

      // Add locations from the hole-filling step that are inside the blobs to be removed
      // - Otherwise small holes in the removed blobs will be filled in!
      BlobIndexThreaded::blob_iterator blobIter, holeIter;
      // Loop through blobs
      for (blobIter=smallBlobIndex.begin(); blobIter!=smallBlobIndex.end(); ++blobIter) {
        fullBlobList.push_back(*blobIter);
        // Loop through holes
        for (holeIter=smallHoleIndex.begin(); holeIter!=smallHoleIndex.end(); ++holeIter) {
          // If this hole is completely contained in this blob
          if ( blobIter->bounding_box().contains(holeIter->bounding_box()) ) {
            // Add that hole as a blob to remove
            fullBlobList.push_back(*holeIter);
            //std::cout << "Removing hole in blob with " << blobIter->num_rows() << " lines." << std::endl;
          }
        } // End hole loop
      } // End blob loop


      // Write out the image to disk, filling in and removing blobs in the process
      // - Blob removal is done second to make sure inner-blob holes are removed.
      asp::block_write_gdal_image( opt.out_prefix + "-F.tif",
                                      blobMask(inpaint(inputview.impl(), smallHoleIndex,
                                                       use_grassfire, default_inpaint_val),
                                                       fullBlobList),
                                   opt, TerminalProgressCallback
                                   ("asp","\t--> Filtering: ") );
    }

  } else  // No hole filling
  {
    if (!REMOVE_SMALL_BLOBS) // Skip small blob removal
    {
      asp::block_write_gdal_image( opt.out_prefix + "-F.tif",
                                       inputview.impl(), opt,
                                       TerminalProgressCallback("asp", "\t--> Filtering: ") );
    }
    else // Add small blob removal step
    {
      // Get a list of small blobs, almost identical to how the fill holes
      vw_out() << "\t--> Removing small blobs.\n";
      BlobIndexThreaded smallBlobIndex( inputview.impl(), SMALL_BLOB_PIXEL_CUTOFF );
      vw_out() << "\t    * Identified " << smallBlobIndex.num_blobs() << " small blobs\n";

/*
      //DEBUG: Write out the blob group!
      std::ofstream debugFile("/home/smcmich1/data/FRESHMELT2/local/workdir/stereoWorkDir/blobDebug.txt");
      BlobIndexThreaded::blob_iterator iter;
      for (iter=smallBlobIndex.begin(); iter!=smallBlobIndex.end(); ++iter)
      {
        debugFile << iter->bounding_box() << std::endl << std::endl;
      }
      debugFile.close();
*/

      // Write out the image to disk, removing the blobs in the process
      asp::block_write_gdal_image( opt.out_prefix + "-F.tif",
                                   blobMask(inputview.impl(), smallBlobIndex),
                                   opt, TerminalProgressCallback("asp","\t--> Filtering: ") );
    }


  }
}

void stereo_filtering( Options& opt ) {
  vw_out() << "\n[ " << current_posix_time_string() << " ] : Stage 3 --> FILTERING \n";

  std::string post_correlation_fname;
  opt.session->pre_filtering_hook(opt.out_prefix+"-RD.tif",
                                  post_correlation_fname);

  try {

    // Rasterize the results so far to a temporary file on disk.
    // This file is deleted once we complete the second half of the
    // disparity map filtering process.

    // Apply filtering for high frequencies
    typedef DiskImageView<PixelMask<Vector2f> > input_type;
    input_type disparity_disk_image(post_correlation_fname);

    // Applying additional clipping from the edge. We make new
    // mask files to avoid a weird and tricky segfault due to
    // ownership issues.
    DiskImageView<vw::uint8> left_mask ( opt.out_prefix+"-lMask.tif" );
    DiskImageView<vw::uint8> right_mask( opt.out_prefix+"-rMask.tif" );
    int32 mask_buffer = max( stereo_settings().subpixel_kernel );

    vw_out() << "\t--> Cleaning up disparity map prior to filtering processes ("
             << stereo_settings().rm_cleanup_passes << " pass).\n";

    // If the user wants to do no filtering at all, that amounts
    // to doing no passes.
    if (stereo_settings().filter_mode == 0)
      stereo_settings().rm_cleanup_passes = 0;

    if ( stereo_settings().mask_flatfield )
    {
      ImageViewRef<PixelMask<Vector2f> > filtered_disparity;
      if ( stereo_settings().rm_cleanup_passes >= 1 )
      {
        filtered_disparity =
          stereo::disparity_mask
          (MultipleDisparityCleanUp<input_type>()
           (disparity_disk_image, stereo_settings().rm_cleanup_passes),
           apply_mask(asp::threaded_edge_mask(left_mask, 0,mask_buffer,1024)),
           apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));
      }
      else // No cleanup passes
      {
        filtered_disparity =
          stereo::disparity_mask
          (disparity_disk_image,
           apply_mask(asp::threaded_edge_mask(left_mask, 0,mask_buffer,1024)),
           apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024)));
      }
      
      // This is only turned on for apollo. Blob detection doesn't
      // work to great when tracking a whole lot of spots. HiRISE
      // seems to keep breaking this so I've keep it turned off.
      //
      // The crash happens inside Boost Graph when dealing with
      // large number of blobs.
      BlobIndexThreaded bindex( filtered_disparity,
                                stereo_settings().erode_max_size );
      vw_out() << "\t    * Eroding " << bindex.num_blobs() << " islands\n";
      write_good_pixel_and_filtered
        ( ErodeView<ImageViewRef<PixelMask<Vector2f> > >(filtered_disparity,
                                                         bindex ), opt );
    } else {
      // No Erosion step
      if ( stereo_settings().rm_cleanup_passes >= 1 )
      {
        // Apply an outlier removal filter
        write_good_pixel_and_filtered
          (stereo::disparity_mask
            (MultipleDisparityCleanUp<input_type>()
              (disparity_disk_image, stereo_settings().rm_cleanup_passes),
               apply_mask(asp::threaded_edge_mask(left_mask, 0,mask_buffer,1024)),
               apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))),
             opt);
      }
      else // No cleanup passes
      {
        write_good_pixel_and_filtered
          (stereo::disparity_mask
            (disparity_disk_image,
              apply_mask(asp::threaded_edge_mask(left_mask, 0,mask_buffer,1024)),
              apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))),
            opt);
      } // End cleanup passes check
    } // End mask_flatfield check

  } catch (IOErr const& e)
  {
    vw_throw( ArgumentErr() << "\nUnable to start at filtering stage -- could not read input files.\n"
              << e.what() << "\nExiting.\n\n" );
  }
}

int main(int argc, char* argv[]) {

  stereo_register_sessions();
  Options opt;
  try {
    handle_arguments( argc, argv, opt,
                      FilteringDescription() );

    // Internal Processes
    //---------------------------------------------------------
    stereo_filtering( opt );

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : FILTERING FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}
