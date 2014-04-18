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
  // Write Good Pixel Map
  // Sub-sampling so that the user can actually view it.
  float sub_scale =
    float( std::min( inputview.impl().cols(),
                     inputview.impl().rows() ) ) / 2048.0;
  
  asp::block_write_gdal_image
    ( opt.out_prefix + "-GoodPixelMap.tif",
      subsample
      (apply_mask
       (copy_mask
        (stereo::missing_pixel_image(inputview.impl()),
         create_mask(DiskImageView<vw::uint8>(opt.out_prefix+"-lMask.tif"), 0)
         )
        ),
       sub_scale < 1 ? 1 : sub_scale
       ),
      opt, TerminalProgressCallback
      ("asp", "\t--> Good Pxl Map: ") );
  
  bool removeSmallBlobs = (stereo_settings().erode_max_size > 0);

  // Fill holes
  if(stereo_settings().enable_fill_holes) {
    // Generate a list of blobs below a maximum size
    // - This requires the entire input image to be read in
    //    and produces a single blob list for the entire image.
    vw_out() << "\t--> Filling holes with inpainting method.\n";
    BlobIndexThreaded smallHoleIndex( invert_mask( inputview.impl() ),
                                      stereo_settings().fill_hole_max_size );
    vw_out() << "\t    * Identified " << smallHoleIndex.num_blobs() << " holes\n";
    bool use_grassfire = true;
    typename ImageT::pixel_type default_inpaint_val;


    if (!removeSmallBlobs) { // Skip small blob removal
      // Write out the image to disk, filling in the blobs in the process
      asp::block_write_gdal_image( opt.out_prefix + "-F.tif",
                                   inpaint(inputview.impl(), smallHoleIndex,
                                           use_grassfire, default_inpaint_val),
                                   opt, TerminalProgressCallback
                                   ("asp","\t--> Filtering: ") );
    }
    else { // Add small blob removal step
      // Get a list of small blobs, almost identical to how the fill holes
      vw_out() << "\t--> Removing small blobs.\n";
      BlobIndexThreaded smallBlobIndex( inputview.impl(), stereo_settings().erode_max_size );
      vw_out() << "\t    * Identified " << smallBlobIndex.num_blobs() << " small blobs\n";


      // Set locations from the hole-filling step that are inside the blobs to be removed.
      // - Otherwise small holes in the removed blobs will be filled in, leaving mini-blobs!
      std::list<blob::BlobCompressed> fullBlobList;
      BlobIndexThreaded::blob_iterator blobIter, holeIter;
      // Loop through blobs
      for (blobIter=smallBlobIndex.begin(); blobIter!=smallBlobIndex.end();
           ++blobIter) {
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
                                   applyErodeView(inpaint(inputview.impl(),
                                                          smallHoleIndex,
                                                          use_grassfire,
                                                          default_inpaint_val),
                                                  fullBlobList),
                                   opt, TerminalProgressCallback
                                   ("asp","\t--> Filtering: ") );
    }

  } else { // No hole filling
    if (!removeSmallBlobs) { // Skip small blob removal
      asp::block_write_gdal_image( opt.out_prefix + "-F.tif",
                                       inputview.impl(), opt,
                                       TerminalProgressCallback("asp", "\t--> Filtering: ") );
    }
    else { // Add small blob removal step
      // Get a list of small blobs, almost identical to how the fill holes
      vw_out() << "\t--> Removing small blobs.\n";
      BlobIndexThreaded smallBlobIndex( inputview.impl(), stereo_settings().erode_max_size );
      vw_out() << "\t    * Identified " << smallBlobIndex.num_blobs() << " small blobs\n";

      // Write out the image to disk, removing the blobs in the process
      asp::block_write_gdal_image( opt.out_prefix + "-F.tif",
                                   applyErodeView(inputview.impl(), smallBlobIndex),
                                   opt, TerminalProgressCallback("asp","\t--> Filtering: ") );
    }

  } // End no hole filling case
}

void stereo_filtering( Options& opt ) {

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

    if ( stereo_settings().mask_flatfield ) {
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
      else { // No cleanup passes
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
      if ( stereo_settings().rm_cleanup_passes >= 1 ) {
        // Apply an outlier removal filter
        write_good_pixel_and_filtered
          (stereo::disparity_mask
            (MultipleDisparityCleanUp<input_type>()
              (disparity_disk_image, stereo_settings().rm_cleanup_passes),
               apply_mask(asp::threaded_edge_mask(left_mask, 0,mask_buffer,1024)),
               apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))),
             opt);
      }
      else { // No cleanup passes
        write_good_pixel_and_filtered
          (stereo::disparity_mask
            (disparity_disk_image,
              apply_mask(asp::threaded_edge_mask(left_mask, 0,mask_buffer,1024)),
              apply_mask(asp::threaded_edge_mask(right_mask,0,mask_buffer,1024))),
            opt);
      } // End cleanup passes check
    } // End mask_flatfield check

  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at filtering stage -- could not read input files.\n"
              << e.what() << "\nExiting.\n\n" );
  }
} // end write_good_pixel_and_filtered

int main(int argc, char* argv[]) {

  vw_out() << "\n[ " << current_posix_time_string() << " ] : Stage 3 --> FILTERING \n";

  // This is probably the right place in which to warn the user about
  // new hole filling behavior.
  vw_out(WarningMessage)
    << "Hole-filling is disabled by default in stereo_fltr. "
    << "It is suggested to use instead point2dem's analogous "
    << "functionality. It can be re-enabled using "
    << "--enable-fill-holes." << std::endl;
  
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
