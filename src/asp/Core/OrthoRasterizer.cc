// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

/// \file OrthoRasterizer.cc
///
/// Given a point image and corresponding texture, this class
/// resamples the point cloud on a regular grid over the [x,y] plane
/// of the point image; producing an evenly sampled ortho-image with
/// interpolated z values.

#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/Algorithms.h>
#include <vw/Image/BlockRasterize.h>
#include <vw/Core/ThreadPool.h>
#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Statistics.h>
#include <vw/Image/Filter.h>
#include <vw/Image/InpaintView.h>

#include <asp/Core/PointUtils.h>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/next.hpp>
#include <asp/Core/OrthoRasterizer.h>
#include <valarray>

namespace asp{

  using namespace vw;

  class compare_bboxes { // simple comparison function
  public:
    bool operator()(const BBox2i A, const BBox2i B) const {
      return (A.min().x() < B.min().x());
    }
  };

  void dump_image(std::string const& prefix, BBox2i const& box,
                  ImageViewRef<Vector3> const& I) {

    //Crop the image to the given box and save to a file.

    std::ostringstream os;
    os << prefix << "_" << box.min().x() << "_" << box.min().y()
       << " " << box.width() << " " << box.height() << ".csv";
    std::string file = os.str();
    vw_out() << "Writing: " << file << std::endl;
    std::ofstream of(file.c_str());
    of.precision(18);

    BBox2i lbox = box;
    lbox.crop(bounding_box(I));
    ImageView<Vector3> crop_img = crop(I, lbox);
    for (int col = 0; col < crop_img.cols(); col++) {
      for (int row = 0; row < crop_img.rows(); row++) {
        Vector3 p = crop_img(col, row);
        if (boost::math::isnan(p.z())) continue;
        of << p.x() << ' ' << p.y()  << ' ' << p.z() << std::endl;
      }
    }
    of.close();
  }

  // Task to parallelize the generation of bounding boxes for each block.
  class SubBlockBoundaryTask: public Task, private boost::noncopyable {
    ImageViewRef<Vector3> m_view;
    int    m_sub_block_size;
    BBox2i m_image_bbox;
    BBox3& m_global_bbox;
    std::vector<BBoxPair>& m_point_image_boundaries;
    ImageViewRef<double> const& m_error_image;
    double m_estim_max_error;   // used for outlier removal by tri error based on percentage
    vw::BBox3 m_estim_proj_box; // used for outlier removal in the bounding box computation
    std::vector<double> & m_errors_hist;
    double m_max_valid_triangulation_error; // used for outlier removal based on user thresh
    Mutex& m_mutex;
    const ProgressCallback& m_progress;
    float m_inc_amt;

    // This is growing a bbox of points in point projection and Z
    // values which are altitude.
    struct GrowBBoxAccumulator {
      BBox3 bbox;
      void operator()(Vector3 const& v) {
        if (!boost::math::isnan(v.z()))
          bbox.grow(v);
      }
    };

    struct ErrorHistAccumulator{
      std::vector<double> & m_hist;
      double m_max_val;
      ErrorHistAccumulator(std::vector<double>& hist, double max_val):
        m_hist(hist), m_max_val(max_val) {}
      void operator()(double err) {
        if (err == 0) return; // null errors come from invalid pixels
        int len = m_hist.size();
        int k = round((len-1)*std::min(err, m_max_val)/m_max_val);
        if (k >= 0 && k < len && m_max_val > 0) {
          // This is a bugfix for an observed situation when err is NaN.
          // In that case the rounding returns a negative integer.
          m_hist[k]++;
        }
      }
    };

  public:
    SubBlockBoundaryTask(ImageViewRef<Vector3> const& view,
                         int sub_block_size,
                         BBox2i const& image_bbox,
                         BBox3       & global_bbox,
                         std::vector<BBoxPair>& boundaries,
                         ImageViewRef<double> const& error_image,
                         double estim_max_error,
                         vw::BBox3 const& estim_proj_box,
                         std::vector<double> & errors_hist,
                         double max_valid_triangulation_error,
                         Mutex& mutex, const ProgressCallback& progress, float inc_amt) :
      m_view(view.impl()), m_sub_block_size(sub_block_size),
      m_image_bbox(image_bbox),
      m_global_bbox(global_bbox), m_point_image_boundaries(boundaries),
      m_error_image(error_image), m_estim_max_error(estim_max_error),
      m_estim_proj_box(estim_proj_box),
      m_errors_hist(errors_hist), m_max_valid_triangulation_error(max_valid_triangulation_error),
      m_mutex(mutex), m_progress(progress), m_inc_amt(inc_amt) {}

    void operator()() {
      ImageView<Vector3> local_image = crop(m_view, m_image_bbox);

      bool remove_outliers_with_pct = (!m_errors_hist.empty());
      ImageView<double> local_error;
      if (remove_outliers_with_pct || m_max_valid_triangulation_error > 0.0)
        local_error = crop(m_error_image, m_image_bbox);

      // Further subdivide into boundaries so that prerasterize will
      // only query what it needs.
      std::vector<BBox2i> blocks = subdivide_bbox(m_image_bbox, m_sub_block_size,
                                                  m_sub_block_size);
      BBox3 local_union;
      std::list<BBoxPair> solutions;
      std::vector<double> local_hist(m_errors_hist.size(), 0);
      bool nonempty_estim_proj_box = (!m_estim_proj_box.empty());

      for (size_t i = 0; i < blocks.size(); i++) {
        BBox3 pts_bdbox;
        ImageView<Vector3> local_image2 = crop(local_image, blocks[i] - m_image_bbox.min());

        // See if to filter by user-provided m_max_valid_triangulation_error.
        // Otherwise try to make use of m_estim_max_error if it was found.
        // If this is not provided we will later estimate and use such a value automatically
        // when doing the gridding.
        ImageView<double> local_error2;
        if (m_max_valid_triangulation_error > 0 || m_estim_max_error > 0)
          local_error2 = crop(local_error, blocks[i] - m_image_bbox.min());

        for (int col = 0; col < local_image2.cols(); col++) {
          for (int row = 0; row < local_image2.rows(); row++) {

            // Skip invalid points
            if (boost::math::isnan(local_image2(col, row).z()))
              continue;

            // Skip outliers, points not in the estimated bounding box
            if (nonempty_estim_proj_box &&
                !m_estim_proj_box.contains(local_image2(col, row))) {
              continue;
            }

            if (m_max_valid_triangulation_error > 0) {
              if (local_error2(col, row) > m_max_valid_triangulation_error)
                continue;
            } else if (m_estim_max_error > 0) {
              if (local_error2(col, row) > m_estim_max_error)
                continue;
            }
            pts_bdbox.grow(local_image2(col, row));
          }
        }

        if (pts_bdbox.min().x() <= pts_bdbox.max().x() &&
            pts_bdbox.min().y() <= pts_bdbox.max().y()) {

          // If the length is 0 along some dimension, grow it by a small amount
          // as otherwise this data point will not be included in the bounding box.
          for (int i = 0; i < 3; i++) {
            if (std::isinf(pts_bdbox.min()[i]) || std::isinf(pts_bdbox.max()[i]))
              continue;
            if (pts_bdbox.min()[i] == pts_bdbox.max()[i])
              pts_bdbox.max()[i] = boost::math::float_next(pts_bdbox.max()[i]);
          }

          local_union.grow(pts_bdbox);
          solutions.push_back(std::make_pair(pts_bdbox, blocks[i]));
        }

        if (remove_outliers_with_pct) {
          ErrorHistAccumulator error_accum(local_hist, m_estim_max_error);
          for_each_pixel(crop(local_error, blocks[i] - m_image_bbox.min()),
                         error_accum);
        }
      }

      // Append to the global list of boxes and expand the point cloud
      // bounding box.
      if (local_union != BBox3()) {
        Mutex::Lock lock(m_mutex);
        for (std::list<BBoxPair>::const_iterator it = solutions.begin();
              it != solutions.end(); it++) {
          m_point_image_boundaries.push_back(*it);
        }

        m_global_bbox.grow(local_union);

        if (remove_outliers_with_pct)
          for (int i = 0; i < (int)m_errors_hist.size(); i++)
            m_errors_hist[i] += local_hist[i];

        m_progress.report_incremental_progress(m_inc_amt);
      }
    }
  }; // End function operator()

  void remove_outliers(ImageView<Vector3> & image, ImageViewRef<double> const& errors,
                       double error_cutoff, BBox2i const& box) {

    // Mask as NaN points above triangulation error
    if (error_cutoff < 0)
      return; // nothing to do

    double nan = std::numeric_limits<double>::quiet_NaN();
    ImageView<float> error_copy = crop(errors, box);

    VW_ASSERT(image.cols() == error_copy.cols() &&
              image.rows() == error_copy.rows(),
              ArgumentErr() << "Size mis-match in remove_outliers().");

    for (int col = 0; col < image.cols(); col++) {
      for (int row = 0; row < image.rows(); row++) {
        if (error_copy(col, row) > error_cutoff) {
          image(col, row).z() = nan;
        }
      }
    }

  }

  void filter_by_median(ImageView<Vector3> & image, Vector2 const& median_filter_params) {

    // If the point cloud height at the current point differs by more
    // than the given threshold from the median of heights in the
    // window of given size centered at the point, remove it as an outlier.

    int    half   = median_filter_params[0]/2; // half window size
    double thresh = median_filter_params[1];
    if (half <= 0 || thresh <= 0)
      return;

    int nc = image.cols(), nr = image.rows(); // shorten
    double nan = std::numeric_limits<double>::quiet_NaN();

    ImageView<Vector3> image_out = copy(image);

    for (int col = 0; col < image.cols(); col++) {
      for (int row = 0; row < image.rows(); row++) {

        if (boost::math::isnan(image(col, row).z()))
          continue;

        std::vector<double> vals;
        for (int c = std::max(col-half, 0); c <= std::min(col+half, nc-1); c++) {
          for (int r = std::max(row-half, 0); r <= std::min(row+half, nr-1); r++) {
            if (boost::math::isnan(image(c, r).z()))
              continue;
            vals.push_back(image(c, r).z());
          }
        }
        double median = vw::math::destructive_median(vals);
        if (fabs(median - image(col, row).z()) > thresh) {
          image_out(col, row).z() = nan;
        }
      }
    }

    image = copy(image_out);
  }

  // TODO: This function should live somewhere else!
  // Erode this many pixels around invalid pixels
  void erode_image(ImageView<Vector3> & image, int erode_len) {

    if (erode_len <= 0) // No erode, we are finished!
      return;

    int    nc      = image.cols(),
           nr      = image.rows(); // shorten
    double nan     = std::numeric_limits<double>::quiet_NaN();
    int    max_col = nc - 1;
    int    max_row = nr - 1;

    // Create a buffer to modify and a pointer to allow easy
    //  swapping of which image we are writing to.
    // - The first write is to the input image, so we don't have to do extra copies with one pass.
    ImageView<Vector3>  buffer = copy(image);
    ImageView<Vector3>* read_ptr  = &buffer;
    ImageView<Vector3>* write_ptr = &image;
    ImageView<Vector3>* temp_ptr = 0;

    // One pass per erode length
    for (int pass = 0; pass < erode_len; pass++) {

      // Loop through the entire image
      for (int col = 0; col < nc; col++) {
        int start_col = std::max(col-1, 0);
        int stop_col  = std::min(col+1, max_col);
        for (int row = 0; row < nr; row++) {
          int start_row = std::max(row-1, 0);
          int stop_row  = std::min(row+1, max_row);

          // Loop through bounds checked border 1 region around this pixel
          for (int c = start_col; c <= stop_col; c++) {
            for (int r = start_row; r <= stop_row; r++) {
              // If any of these pixels are bad, throw out this pixel
              if (boost::math::isnan(read_ptr->operator()(c, r).z()))
                write_ptr->operator()(col, row).z() = nan;
            }
          } // End inner erode double loop

        }
      } // End double loop through image pixels

      // Swap the read and write buffer pointers for the next pass
      // - No need to make copies each time, the nan regions will just keep expanding.
      temp_ptr  = write_ptr;
      write_ptr = read_ptr;
      read_ptr  = temp_ptr;

    } // end passes

    // If we had an even number of erode passes, the last write was to the temp
    //  buffer and we need to copy it back to the input image.
    if ((erode_len % 2) == 0)
      image = copy(buffer);
  }

  // Given a histogram as a vector of counts, based on binning values
  // in the interval [0, max_val] with n bins, find a given percentile
  // error. Here, pct is in [0, 100].
  double percentile_error(std::vector<double> const& hist, double max_val, double pct) {

    double pct_ratio = pct / 100.0;

    // Total number of counts in the histogram
    int hist_size = hist.size();
    vw::int64 num_errors = 0;
    for (int s = 0; s < hist_size; s++)
      num_errors += hist[s];

    // The index so that the number of counts up to index is the fraction pct/100.0
    // of the total number of counts.

    int cutoff_index = 0;
    vw::int64 sum = 0; // to protect against overflow when adding many numbers
    for (int s = 0; s < hist_size; s++) {
      sum += hist[s];
      if (sum >= pct_ratio * num_errors) {
        cutoff_index = s;
        break;
      }
    }

    return max_val * cutoff_index / double(hist_size);
  }

  OrthoRasterizerView::OrthoRasterizerView
  (ImageViewRef<Vector3> point_image, ImageViewRef<double> texture,
   double search_radius_factor, double sigma_factor, int pc_tile_size,
   vw::BBox2 const& projwin,
   OutlierRemovalMethod outlier_removal_method,
   Vector2 const& remove_outliers_params,
   ImageViewRef<double> const& error_image,
   double estim_max_error, vw::BBox3 const& estim_proj_box,
   double max_valid_triangulation_error,
   Vector2 median_filter_params, int erode_len, bool has_las_or_csv,
   std::string const& filter,
   double default_grid_size_multiplier,
   std::int64_t * num_invalid_pixels, vw::Mutex *count_mutex,
   const ProgressCallback& progress):
    // Ensure all members are initiated, even if to temporary values
    m_point_image(point_image), m_texture(ImageView<float>(1,1)),
    m_bbox(BBox3()), m_snapped_bbox(BBox3()), m_spacing(0.0), m_default_spacing(0.0),
    m_default_spacing_x(0.0), m_default_spacing_y(0.0),
    m_search_radius_factor(search_radius_factor),
    m_sigma_factor(sigma_factor),
    m_default_value(0),
    m_minz_as_default(true), m_use_alpha(false),
    m_block_size(pc_tile_size),
    m_projwin(projwin),
    m_error_image(error_image), m_error_cutoff(-1.0),
    m_median_filter_params(median_filter_params), m_erode_len(erode_len),
    m_default_grid_size_multiplier(default_grid_size_multiplier),
    m_num_invalid_pixels(num_invalid_pixels),
    m_count_mutex(count_mutex) {

    *m_num_invalid_pixels = 0; // Init counter
    set_texture(texture.impl());

    // Convert the filter from string to enum, to speed up checking against it later
    m_percentile = -1; // ensure it is initialized
    if (filter      == "weighted_average") m_filter = asp::f_weighted_average;
    else if (filter == "min") m_filter = asp::f_min;
    else if (filter == "max") m_filter = asp::f_max;
    else if (filter == "mean") m_filter = asp::f_mean;
    else if (filter == "median") m_filter = asp::f_median;
    else if (filter == "stddev") m_filter = asp::f_stddev;
    else if (filter == "count") m_filter = asp::f_count;
    else if (filter == "nmad") m_filter = asp::f_nmad;
    else if (sscanf (filter.c_str(), "%lf-pct", &m_percentile) == 1)
      m_filter = asp::f_percentile;
    else
    vw_throw(ArgumentErr() << "OrthoRasterize: unknown filter: " << filter << ".\n");

    //dump_image("img", BBox2(0, 0, 3000, 3000), point_image);

    // Compute the bounding box that encompasses tiles within the image
    //
    // They're used for querying what part of the image we need
    VW_OUT(DebugMessage,"asp") << "Computing raster bounding box.\n";

    int num_bins = 1024;
    std::vector<double> errors_hist;
    if (outlier_removal_method != NO_OUTLIER_REMOVAL_METHOD) {
      // Need to compute the histogram of all errors in the error image
      errors_hist = std::vector<double>(num_bins, 0.0);
    }

    // Subdivide each block into smaller chunks. Note: small chunks
    // greatly increase the memory usage and run-time for very large
    // images (because they are very many). As such, make the chunks
    // bigger for bigger images.
    // Creating each task also has overhead from copying the
    // georeferences.
    // TODO(oalexan1): Check how big these are. Maybe
    // they should be as many tasks as there are threads or 2x that.
    double s = 10000.0;
    int sub_block_size
      = int(double(point_image.cols())*double(point_image.rows())/(s*s));
    sub_block_size = std::max(1, sub_block_size);
    sub_block_size = int(round(pow(2.0, floor(log(sub_block_size)/log(2.0)))));
    sub_block_size = std::max(16, sub_block_size);
    sub_block_size = std::min(ASP_MAX_SUBBLOCK_SIZE, sub_block_size);
    std::vector<BBox2i> blocks = subdivide_bbox(m_point_image, m_block_size, m_block_size);

    // Find the bounding box of each subblock, stored in
    // m_point_image_boundaries, together with other info by
    // searching through the image.
    FifoWorkQueue queue(vw_settings().default_num_threads());
    typedef SubBlockBoundaryTask task_type;
    Mutex mutex;
    float inc_amt = 1.0 / float(blocks.size());
    for (size_t i = 0; i < blocks.size(); i++) {
      boost::shared_ptr<task_type>
        task(new task_type(m_point_image, sub_block_size, blocks[i],
                           m_bbox, m_point_image_boundaries,
                           error_image, estim_max_error, estim_proj_box, errors_hist,
                           max_valid_triangulation_error,
                           mutex, progress, inc_amt));
      queue.add_task(task);
    }
    queue.join_all();
    progress.report_finished();

    if (m_bbox.empty())
      vw_throw(ArgumentErr() << "OrthoRasterize: Input point cloud is empty!\n");

    // Override with user's projwin, if specified
    if (m_projwin != BBox2()) {
      subvector(m_bbox.min(), 0, 2) = m_projwin.min();
      subvector(m_bbox.max(), 0, 2) = m_projwin.max();
    }

    VW_OUT(DebugMessage,"asp") << "Point cloud boundary is " << m_bbox << "\n";

    if (outlier_removal_method != NO_OUTLIER_REMOVAL_METHOD) {

      // Per user request, find some error percentiles to print.
      std::map<double, double> percentiles = {{25.0, 0.0}, {50.0, 0.0}, {75.0, 0.0},
                                              // The user-percentile
                                              {remove_outliers_params[0], 0.0}};

      for (auto it = percentiles.begin(); it != percentiles.end(); it++)
        percentiles[it->first] = percentile_error(errors_hist, estim_max_error,
                                                  it->first);

      vw::int64 num_samples = 0; // to protect against overflow when adding many numbers
      for (int s = 0; s < errors_hist.size(); s++)
        num_samples += errors_hist[s];

      // Find the outlier cutoff from the histogram of all errors.
      // The cutoff is the outlier factor times the percentile of the errors.
      double user_percentile = percentiles[remove_outliers_params[0]];

      vw_out() << "Collected a sample of " << num_samples << " positive triangulation errors.\n";
      vw_out() << "Error percentiles: "
               << "Q1 (25%): " << percentiles[25.0] << ", "
               << "Q2 (50%): " << percentiles[50.0] << ", "
               << "Q3 (75%): " << percentiles[75.0] << "."
               << std::endl;

      // Multiply by the outlier factor
      if (outlier_removal_method == PERCENTILE_OUTLIER_METHOD) {
        double factor = remove_outliers_params[1];       // e.g., 3.0
        m_error_cutoff = factor * user_percentile;
        vw_out() << "Computing triangulation error cutoff based on --remove-outliers-params.\n";
      } else if (outlier_removal_method == TUKEY_OUTLIER_METHOD) {
        vw_out() << "Using as outlier cutoff the Tukey formula Q3 + 1.5*(Q3 - Q1)." << std::endl;
        m_error_cutoff = percentiles[75.0] + 1.5*(percentiles[75.0] - percentiles[25.0]);
      } else {
        vw_throw(ArgumentErr() << "Unexpected choice for outlier removal method.\n");
      }

      vw_out() << "Triangulation error cutoff is " << m_error_cutoff
               << " meters.\n";

    } else if (max_valid_triangulation_error > 0.0) {
      m_error_cutoff = max_valid_triangulation_error;
      vw_out() << "Manual triangulation error cutoff is " << m_error_cutoff
               << " meters.\n";
    }

    // Find the width and height of the median point cloud pixel in
    // projected coordinates. For las or csv files, this approach
    // does not work.
    int len = m_point_image_boundaries.size();
    if (!has_las_or_csv) {
      // This vectors can be large, so don't keep them for too long
      std::vector<double> vx, vy;
      vx.reserve(len); vx.clear();
      vy.reserve(len); vy.clear();
      BOOST_FOREACH(BBoxPair const& boundary, m_point_image_boundaries) {
        if (boundary.first.empty())
          continue;
        vx.push_back(boundary.first.width() /sub_block_size);
        vy.push_back(boundary.first.height()/sub_block_size);
      }
      std::sort(vx.begin(), vx.end());
      std::sort(vy.begin(), vy.end());

      if (len > 0) {
        // Get the median
        // TODO(oalexan1): This is not robust. For lro nac, vertical resolution
        // and horizontal resolution differ by a factor of 4, e.g.,
        // 0.5 m and 2 m. The median can be one of the two, which is
        // wrong.  This code should be an average of the values in the
        // [25%, 75%] range.
        // TODO(oalexan1): Integrate with the logic for mapproject.
        // https://github.com/NeoGeographyToolkit/StereoPipeline/issues/173
        m_default_spacing_x = vx[(int)(0.5*len)];
        m_default_spacing_y = vy[(int)(0.5*len)];
      }
    }

    return;
  } // End OrthoRasterizerView Constructor

  // This is kind of like part 2 of the constructor
  // - This function finalizes the spacing and generates a spacing-snapped BBox.
  void OrthoRasterizerView::initialize_spacing(const double spacing) {

    // This must happen after the bounding box was computed, but before setting
    // the spacing. We choose the coarsest of the two spacings
    m_default_spacing = std::max(m_default_spacing_x, m_default_spacing_y);
    
    if (spacing > 0 && m_default_spacing > 0 && spacing < m_default_spacing) {
      if (spacing < 0.001 * m_default_spacing)
        vw::vw_throw(vw::ArgumentErr() 
          << "The user-provided grid size (--tr) is so small that likely it is in degrees, "
          << "while meters are expected.\n");
        // For a lesser discrepancy, just print a warning.
        vw_out(vw::WarningMessage) 
          << "The user-provided grid size (--tr) is " << spacing << ", "
          << "which is smaller than the auto-estimated grid size of " 
          << m_default_spacing << ". Likely the DEM will not be accurate.\n";
    }
        
    // Set the sampling rate (i.e. spacing between pixels)
    this->set_spacing(spacing);
    VW_OUT(DebugMessage,"asp") << "Pixel spacing is " << m_spacing << " pnt/px\n";

    // We will snap the box so that its corners are integer multiples
    // of the grid size. This ensures that any two DEMs
    // with the same grid size and overlapping grids have those
    // grids match perfectly.
    m_snapped_bbox = m_bbox;

    // If the user wants to use m_search_radius_factor to do filling,
    // expand the box to allow the DEM to grow.
    if (m_search_radius_factor > 0)
      m_snapped_bbox.expand(spacing*m_search_radius_factor);

    snap_bbox(m_spacing, m_snapped_bbox);

    // Override with user's projwin, if specified
    if (m_projwin != BBox2()) {
      subvector(m_snapped_bbox.min(), 0, 2) = m_projwin.min();
      subvector(m_snapped_bbox.max(), 0, 2) = m_projwin.max();

      // The proj win takes into account that each pixel's physical size is
      // m_spacing. So it is biased by half a pixel outwards from the snapped
      // box. Compensate for that here.
      m_snapped_bbox.min() += Vector3(m_spacing/2.0, m_spacing/2.0, 0);
      m_snapped_bbox.max() -= Vector3(m_spacing/2.0, m_spacing/2.0, 0);
      snap_bbox(m_spacing, m_snapped_bbox);
    }

  } // End function initialize_spacing()

  // Function to convert pixel coordinates to the point domain
  BBox3 OrthoRasterizerView::pixel_to_point_bbox(BBox2 const& inbox) const {
    BBox3 outbox = m_snapped_bbox;
    outbox.min().x() = m_snapped_bbox.min().x() + ((double(inbox.min().x()))
                                                   * m_spacing);
    outbox.max().x() = m_snapped_bbox.min().x() + ((double(inbox.max().x()))
                                                   * m_spacing);
    outbox.min().y() = m_snapped_bbox.min().y() + ((double(rows() - inbox.max().y()))
                                                   * m_spacing);
    outbox.max().y() = m_snapped_bbox.min().y() + ((double(rows() - inbox.min().y()))
                                                   * m_spacing);
    return outbox;
  }

  /// \cond INTERNAL
  OrthoRasterizerView::prerasterize_type
  OrthoRasterizerView::prerasterize(BBox2i const& bbox) const {

    BBox2i bbox_1 = bbox;

    // bugfix, ensure we see enough beyond current tile
    bbox_1.expand((int)ceil(std::max(m_search_radius_factor, 5.0)));

    // Used to find which polygons are actually in the draw space.
    BBox3 local_3d_bbox = pixel_to_point_bbox(bbox_1);

    ImageView<double> d_buffer, weights;

    // Given a DEM grid point, search for cloud points within the
    // circular region of radius equal to grid size. As such, a
    // given cloud point may contribute to multiple DEM points, but
    // with different weights (set by Gaussian). We make this radius
    // no smaller than the default DEM spacing. Search radius can be
    // over-ridden by user.
    double search_radius;
    if (m_search_radius_factor <= 0.0)
      search_radius = std::max(m_spacing, m_default_spacing);
    else
      search_radius = m_spacing*m_search_radius_factor;
    asp::Point2Grid point2grid(bbox_1.width(),
                               bbox_1.height(),
                               d_buffer, weights,
                               local_3d_bbox.min().x(),
                               local_3d_bbox.min().y(),
                               m_spacing, m_default_spacing,
                               search_radius, m_sigma_factor,
                               m_filter, m_percentile);

    // Set up the default color value
    double min_val = 0.0;
    if (m_use_alpha) {
      // use this dummy value to denote transparency
      min_val = std::numeric_limits<float>::min();
    } else if (m_minz_as_default) {
      min_val = m_snapped_bbox.min().z();
    } else {
      min_val = m_default_value;
    }

    std::valarray<float> vertices(10), intensities(5);
    point2grid.Clear(min_val);

    // For each block in the DEM space intersecting local_3d_bbox,
    // find the corresponding blocks in the point cloud space.  We
    // use here a map since we'd like to group together the point
    // cloud blocks which fall within the same 256 x 256 tile, to do
    // their union instead of them individually, for reasons of speed.
    typedef std::map<BBox2i, BBox2i, compare_bboxes> BlockMapType;
    typedef BlockMapType::iterator MapIterType;
    BlockMapType blocks_map;
    BOOST_FOREACH(BBoxPair const& boundary, m_point_image_boundaries) {
      if (! local_3d_bbox.intersects(boundary.first))
        continue;

      BBox2i pc_block = boundary.second;

      BBox2i snapped_block;
      snapped_block.min() = m_block_size*floor(pc_block.min()/double(m_block_size));
      snapped_block.max() = m_block_size*ceil(pc_block.max()/double(m_block_size));
      MapIterType it = blocks_map.find(snapped_block);
      if (it != blocks_map.end()) {
        (it->second).grow(pc_block);
      } else {
        blocks_map.insert(std::pair<BBox2, BBox2>(snapped_block, pc_block));
      }

    }

    if (blocks_map.empty()) {
      // TODO: Don't include these pixels in the total?
      { // Lock and update the total number of invalid pixels in this tile.
        vw::Mutex::Lock lock(*m_count_mutex);
        // Care here, convert to int64_t before multiplication, to avoid
        // int32 overflow.
        (*m_num_invalid_pixels) += std::int64_t(bbox.width())*std::int64_t(bbox.height());
      }

      return prerasterize_type(d_buffer, BBox2i(-bbox_1.min().x(),
                                                -bbox_1.min().y(), cols(), rows()));
    }

    for (MapIterType it = blocks_map.begin(); it != blocks_map.end(); it++) {

      BBox2i block = it->second;

      block.crop(vw::bounding_box(m_point_image));

      // Pull a copy of the input image in memory.  Expand the image
      // to be able to see a bit beyond when filling holes.
      BBox2i biased_block = block;
      int bias = m_median_filter_params[0]/2 + m_erode_len;
      biased_block.expand(bias);
      biased_block.crop(vw::bounding_box(m_point_image));
      ImageView<Vector3> point_copy = crop(m_point_image, biased_block);

      remove_outliers(point_copy, m_error_image, m_error_cutoff, biased_block);
      filter_by_median(point_copy, m_median_filter_params);
      erode_image(point_copy, m_erode_len);

      // Crop back to the area of interest
      point_copy = crop(point_copy, block - biased_block.min());

      ImageView<float> texture_copy = crop(m_texture, block);

      typedef ImageView<Vector3>::pixel_accessor PointAcc;
      PointAcc row_acc = point_copy.origin();
      for (int32 row = 0; row < point_copy.rows(); row++) {
        PointAcc point_ul = row_acc;

        for (int32 col = 0; col < point_copy.cols(); col++) {

          PointAcc point_ur = point_ul; point_ur.next_col();
          PointAcc point_ll = point_ul; point_ll.next_row();
          PointAcc point_lr = point_ul; point_lr.advance(1,1);

          if (!boost::math::isnan(point_copy(col, row).z()) &&
                local_3d_bbox.contains(point_copy(col, row))) {
            point2grid.AddPoint(point_copy(col, row).x(),
                                point_copy(col, row).y(),
                                texture_copy(col,  row));
          }
          point_ul.next_col();
        } // End column loop
        row_acc.next_row();
      } // End row loop

    }

    point2grid.normalize();

    // The software renderer returns an image which will render
    // upside down in most image formats, so we correct that here.
    // We also introduce transparent pixels into the result where necessary.
    // TODO: Here can do flipping in place.
    ImageView<PixelGray<float>> result;
    result = flip_vertical(d_buffer);

    // Loop through result here and count up how many pixels have been
    // changed from the default value.
    std::int64_t num_unset = 0;
    for (int r = 0; r < result.rows(); r++) {
      for (int c = 0; c < result.cols(); c++) {

        Vector2i pix = Vector2(c, r) + bbox_1.min();
        if (bbox.contains(pix)) {
          //  Ignore the pixels in the temporary extension of bbox.
          if (result(c,r) == min_val)
            ++num_unset;
        }
      }
    }
    { // Lock and update the total number of invalid pixels in this tile.
      vw::Mutex::Lock lock(*m_count_mutex);
      (*m_num_invalid_pixels) += num_unset;
    }

    return prerasterize_type(result,
                             BBox2i(-bbox_1.min().x(), -bbox_1.min().y(), cols(), rows()));
  }

  // Return the affine georeferencing transform.
  vw::Matrix<double,3,3> OrthoRasterizerView::geo_transform() {
    vw::Matrix<double,3,3> geo_transform;
    geo_transform.set_identity();
    geo_transform(0,0) = m_spacing;
    geo_transform(1,1) = -m_spacing;
    geo_transform(0,2) = m_snapped_bbox.min().x();
    geo_transform(1,2) = m_snapped_bbox.max().y();
    return geo_transform;
  }

} // namespace asp
