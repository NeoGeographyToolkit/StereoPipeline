#include <boost/version.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

class StereoSettings {
  po::options_description m_desc;
  po::variables_map m_vm;

public:
  StereoSettings();
  void StereoSettings::read(std::string const& filename);

  // ----------------
  // Public variables
  // ----------------

  // Preprocessing options
  int epipolar_alignment;     /* Align images using the epipolar constraints */
  int keypoint_alignment;     /* Align images using the keypoint alignment method */
  int keypoint_align_subsampling;	   // if > 1, image sub-sampling factor for keypoint aligment (to speed things up)

  // Correlation Options
  int slog;		/* perform an slog (relpace the emboss) */
  int log;		/* perform a log (laplacian of the gaussian blur) */
  float slogW;
  int h_kern;			/* kernel width first pass */
  int v_kern;			/* kernel height first pass*/
  int h_corr_max;		/* correlation window max x */
  int h_corr_min;		/* correlation window min x */
  int v_corr_max;		/* correlation window max y */
  int v_corr_min;		/* correlation window min y */
  int do_h_subpixel;
  int do_v_subpixel;
  float xcorr_treshold;
  float corrscore_rejection_treshold;

  // Filtering Options
  int rm_h_half_kern;		/* low confidence pixel removal kernel size */
  int rm_v_half_kern;
  int rm_min_matches;		/* min # of pxl to be matched to keep pxl */
  int rm_treshold;		/* rm_treshold < disp[n]-disp[m] reject pxl */ 
  int rm_cleanup_passes;  /* number of times to perform cleanup in the post-processing phase */
  int fill_holes_NURBS;  
  
  // Triangulation Options
  float near_universe_radius;  	/* radius of the universe in meters */
  float far_universe_radius;  	/* radius of the universe in meters */
};

/// Return the singleton instance of the stereo setting structure.
/// The stereo settings struct is created the first time this method
/// is invoked.  You should *always* access the stereo settings
/// through this function.
StereoSettings& stereo_settings();
