#ifndef __STEREO_H__
#define __STEREO_H__

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <stddef.h>
#include <math.h>
#include <string>

#include <vw/Core/Exception.h>

typedef double pixel;

#define ASP_VERSION_STRING "2.0a"

// Common exceptions
VW_DEFINE_EXCEPTION(EphemerisErr, vw::Exception);

/// Erases a file suffix if one exists and returns the base string
static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}

/// Erases a file suffix if one exists and returns the base string
static std::string suffix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(0, index);
  return result;
}

#ifndef BUFFER_SIZE
#define	BUFFER_SIZE	256
#endif /* BUFFER_SIZE */

/* 2D homogenous matrix and associated vector

   [ a  b  Tx ]		[ x ]
 M=[ c  d  Ty ]	     v=	[ y ]
   [ 0  0  1  ]		[ 1 ]	*/

typedef struct {
  float h11;
  float h12;
  float h13;
  float h21;
  float h22; 
  float h23; 
  float h31;
  float h32; 
  float h33; 
}hMatrix2D;

typedef struct {
  int do_alignment;           /* Do we do alignment at all? */
  int keypoint_alignment;     /* Align images using the keypoint alignment method */
  int ephemeris_alignment;    /* Align images using the ephemeris alignment method */
  int epipolar_alignment;     /* Align images using the epipolar constraints */
  int format_size;		/* format the size of the image */
  int slog;		/* perform an slog (relpace the emboss) */
  int log;		/* perform a log (laplacian of the gaussian blur) */
  int eq_hist1;		/* do the first histogam equalisation */
  int emboss;			/* do the emboss convolution */
  int eq_hist2;		/* do the second histogam equalisation */
  int apply_mask;		/* mask img region likely to poorly corrlate */
  int w_mask;			/* write 2 buffers file with texture - mask */
  int autoSetCorrParam;	/*uses pyramidal scheme to autom get search range */  
  int vert_cal;		/* do the vertical calibration */
  int w_texture;	       	/* write the pgm texture File */
  int w_preprocessed;		/* write the preprocessed image file */
  int w_extrapolation_mask;
  int corr_1st_pass;      	/* do the correlation */
  int biDimCorr;	/* perform the search in 2D instead of in 1D */
  int corr_clean_up;	/* do n filtering pass to rm wrong matches */
  int w_debug_disp;     	/* write intermediate disp.pgm files */
  int w_disp_stp;		/* write a stp file of the raw disparity map */
  int w_disp_pgm;		/* write a pgm file of the raw disparity map */
  int w_disp_vicar;	/* write a vicar file(s) (x and y) of the disp map */
  int w_raw_disparity_map;	// write the raw disparity values as shorts
  int w_pgm_disparity_map;	// write the disparity without scaling as pgm
  int fill_holes_NURBS;         // fill the holes using Larry's NURB code. 
  int fill_v_holes;	/* fill holes in disp map with vertical algorithm */
  int fill_h_holes;	/* fill hole in disp map with horizontal algorithm */
  int extend_lr;		/* extrapolate disp map values (LEFT/RIGHT) */
  int extend_tb;		/* extrapolate disp map values (TOP/BTM) */
  int smooth_disp;		/* do a smooth disp on the disp file */
  int w_filtered_disp_pgm;	/* write the filtered disp map in pgm */
  int smooth_range;		/* do a smooth range on the range file */
  int dotcloud;		/* build the dotcloud model */
  int local_level_transform;	/* change dotcloud ref frame */
  int w_dotcloud;		/* write the dotcloud file */
  int w_vicar_range_maps;	/* write range maps in mvacs vicar format */
  int w_vicar_xyz_map;		/* write xyz range map in vicar format */
  int alt_texture;		/* create and write a texture f(altitude) */
  int mesh;			/* do the mesh */
  int adaptative_meshing;	/* do the adaptative meshing */
  int nff_plain;		/* save it as a plain model */
  int nff_txt;		/* save it as a textured model */
  int double_sided;		/* draw the two side of the polygons */
  int inventor;		/* save it as an Inventor file */
  int vrml;			/* save it as an VRML file */
  int write_ive;			/* save it as an VRML file */
  int write_dem;		/* save it as an ENVI DEM file */
} TO_DO;		/* Main to do file */

typedef struct {
  int h_kern;			/* kernel width first pass */
  int v_kern;			/* kernel height first pass*/
  int corr_margin;		/* extra margin for the search window */
  int h_corr_max;		/* correlation window max x */
  int h_corr_min;		/* correlation window min x */
  int v_corr_max;		/* correlation window max y */
  int v_corr_min;		/* correlation window min y */
  int crop_x_min;		/* cropping coordonate */
  int crop_x_max;
  int crop_y_min;
  int crop_y_max;
  int autoSetVCorrParam;	/* goes with autoSetCorrParam */
  float xcorr_treshold;
  float corrscore_rejection_treshold;
  /* Ephemeris alignment */
  double ephem_align_kernel_x; /* x coordinate of the ephem. alignmnt kernel */
  double ephem_align_kernel_y; /* y coordinate of the ephem. alignmnt kernel */
  int ephem_align_kernel_width;  /* Width of the ephemeris alignment kernel */
  int ephem_align_kernel_height; /* Height of the ephemeris alignment kernel */
  
  int num_processors;      // The number of processors on the host machine */

  /* Camera parameters */
  int useCAHV;
  float baseline;		/* distance between the cameras */
  float tilt_pivot_offset;	/* vert dist btwn optical axis and tilt axis */
  float camera_offset;	/* horz dist btwn cam nodal pt and tilt axis */
  float x_pivot_offset;	/* offset btw origin of world and tilt axis */
  float y_pivot_offset;
  float z_pivot_offset;
  float toe_r;		/* toe in for the right eye */
  float toe_l;		/* toe in for the left eye */
  float h_theta_Rpixel;	/* field of view per pixel */  
  float h_theta_Lpixel;
  float v_theta_Rpixel; 
  float v_theta_Lpixel;

  int use_motor_count;	/* use the motor count instead of the MIPL value */
  int out_width;		/* desired image output size */
  int out_height;
  float near_universe_radius;  	/* radius of the universe in meters */
  float far_universe_radius;  	/* radius of the universe in meters */
  float ground_plane;   /* elevation of the ground plane rel to the origin */
  int sky_billboard;		// do place everything higher than a
				// given elev. on billboard
  float sky_billboard_elevation; // Sky billboard elevation angle.
  int sky_brightness_threshold; // Anything brighter than this is
				// considered sky.
  int rm_h_half_kern;		/* low confidence pixel removal kernel size */
  int rm_v_half_kern;
  int rm_min_matches;		/* min # of pxl to be matched to keep pxl */
  int rm_treshold;		/* rm_treshold < disp[n]-disp[m] reject pxl */ 
  float smr_treshold;		/* treshold for the smooth_range function */
  int v_fill_treshold;	/* treshold for the file_hole_vert function */
  int h_fill_treshold;	/* treshold for the file_hole_vert function */
  int nff_v_step;		/* vertical step of the grid of the mesh */
  int nff_h_step;		/* horizontal step of the grid of the mesh */
  int mosaic_v_step;	/* vertical step of the grid of the mosaic mesh */
  int mosaic_h_step;	/* horizontal step of the grid of the mosaic mesh */
  float mosaic_sphere_center_x;	// x coord of mosaic sphere center
  float mosaic_sphere_center_y;	// y coord of mosaic sphere center
  float mosaic_sphere_center_z;	// z coord of mosaic sphere center
  int draw_mosaic_ground_plane;	// If 1, draw the ground plane for mosaics
  int mosaic_ignore_intensity;	// Ignore pixels of this intensity in mosaics
  int nff_max_jump;		/* max step in disp space to draw a triangle */
  int nff_2d_map;		/* draw a flat map */
  int verbose;		/* no comment */
  float pan_offset;		/* offset added to  pan/tilt readed */
  float tilt_offset;
  float altitude_range;	/* for the altitude texturing */
  float altitude_offset;	/* color coding the altitude */
  int altitude_mode;		/* 0 limited 1 periodic */
  float alt_btm_color;
  float alt_top_color;
  float texture_cntrst;		/* constrast for overlayd img texture */
  int texture_casting_type;	/* how to cast the texture in the pgm file */ 
  int max_gray_in_texture; 	/* 0-> fit between Imax and Imin (0-255) */
  int min_gray_in_texture; 	/* 1-> fit btw {max,min}_gray_in_texture */
  float x_disp_corr;		/* correct small and linear distortion in */
  float y_disp_corr;		/* disp map */
  float disp_corr_offset;
  int mosaic;			/* mosaic mode map the texture on a sphere */
  int smooth_disp_M;		/* matrix size for disp smoothing */
  int smooth_disp_N;
  int Lextend;		/* extrapolate disp map toward the Left edge */
  int Rextend;			/* right edge */
  int Textend;			/* top edge */
  int Bextend;			/* btm edge */
  int Toffset;		/* offset to give to extrapolated pixels */
  int Boffset;		/* relative to valid neibourgs */
  float lens_corr2;		/* 2nd order corr fctr for lens aberation */
  float lens_corr1;
  float lens_corr0;
  float range_scale;		/* model scale -- rel. to origin -- */ 
  float imp_az_offset; /* offset btwn zero motor count and x axis of cam */
  float imp_can_z_offset;	/* z offset btwn imp origin and el/az X axis */
  float x_imp_offset;		/* offset btwn Camera and lander frame */
  float y_imp_offset;
  float z_imp_offset;
  float local_level_x;	/* quaternion for reference frame changes */
  float local_level_y;
  float local_level_z;
  float local_level_w;
  int billboard_on;		/* put the far field pixels on a billboard */
  float out_mesh_scale;       /* scale factor to be applied to the mesh */
  float sub_pxl_treshold;	/* limit for subpxl approx. valid */
  float mask_low_contrast_treshold; /* treshold for low contrast mask */
  int h_tie_pts;		/* # of tiePt per row for img align/rectif. */
  int v_tie_pts;		/* # of tiePt per col for img align/rectif. */
  hMatrix2D alignMatrix;	/* matrix to align L img with Rimg */
  float rFct;			/* ppm image channel weight factor */
  float gFct;
  float bFct;
  int align_margin;		/* percentage of tie pts to reject */
  float slogW; /* diameter of the center of the center surround of the 
		  convolution kernel in the Sign of Lap. of Gauss. */
  int ref_cam; /* use the L/R cam to build the disp map dotcloud and model */
  int ref_eye; /* Eye that is used to align the camera frame to the lander's */
  float ambiColorRed;		/* VRML / IV red ambiant color */
  float ambiColorGreen;		/* VRML / IV green ambiant color */
  float ambiColorBlue;		/* VRML / IV blue ambiant  color */
  float diffColorRed;		/* VRML / IV red diffuse color */
  float diffColorGreen;		/* VRML / IV green diffuse color */
  float diffColorBlue;		/* VRML / IV blue diffuse color */
  float specColorRed;		/* VRML / IV red specular color */
  float specColorGreen;		/* VRML / IV green specular color */
  float specColorBlue;		/* VRML / IV blue specular color */
  float emisColorRed;		/* VRML / IV red emissive color */
  float emisColorGreen;		/* VRML / IV green emissive color */
  float emisColorBlue;		/* VRML / IV blue emissive color */
  float shininess;		/* VRML / IV model shininess */
  float transparency;		/* VRML / IV model transarency */
  float creaseAngle;		/* VRML / IV crease angle */
  int shapeType_solid;		/* VRML / IV back face culling */
  double mesh_tolerance;	/* tolerance of mesh */
  int max_triangles;		/* maximum number of triangles in mesh */
  int write_texture_switch; 	/* write the vrml texture switch by default
				   T.rgb, S.rgb, M.rgb, A.rgb */
  float dem_spacing;			   // DEM grid spacing
  float dem_planet_radius;		   // Zero elevation planet radius
  int ENVI_dem_data_type;		   // float = 4, long = 3, etc.
} DFT_F;			/* default file structure */

typedef struct {
  char *type;			/* Type of the image */
  int	width;			/* Image width */
  int	height;			/* Image height */
  int	total_height;		/* Height of all the buffer */
  int	max_gray;		/* Max gray level in the pgm image */
  char *in_file;		/* Input file name */
  char *in_file2;		/* Input filename for the 2nd image (vicar) */
  char *out_file;		/* Output file name */
  char *extention;		/* type file Extention */
  char *cmd_name;		/* name of the command */
  float h_theta_pixel;		/* [rad/pixel] horizontaly */
  float v_theta_pixel;		/* [rad/pixel] vertically */
  float pan;
  float tilt;			/* tilt angle */
  float hcf;			/* horizontal compression factor */
  float vcf;			/* vertical compression factor */
  float	h_fov;			/* horizontal field of view */
  float	v_fov;			/* vertical field of view */
} F_HD;				// pgm header structure


struct PixelCoords
{
  PixelCoords() { row = col = 0; }
  PixelCoords(int x, int y) { row = y; col = x; }
  int PixelIndex(int imageWidth);

  int row,col;
};			/* xy vector or vertex position */ 

#endif // __STEREO_H__
