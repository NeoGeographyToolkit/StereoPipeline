.. _spot5:

SPOT5
-----

SPOT5 is a CNES (Space Agency of France) satellite launched on May 2002
and decommissioned in March 2015. SPOT5 contained two High Resolution
Stereoscopic (HRS) instruments with a ground resolution of 5 meters.
These two cameras were pointed forwards and backwards, allowing capture
of a stereo image pair in a single pass of the satellite.

ASP supports only images from the HRS sensors on SPOT5. These images
come in two parts, the data file (extension ``.bil`` or ``.tif``) and
the header file the data file (extension ``.dim``). The data file can be
either a plain binary file with no header information or a GeoTIFF file.
The header file is a plain text XML file. When using SPOT5 images with
ASP tools, pass in the data file as the image file and the header file
as the camera model file.

All ASP tools can handle ``.bil`` images (and also ``.bip`` and ``.bsq``)
as long as a similarly named ``.dim`` file exists that can be looked
up. The lookup succeeds if, for example, the ``.dim`` and ``.bil``
files differ only by extension (lower or upper case), or, as below,
when an IMAGERY.BIL file has a corresponding METADATA file.

A sample SPOT5 image can be found at at
http://www.geo-airbusds.com/en/23-sample-imagery.

One issue to watch out for is that SPOT5 data typically comes in a
standard directory structure where the image and header files always
have the same name. The header (camera model) files cannot be passed
into the ``bundle_adjust`` tool with the same file name even if they are
in different folders. A simple workaround is to create symbolic links to
the original header files with different names::

    ln -s front/SEGMT01/METADATA.DIM front/SEGMT01/METADATA_FRONT.DIM
    ln -s back/SEGMT01/METADATA.DIM  back/SEGMT01/METADATA_BACK.DIM

The same should be done for image files, for consistency. 

Alternatively, the images and cameras can just be renamed to have different
names.
    
Then run bundle adjustment (:numref:`bundle_adjust`)::

    bundle_adjust -t spot5             \
      front/SEGMT01/IMAGERY_FRONT.BIL  \
      back/SEGMT01/IMAGERY_BACK.BIL    \
      front/SEGMT01/METADATA_FRONT.DIM \
      back/SEGMT01/METADATA_BACK.DIM   \
      -o ba_run/out
      
Run ``parallel_stereo`` (:numref:`parallel_stereo`) with the adjusted cameras::

    parallel_stereo -t spot5            \
      --bundle-adjust-prefix ba_run/out \
      --stereo-algorithm asp_mgm        \
      front/SEGMT01/IMAGERY_FRONT.BIL   \
      back/SEGMT01/IMAGERY_BACK.BIL     \
      front/SEGMT01/METADATA_FRONT.DIM  \
      back/SEGMT01/METADATA_BACK.DIM    \
      st_run/out 

Here uses the ``asp_mgm`` algorithm. See :numref:`nextsteps` for a discussion
about various speed-vs-quality choices of the stereo algorithms. 

This is followed by DEM creation with ``point2dem`` (:numref:`point2dem`)::

    point2dem st_run/out-PC.tif

For terrains with steep slopes, it is strongly suggested to run stereo with
mapprojected images (:numref:`mapproj-example`). For that, first use the
``add_spot_rpc`` tool to generate an RPC model approximation of the SPOT5 sensor
model.

::

    add_spot_rpc front/SEGMT01/METADATA_FRONT.DIM \
      -o front/SEGMT01/METADATA_FRONT.DIM
    add_spot_rpc back/SEGMT01/METADATA.DIM        \
      -o back/SEGMT01/METADATA_BACK.DIM

This will append the RPC model to the existing file. If the output
is a separate file, only the RPC model will be saved to the new file.

Then use the ``spot5maprpc`` session type when running parallel_stereo on the
mapprojected images. See the note in :numref:`mapproj-example` about perhaps
reducing the resolution of the DEM to mapproject onto (and perhaps blurring it)
if ghosting artifacts are seen in the produced DEM.

Mapprojection (:numref:`mapproject`)::

    mapproject -t rpc                   \
      --bundle-adjust-prefix ba_run/out \
      --tr gridSize                     \
      sample_dem.tif                    \
      front/SEGMT01/IMAGERY_FRONT.BIL   \
      front/SEGMT01/METADATA_FRONT.DIM  \
      front_map_proj.tif
    mapproject -t rpc                   \
      --bundle-adjust-prefix ba_run/out \
      --ref-map front_map_proj.tif      \
      sample_dem.tif                    \
      back/SEGMT01/IMAGERY_BACK.BIL     \
      back/SEGMT01/METADATA_BACK.DIM    \
      back_map_proj.tif

The grid size is the known ground sample distance (GSD) of the image, in meters.
If not set, it will be auto-guessed.
      
Notice how we used the option ``--ref-map`` to ensure the second mapprojected
image uses the same grid size and projection as the first one. In older versions
of ASP, one must specify for both images the same projection in meters (such as
UTM), via ``--t_srs``, and the same grid size, via ``--tr``. 

Stereo::

    parallel_stereo -t spot5maprpc      \
      --bundle-adjust-prefix ba_run/out \
      --stereo-algorithm asp_mgm        \
      front_map_proj.tif                \
      back_map_proj.tif                 \
      front/SEGMT01/METADATA_FRONT.DIM  \
      back/SEGMT01/METADATA_BACK.DIM    \
      st_run_map/out                    \
      sample_dem.tif

DEM creation::
      
    point2dem st_run_map/out-PC.tif

See :numref:`nextsteps` for a discussion about various speed-vs-quality choices
of the stereo algorithms.

If desired not to use bundle adjustment, then need not use the option ``--bundle-adjust-prefix``.

.. figure:: ../images/examples/spot5_figure.png
   :name: spot5_output
         
   Cropped region of SPOT5 image and a portion of the associated stereo
   DEM overlaid on a low resolution Bedmap2 DEM.
