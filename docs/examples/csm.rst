.. _csm:

Community Sensor Model (CSM)
----------------------------

The Community Sensor Model (CSM), established by the U.S. defense
and intelligence community, has the goal of standardizing camera
models for various remote sensor types :cite:`CSMTRD`. It provides
a well-defined application program interface (API) for multiple
types of sensors and has been widely adopted by Earth remote sensing
software systems :cite:`hare2017community,2019EA000713`.

ASP supports and ships the USGS implementation of CSM for planetary images
(https://github.com/USGS-Astrogeology/usgscsm), which provides
Linescan, Frame, and  Synthetic Aperture Radar (SAR) implementations.

CSM is handled via dynamically loaded plugins. Hence, if a user has a
new sensor model, ASP should, in principle, be able to use it as soon
as a supporting plugin is added to the existing software, without
having to rebuild ASP or modify it otherwise. In practice, while this
logic is implemented, ASP defaults to using only the USGS
implementation, though only minor changes are needed to support
additional plugins.

Each stereo pair to be processed by ASP should be made up of two
images (for example ``.cub`` or ``.tif`` files) and two plain
text camera files with ``.json`` extension. The CSM information is
contained in the ``.json`` files and it determines which plugin to
load to use with those cameras.  More details are available at the
USGS CSM repository mentioned earlier.

.. _csm_frame:

Example using the USGS CSM Frame sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The USGS CSM *Frame* sensor models a frame camera. All the
pixels get acquired at the same time, unlike for pushbroom and
pushframe cameras, which keep on acquiring image lines as they fly
(those are considered later in the text). Hence, a single camera
center and orientation is present. This model serves the same function
as ASP's own Pinhole camera model (:numref:`pinholemodels`).

In this example we will consider images acquired with the Dawn
Framing Camera instrument, which took pictures of the Ceres and Vesta
asteroids. This particular example will be for Vesta. Note that one
more example of this sensor is shown in this documentation, in
:numref:`dawn_isis`, which uses ISIS ``.cub`` camera models rather
than CSM ones.

This example can be downloaded from:

  https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples

Creating the input images
^^^^^^^^^^^^^^^^^^^^^^^^^

Fetch the data from PDS then extract it::

    wget https://sbib.psi.edu/data/PDS-Vesta/Survey/img-1B/FC21B0004011_11224024300F1E.IMG.gz
    wget https://sbib.psi.edu/data/PDS-Vesta/Survey/img-1B/FC21B0004012_11224030401F1E.IMG.gz
      
    gunzip FC21B0004011_11224024300F1E.IMG.gz 
    gunzip FC21B0004012_11224030401F1E.IMG.gz

For simplicity of notation, we will rename these to ``left.IMG`` and ``right.IMG``.

Set up the ISIS environment. These will need adjusting for your system::

    export ISISROOT=$HOME/miniconda3/envs/isis6
    export PATH=$ISISROOT/bin:$PATH
    export ISISDATA=$HOME/isisdata

Create cub files and initialize the kernels::

    dawnfc2isis from = left.IMG  to = left.cub  target = VESTA
    dawnfc2isis from = right.IMG to = right.cub target = VESTA

    spiceinit from = left.cub  shape = ellipsoid
    spiceinit from = right.cub shape = ellipsoid

The ``target`` field is likely no longer needed in newer versions of
ISIS.

.. _create_csm_dawn:

Creation of CSM camera files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Some care is needed here, as the recipe provided below has some subtle
differences with the ones used later for linescan and SAR camera
models (:numref:`create_csm_linescan` and :numref:`create_csm_sar`).

Create a conda environment for the ``ale`` package::

    conda create -c conda-forge -n ale_env python=3.6 ale  
    conda activate ale_env

(other versions of Python may result in a runtime error later). 

Create a Python script named ``gen_csm.py``::

    #!/usr/bin/python
    
    import os, sys
    import json
    import ale
    
    prefix = sys.argv[1]
    
    if prefix.lower().endswith(".cub") or prefix.lower().endswith(".img") \
        or prefix.lower().endswith(".lbl"):
        # Wipe extension
        prefix = os.path.splitext(prefix)[0]
    
    print("Prefix is: " + prefix)
    
    cub_file = prefix + '.cub'
    img_file = prefix + '.IMG'
    
    kernels = ale.util.generate_kernels_from_cube(cub_file, expand = True)
    
    usgscsm_str = ale.loads(img_file, props={'kernels': kernels},
                            formatter='ale', verbose = False)
    
    csm_isd = prefix + '.json'
    print("Writing: " + csm_isd)
    with open(csm_isd, 'w') as isd_file:
        isd_file.write(usgscsm_str)

Assuming that conda installed this environment in the default location,
run::

    $HOME/miniconda3/envs/ale_env/bin/python gen_csm.py left.IMG
    $HOME/miniconda3/envs/ale_env/bin/python gen_csm.py right.IMG

This will create ``left.json`` and ``right.json``.

As a sanity check, run ``cam_test`` to see how well the CSM camera
approximates the ISIS camera::

    cam_test --image left.cub  --cam1 left.cub  --cam2 left.json
    cam_test --image right.cub --cam1 right.cub --cam2 right.json

Note that for a handful of pixels these errors may be big. That is a
known issue, and it seems to be due to the fact that a ray traced from
the camera center towards the ground may miss the body of the asteroid.
That should not result in inaccurate stereo results.

Running stereo
^^^^^^^^^^^^^^

::

    parallel_stereo --stereo-algorithm asp_mgm \
      --left-image-crop-win 243 161 707 825    \
      --right-image-crop-win 314 109 663 869   \
      left.cub right.cub left.json right.json  \
      run/run

    point2dem run/run-PC.tif --orthoimage run/run-L.tif 
    hillshade run/run-DEM.tif 
    colormap run/run-DEM.tif -s run/run-DEM_HILLSHADE.tif 

See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices when running stereo.

.. figure:: ../images/CSM_Frame.png
   :name: CSM_Frame_example

   The produced colorized DEM and orthoimage for the CSM Frame camera
   example. Likely using mapprojection (:numref:`mapproj-example`)
   may have reduced the number and size of the holes in the DEM.

Example using the USGS CSM linescan sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Here we use CSM for Mars images, specifically for the CTX camera,
which is a linescan sensor. The images are regular ``.cub`` files as
in the tutorial in :numref:`moc_tutorial`, hence the only distinction
compared to that example is that the cameras are stored as ``.json``
files.

We will work with the dataset pair::

     J03_045994_1986_XN_18N282W.cub J03_046060_1986_XN_18N282W.cub

which, for simplicity, we will rename to ``left.cub`` and ``right.cub``
and the same for the associated camera files.

.. _create_csm_linescan:

Creating CSM cameras from ISIS .cub files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Note that this recipe looks a little different for Frame and SAR cameras,
as can be seen in :numref:`create_csm_dawn` and :numref:`create_csm_sar`.

Run the ISIS ``spiceinit`` command on the .cub files as::

    spiceinit from = left.cub  shape = ellipsoid
    spiceinit from = right.cub shape = ellipsoid

Create a conda environment for the ``ale`` package::

    conda create -c conda-forge -n ale_env python=3.6 ale  
    conda activate ale_env

(other versions of Python may result in a runtime error later). 

Create a Python script named ``gen_csm.py``::

    #!/usr/bin/python
    
    import ale, os, sys
    
    # Get the input cub
    cub_file = sys.argv[1]
    
    # Form the output cub
    isd_file = os.path.splitext(cub_file)[0] + '.json'
    
    print("Reading: " + cub_file)
    usgscsm_str = ale.loads(cub_file)
    
    print("Writing: " + isd_file)
    with open(isd_file, 'w') as isd_file:
        isd_file.write(usgscsm_str)

Assuming that conda installed this environment in the default location,
run::

    $HOME/miniconda3/envs/ale_env/bin/python gen_csm.py camera.cub

This will produce ``left.json`` and ``right.json``.

Running stereo
^^^^^^^^^^^^^^

::

     parallel_stereo left.cub right.cub left.json right.json run/run    
     point2dem -r mars --stereographic --proj-lon 77.4 \
       --proj-lat 18.4 run/run-PC.tif

Check the stereo convergence angle as printed during preprocessing
(:numref:`stereo_pairs`). If that angle is small, the results are not
going to be great.

See :numref:`nextsteps` for a discussion about various stereo
algorithms and speed-vs-quality choices.

The actual stereo session used is ``csm``, and here it will be
auto-detected based on the extension of the camera files. For
``point2dem`` we chose to use a stereographic projection centered at
some point in the area of interest. The fancier MGM algorithm could be
used by running this example with ``--stereo-algorithm asp_mgm``.

One can also run ``parallel_stereo`` with mapprojected images
(:numref:`mapproj-example`). The first step would be to create a
low-resolution smooth DEM from the previous cloud::

     point2dem  -r mars --stereographic --proj-lon 77.4 \
       --proj-lat 18.4 run/run-PC.tif --tr 120          \
       -o run/run-smooth

followed by mapprojecting onto it and redoing stereo::

     mapproject --tr 6 run/run-smooth-DEM.tif left.cub left.json     \
       left.map.tif
     mapproject --tr 6 run/run-smooth-DEM.tif right.cub right.json   \
       right.map.tif
     parallel_stereo left.map.tif right.map.tif left.json right.json \
       run_map/run run/run-smooth-DEM.tif

Notice how we used the same resolution for both images when
mapprojecting. That helps making the resulting images more similar and
reduces the processing time (:numref:`mapproj-res`).

.. _csm_minirf:

The USGS CSM SAR sensor for LRO Mini-RF 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

*Mini-RF* was a Synthetic Aperture Radar (SAR) sensor on the LRO
spacecraft. It is challenging to process its data with ASP for several
reasons:

 - The synthetic image formation model produces curved rays going from the
   ground to the pixel in the camera (:cite:`kirk2016semi`). To simplify the
   calculations, ASP finds where a ray emanating from the camera
   intersects the standard Moon ellipsoid with radius 1737.4 km and
   declares the ray to be a straight line from the camera center to this
   point.

 - This sensor very rarely acquires stereo pairs. The convergence angle
   (:numref:`stereo_pairs`) as printed by ``parallel_stereo`` in
   pre-processing is usually less than 5 degrees, which is little and
   results in noisy DEMs. In this example we will use a dataset
   intentionally created with stereo in mind. The images will cover a
   part of Jackson crater (:cite:`kirk2011radargrammetric`).

 - It is not clear if all modeling issues with this sensor were
   resolved. The above publication states that "Comparison of the stereo
   DTM with ~250 m/post LOLA grid data revealed (in addition to
   dramatically greater detail) a very smooth discrepancy that varied
   almost quadratically with latitude and had a peak-to-peak amplitude
   of nearly 4000 m."
  
 - The images are dark and have unusual appearance, which requires
   some pre-processing and a large amount of interest points. 

Hence, ASP's support for this sensor is experimental. The results
are plausible but likely not fully rigorous.

This example, including input images, produced outputs, and a recipe, is available
for download at:

    https://github.com/NeoGeographyToolkit/StereoPipelineSolvedExamples

No ISIS data are needed to run it.

Creating the input images
^^^^^^^^^^^^^^^^^^^^^^^^^

Fetch the data from PDS::

    wget https://pds-geosciences.wustl.edu/lro/lro-l-mrflro-4-cdr-v1/lromrf_0002/data/sar/03800_03899/level1/lsz_03821_1cd_xku_16n196_v1.img
    wget https://pds-geosciences.wustl.edu/lro/lro-l-mrflro-4-cdr-v1/lromrf_0002/data/sar/03800_03899/level1/lsz_03821_1cd_xku_16n196_v1.lbl
    wget https://pds-geosciences.wustl.edu/lro/lro-l-mrflro-4-cdr-v1/lromrf_0002/data/sar/03800_03899/level1/lsz_03822_1cd_xku_23n196_v1.img
    wget https://pds-geosciences.wustl.edu/lro/lro-l-mrflro-4-cdr-v1/lromrf_0002/data/sar/03800_03899/level1/lsz_03822_1cd_xku_23n196_v1.lbl

These will be renamed to ``left.img``, ``right.img``, etc., to simply
the processing.

Create .cub files::

    export ISISROOT=$HOME/miniconda3/envs/isis6
    export PATH=$ISISROOT/bin:$PATH
    export ISISDATA=$HOME/isis3data
   
    mrf2isis from = left.lbl  to = left.cub
    mrf2isis from = right.lbl to = right.cub

Run ``spiceinit``. Setting the shape to the ellipsoid makes it easier
to do image-to-ground computations and is strongly suggested::

    spiceinit from = left.cub  shape = ellipsoid
    spiceinit from = right.cub shape = ellipsoid

.. _create_csm_sar:

Creating the CSM cameras
^^^^^^^^^^^^^^^^^^^^^^^^

Fetch the latest ``ale`` from GitHub:

    https://github.com/USGS-Astrogeology/ale

or something newer than version 0.8.7 on conda-forge, which lacks
certain functionality for SAR. Below we assume a very recent version
of USGS CSM, as shipped with ASP. Version 1.5.2 of this package on
conda-forge is too old for the following to work.

Create a script called ``gen_json.py``. (Note that this script
differs somewhat for analogous scripts earlier in the text, at
:numref:`create_csm_dawn` and :numref:`create_csm_linescan`.)

::

    #!/usr/bin/python
    
    import os, sys
    import json
    import ale
    
    prefix = sys.argv[1]
    
    if prefix.lower().endswith(".cub") or prefix.lower().endswith(".img") \
      or prefix.lower().endswith(".lbl"):
      # Remove extension
      prefix = os.path.splitext(prefix)[0]
    
    cub_file = prefix + '.cub'
    print("Loading cub file: " + cub_file)
    
    kernels = ale.util.generate_kernels_from_cube(cub_file, expand = True)
    usgscsm_str = ale.loads(cub_file, formatter = "ale", \
      props={"kernels": kernels}, verbose = False)
    
    csm_isd = prefix + '.json'
    print("Saving: " + csm_isd)
    with open(csm_isd, 'w') as isd_file:
      isd_file.write(usgscsm_str)
    
Run it as::

   python gen_json.py left.cub
   python gen_json.py right.cub

The above paths will need adjusting for your system. The path to
Python should be such that the recently installed ``ale`` is picked
up.

Run ``cam_test`` (:numref:`cam_test`) as a sanity check::

    cam_test --image left.cub  --cam1 left.cub  --cam2 left.json
    cam_test --image right.cub --cam1 right.cub --cam2 right.json

Preparing the images
^^^^^^^^^^^^^^^^^^^^

ASP accepts only single-band images, while these .cub files have four of them.
We will pull the first band and clamp it to make it easier for stereo to find
interest point matches::

    gdal_translate -b 1 left.cub  left_b1.tif
    gdal_translate -b 1 right.cub right_b1.tif

    image_calc -c "min(var_0, 0.5)" left_b1.tif  -d float32 \
      -o left_b1_clamp.tif 
    image_calc -c "min(var_0, 0.5)" right_b1.tif -d float32 \
      -o right_b1_clamp.tif 

Running stereo
^^^^^^^^^^^^^^

It is simpler to first run a clip with ``stereo_gui``
(:numref:`stereo_gui`).  This will result in the following command::

    parallel_stereo --ip-per-tile 3500             \
      --left-image-crop-win 0 3531 3716 10699      \
      --right-image-crop-win -513 22764 3350 10783 \
      --stereo-algorithm asp_mgm --min-num-ip 10   \
      left_b1_clamp.tif right_b1_clamp.tif         \
      left.json right.json run/run  

The stereo convergence angle for this pair is 18.4 degrees which is
rather decent.

Create a colorized DEM and orthoimage::

    point2dem run/run-PC.tif --orthoimage run/run-L.tif 
    hillshade run/run-DEM.tif 
    colormap run/run-DEM.tif -s run/run-DEM_HILLSHADE.tif 

See :numref:`nextsteps` for a discussion about various
speed-vs-quality choices when running stereo.

.. figure:: ../images/CSM_MiniRF.png
   :name: CSM_miniRF_example

   The produced colorized DEM and orthoimage for the CSM SAR example. 

.. _csm_msl:

Using CSM cameras with MSL
~~~~~~~~~~~~~~~~~~~~~~~~~~

This example shows how given a set of Mars Science Laboratory (MSL) Curiosity rover ``navcam`` images, CSM camera models can be created. Stereo pairs are then used to make DEMs.

It is important to note that, as long as the rover is fixed in place, the cameras corresponding to overlapping images are self-consistent, and the produced DEMs have a roughly correct position and orientation.

If the rover moves, however, the rover height above the Mars datum and the corresponding DEM can jump vertically by 60 meters or so, in some circumstances, which appears to be due to problems in in the input SPICE data. The orientation can jump as well, sometimes by a few tens of degreses in azimuth. The altitude seems rather stable, so the rover maintains a roughly horizontal orientation. It is not known if the orientation issue is because of the input data or insufficeintly accurate modeling.

Hence, for now this functionality can only be used to create small fused DEMs from 6-12 images, and not for large DEMs covering the whole rover traverse.

See :numref:`rig_msl` for a Structure-from-Motion solution without using CSM cameras.

Fetching the data
^^^^^^^^^^^^^^^^^^

See :numref:`msl_image_prep`. Here we will work with .cub files rather than converting them to .png. The same Mars day will be used (SOL 597).


.. _csm_state:

Exporting CSM model state
~~~~~~~~~~~~~~~~~~~~~~~~~

ASP's bundle adjustment program (:numref:`bundle_adjust`) normally
writes plain text ``.adjust`` files which encode how the position and
orientation of the cameras were modified (:numref:`adjust_files`). If
invoked for CSM cameras, additional files with extension
``.adjusted_state.json`` are saved in the same output directory, which
contain the model state from the input CSM cameras with the
optimization adjustments applied to them (use zero iterations in
``bundle_adjust`` to save the states of the original cameras).

This functionality is only implemented for USGS CSM ``linescan`` and
``SAR`` models.

It is important to note that the ``model state`` of a CSM camera
and the CSM camera itself, while both stored on disk as JSON files,
are not the same thing. The CSM camera file (also called the ``CSM
ISD`` file) has the transforms from sensor coordinates to J2000 and from
J2000 to ECEF. These are then combined together to form the model
state, which has the transforms from the sensor to ECEF. The model
state is used to project ground points into the camera and vice-versa,
so it is sufficient for the purposes of bundle adjustment and stereo.

ASP's ``parallel_stereo`` and bundle adjustment programs can, in addition to CSM
ISD camera model files, also load such model state files, either as
previously written by ASP or from an external source (it will
auto-detect the type from the format of the JSON files). Hence, the
model state is a convenient format for data exchange, while being
less complex than the ISD format.

If ASP's ``parallel_stereo`` program is used to create a point cloud from
images and CSM cameras, and then that point cloud has a transform
applied to it, such as with ``pc_align``, the same transform can be
applied to the model states for the two cameras, which are then saved
to disk as earlier.  That is accomplished by invoking bundle
adjustment with the input images and cameras as follows::

    bundle_adjust left.cub right.cub left.json right.json \
      --initial-transform transform.txt                   \
      --apply-initial-transform-only -o ba/run
 
This will save the state files ``ba/run-left.adjusted_state.json`` and
``ba/run-right.adjusted_state.json``. If it is desired to simply
export the model state of the initial cameras without any alignment,
then the transform passed in can be the identity matrix of size 4.

In case first bundle adjustment was used, then ``parallel_stereo`` was run with
bundle adjusted cameras, then ``pc_align`` was invoked on the
resulting point cloud, obtaining an alignment transform, and is
desired to create model state files having both the effect of bundle
adjustment and subsequent alignment, one can invoke bundle adjustment
just as above, with an initial transform and zero iterations, but use
not the original ``left.json`` and ``right.json`` camera files, but
the model state files after the initial bundle adjustment which encode
that adjustment. (See also :numref:`ba_pc_align` for how to combine
bundle adjustment with the alignment transform.) 

To evaluate how well the obtained CSM camera approximates the ISIS
camera model, run the program ``cam_test`` shipped with ASP
(:numref:`cam_test`) as follows::

    cam_test --sample-rate 100 --image camera.cub \
      --cam1 camera.cub --cam2 camera.json

The pixel errors are expected to be at most on the order of 0.2
pixels.

