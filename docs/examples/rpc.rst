.. _rpc:

RPC camera models
-----------------

Some vendors, such as GeoEye with its Ikonos and two
GeoEye satellites, Airbus, with its SPOT and Pleiades satellites, the
Indian Cartosat-1 satellite, PeruSat-1, the Spanish Deimos 1 and 2,
etc., provide Rational Polynomial Coefficient (RPC) camera models.

(Certain providers also offer exact linescan models. ASP supports the
ones from DigitalGlobe/Maxar (:numref:`dg_tutorial`),
PeruSat-1 (:numref:`perusat1`), Pleiades 1A/1B (:numref:`pleiades`),
and SPOT 5 (:numref:`spot5`).)

About RPC
~~~~~~~~~

RPC represents four 20-element polynomials that map geodetic coordinates
(longitude-latitude-height above datum) to image pixels. Since they are
easy to implement and fast to evaluate, RPC represents a universal
camera model providing a simple approximation to complex exact camera
models that are unique to each vendor. The only downside is that it has
less precision in our opinion compared to the exact camera models.

Our RPC read driver is GDAL. If the command ``gdalinfo``
(:numref:`gdal_tools`) can identify the RPC information inside the
headers of your image files (whether that information is actually
embedded in the images, or stored separately in some auxiliary files
with a convention GDAL understands), ASP will likely be able to see it
as well. This means that sometimes we can get away with only providing
a left and right image, with no extra files containing camera
information. This is specifically the case for GeoEye, and
Cartosat-1. 

Otherwise, the camera files must be specified separately in XML files, as done
for DigitalGlobe/Maxar images (:numref:`rawdg`) and PeruSat-1.

See :numref:`airbus_tiled` if the input Pleiades images arrive in multiple
tiles.

Examples
~~~~~~~~

A sample stereo pair can be downloaded from from GeoEye's website at
:cite:`geoeye:samples`. When we accessed the site, we downloaded a GeoEye-1
image of Hobart, Australia. As previously stated in :numref:`dg_tutorial`, these
types of images are not ideal for ASP. This is both a forest and a urban area
which makes correlation difficult. ASP was designed more for modeling bare rock
and ice. Any results we produce in other environments is a bonus but is not our
objective.

.. figure:: ../images/examples/geoeye/GeoEye_CloseUp_triple.png
   :name: geoeye-nomap-example

   Example colorized height map and ortho image output, produced
   with ``point2dem`` (:numref:`point2dem`) and ``mapproject``
   (:numref:`mapproject`), respectively.

GoeEye's datasets have the RPC coefficients stored as part of the
images. The stereo command is then::

    parallel_stereo -t rpc       \
      --stereo-algorithm asp_mgm \
      --subpixel-mode 9          \
      left.tif right.tif         \
      results/run

See :numref:`nextsteps` for a discussion about various speed-vs-quality choices.

For terrains having steep slopes, we recommend that images be mapprojected onto
an existing DEM before running stereo. This is described in
:numref:`mapproj-example`.

Next, ``point2dem`` (:numref:`point2dem`) is run::

    point2dem --auto-proj-center results/run-PC.tif
    
For some cameras the RPC coefficients are stored in separate files ending in
.RPB or \_RPC.TXT (or in lower-case). These will be loaded automatically and
should not be specified in the stereo command. 

For Cartosat data sometimes one should overwrite the \_RPC.TXT files
that are present with the ones that end in RPC_ORG.TXT in order for
stereo to work.

If the RPC cameras are stored separately in XML files, the stereo 
command is::

    parallel_stereo -t rpc       \
      --stereo-algorithm asp_mgm \
      --subpixel-mode 9          \
      left.tif right.tif         \
      left.xml right.xml         \
      results/run

The RPC cameras can be bundle-adjusted (:numref:`bundle_adjust`).

If the RPC coefficients are stored in the input images, ``mapproject``
copies them to the output mapprojected images. If these coefficients
are in the associated .RPB or \_RPC.TXT files, ``mapproject`` creates
such files for each mapprojected image.

See :numref:`other-mapproj` for how ``parallel_stereo`` is invoked
with mapprojected images when the cameras are stored either separately
or part of the images.

Creation of RPC cameras
~~~~~~~~~~~~~~~~~~~~~~~

In addition to supporting the provided RPC models, ASP provides a
tool named ``cam2rpc`` (:numref:`cam2rpc`), that can be
used to create RPC camera models from ISIS and all other cameras that
ASP understands, including for non-Earth planets (currently only the
Earth, Moon and Mars are supported). 

In such situations, the planet datum must be passed to the tools reading the RPC
models, via the ``--datum`` option. 

