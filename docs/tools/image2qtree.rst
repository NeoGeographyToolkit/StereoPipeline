.. _image2qtree:

image2qtree
-----------

``image2qtree`` turns a georeferenced image (or images) into a quadtree
with geographical metadata for viewing in Google Earth. See 
:numref:`google_earth_overlays` for usage examples.

Command-line options for image2qtree:

--help
    Display a help message.

-o, --output-name <directory-name>
    Specify the base output directory.

-q, --quiet
    Quiet output.

-v, --verbose
    Verbose output.

--cache <number-of-MB (default: 1024)>
    Cache size, in megabytes.

--force-wgs84
    Use WGS84 as the input images' geographic coordinate systems,
    even if they’re not (old behavior).

--pixel-scale <factor (default: 1)>
    Scale factor to apply to pixels.

--pixel-offset <offset (default: 0)>
    Offset to apply to pixels.

--normalize
    Normalize input images so that their full dynamic range falls
    in between [0,255].

-m, --output-metadata <kmltms|uniview|gmap|celestia|none (default: none)>
    Specify the output metadata type.

--file-type <type (default: png)>
    Output file type.

--channel-type <uint8|uint16|int16|float (default: uint8)>
    Output (and input) channel type.

--module-name <name (default: marsds)>
    The module where the output will be placed. Ex: marsds for
    Uniview, or Sol/Mars for Celestia.

--terrain
    Outputs image files suitable for a Uniview terrain view. Implies
    output format as PNG, channel type uint16. Uniview only.

--jpeg-quality <factor (default: 0.75)>
    JPEG quality factor (0.0 to 1.0).

--png-compression <level (default: 3)>
    PNG compression level (0 to 9).

--palette-file <filename>
    Apply a palette from the given file.

--palette-scale <factor>
    Apply a scale factor before applying the palette.

--palette-offset <value>
    Apply an offset before applying the palette.

--tile-size <number-of-pixels (default: 256)>
    Tile size, in pixels.

--max-lod-pixels <number-of-pixels (default: 1024)>
    Max LoD in pixels, or -1 for none (kml only).

--draw-order-offset <value (default: 0)>
    Offset for the ``<drawOrder>`` tag for this overlay (kml only).

--composite-multiband
    Composite images using multi-band blending.

--aspect-ratio <ratio (default: 1)>
    Pixel aspect ratio (for polar overlays; should be a power of two).

--north <latitude-in-degrees>
    The northernmost latitude in degrees.

--south <latitude-in-degrees>
    The southernmost latitude in degrees.

--east <longitude-in-degrees>
    The easternmost longitude in degrees.

--west <longitude-in-degrees>
    The westernmost longitude in degrees.

--sinusoidal
    Assume a sinusoidal projection.

--mercator
    Assume a Mercator projection.

--transverse-mercator
    Assume a transverse Mercator projection.

--orthographic
    Assume an orthographic projection.

--stereographic
    Assume a stereographic projection.

--lambert-azimuthal
    Assume a Lambert azimuthal projection.

--lambert-conformal-conic
    Assume a Lambert Conformal Conic projection.

--utm <zone>
    Assume UTM projection with the given zone.

--proj-lat <latitude>
    The center of projection latitude (if applicable).

--proj-lon <longitude>
    The center of projection longitude (if applicable).

--proj-scale <scale>
    The projection scale (if applicable).

--std-parallel1 <latitude>
    Standard parallels for Lambert Conformal Conic projection.

--std-parallel2 <latitude>
    Standard parallels for Lambert Conformal Conic projection.

--nudge-x <arg>
    Nudge the image, in projected coordinates.

--nudge-y <arg>
    Nudge the image, in projected coordinates.
