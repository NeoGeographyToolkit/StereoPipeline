.. _sat_sim:

sat_sim
-------

The ``sat_sim`` satellite simulator program models a satellite traveling around a planet and taking pictures. It writes the obtained synthetic images and Pinhole (:numref:`pinholemodels`) camera files on disk.

The inputs are a DEM and georeferenced image of the area of interest, the end points of the orbit, and number of samples. The orbit is represented as a straight edge in the projected coordinate system of the DEM, which results in an arc around the planet. 

The nominal orientation of the cameras is with the *x*, *y* and *z* axes pointing along satellite track, across track, and towards the planet, respectively. Roll, pitch, and yaw angles can be used to change the camera orientations.

Example
^^^^^^^

::
  
    sat_sim --dem dem.tif --ortho ortho.tif          \
    --first 397 496 2500 --last 397 495 2500 --num 5 \
    --focal-length 10000 --optical-center 400 300    \
    --image-size 800 600                             \
    -o out/run 

Command-line options
^^^^^^^^^^^^^^^^^^^^

--dem <string (default="")>
    Input DEM file.

--ortho <string (default="")>
    Input ortho image file. 

-o, --output-prefix <string (default="")>
    Specify the output prefix. All the files that are saved will start with this prefix.

--first <string (default="")>
    First camera position, specified as DEM pixel column and row, and height above the DEM datum.

--last <string (default="")>
    Last camera position, specified as DEM pixel column and row, and height above the DEM datum.

--num <int (default=0)>
    Number of cameras to generate, including the first and last ones. Must be positive. The cameras are uniformly distributed along the straight edge from first to last (in projected coordinates).

--focal-length <double>
    Output camera focal length in units of pixel.

--optical-center <float, float>
    Output camera optical center (image column and row).

--image-size <int, int>
    Output camera image size (width and height).

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.
 
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--tif-compress <string (default = "LZW")>
    TIFF compression method. Options: None, LZW, Deflate, Packbits.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
