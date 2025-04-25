.. _sfm_proc:

sfm_proc
--------

This helper program takes as input a list of images with EXIF data, such as
produced by an UAS flight, and writes a text file having on each line the image
name, camera center longitude, latitude, and height above datum, then the roll,
pitch, and yaw of the camera.

The roll and pitch are set to 0 as they are not available in the EXIF data. The yaw,
measured from the North direction, is read from the field ``EXIF_GPSImgDirection``.

The fields ``EXIF_GPSLongitude``, ``EXIF_GPSLatitude``, ``EXIF_GPSAltitude``, and
``EXIF_GPSLongitudeRef`` are read for the other data mentioned earlier.

An example is in :numref:`sfm_uas`.

Command-line options
~~~~~~~~~~~~~~~~~~~~
  
--image-list <string (default: "")>
    A file listing the input images with EXIF data, one per line.
    
--out-dir <string (default: "")>
    The output directory that will contain the processed data.
    
-h, --help
    Display this help message.
