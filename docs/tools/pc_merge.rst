.. _pc_merge:

pc_merge
--------

This is a simple tool for combining multiple ASP-generated point cloud
files into a single concatenated file. The output file will be float32
unless the input images are float64 or the user has specified the
float64 option.

``pc_merge`` can merge clouds with 1, 3, 4, and 6 bands. In particular, it can
merge *output-prefix*-L.tif images created by ``stereo``
(:numref:`outputfiles`). This is useful if it is desired to create an
orthoimage from a merged cloud with ``point2dem`` (:numref:`point2dem`).

In that case, one can invoke ``pc_merge`` on individual ``L.tif`` files to
create a merged texture file to pass to ``point2dem`` together with the merged
point cloud tile.

Example::

  pc_merge cloud1.tif cloud2.tif -o merged_cloud.tif
  
Usage::

    pc_merge [options] <input files> -o <output file>

Command-line options for pc_merge:

-d, --write-double
    Force output file to be float64 instead of float32.

-o, --output-file <name>
    Specify the output file (required).

--threads <integer (default: 0)>
    Select the number of threads to use for each process. If 0, use
    the value in ~/.vwrc.
 
--cache-size-mb <integer (default = 1024)>
    Set the system cache size, in MB.

--tile-size <integer (default: 256 256)>
    Image tile size used for multi-threaded processing.

--no-bigtiff
    Tell GDAL to not create bigtiffs.

--tif-compress <None|LZW|Deflate|Packbits (default: LZW)>
    TIFF compression method.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
