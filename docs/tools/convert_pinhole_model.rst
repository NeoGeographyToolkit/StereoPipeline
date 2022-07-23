.. _convert_pinhole_model:

convert_pinhole_model
---------------------

This tool can be used to approximately convert a pinhole model from one
of the types listed in :numref:`pinholemodels` or
an optical bar model (:numref:`panoramic`) to any other
pinhole model type. This can be convenient, for example, because the
Brown-Conrady and Photometrix models provide a fast formula to undistort
pixels, while the distortion operation is very slow, requiring a solver
with multiple iterations using the undistortion formula at each step,
which can make it time-consuming to run bundle adjustment and epipolar
alignment during stereo. For other models, such as Tsai and Adjustable
Tsai, the reverse is true, hence converting from the former to the
latter models can be very convenient for performance reasons.

This program can also be used to convert a pinhole or optical bar model
to a pinhole model with RPC lens distortion, which is a model where
distortion is expressed as a ratio of polynomials. The RPC lens
distortion model has the advantage that both the forward and reverse
distortion calculation are approximated using RPC, hence both of these
operations are fast, which can provide a large speedup when running
stereo and bundle adjustment.

The degree of the RPC lens distortion can be specified via
``--rpc-degree``. A smaller value is suggested to start with, as
lower-degree polynomials may be easier to interpret.

Usage::

     convert_pinhole_model [options] <input image> <input camera> \
       -o <output camera>

Example (convert a camera model to the RPC type)::

     convert_pinhole_model input.jpg input.tsai --output-type RPC \
       --rpc-degree 2 -o output_rpc.tsai

Example (specify the image dimensions instead of the image, and
convert to BrownConradyDistortion)::

     convert_pinhole_model input.tsai --output-type BrownConradyDistortion \
       --image-size "5000 4000" -o output.tsai

Command-line options for ``convert_pinhole_model``:

-o, --output-file <filename>
    Specify the output file. It is expected to have the .tsai
    extension.

--output-type <TsaiLensDistortion|BrownConradyDistortion|RPC (default: TsaiLensDistortion)>
    The output model type.

--sample-spacing <number-of-pixels>
    Pick one out of this many consecutive pixels to sample. If not
    specified, it will be auto-computed.

--rpc-degree <int (default: 3)>
    The degree of the polynomials, if the output distortion model
    is RPC.

--camera-to-ground-dist <double (default: 0)>
    The distance from the camera to the ground, in meters. This is
    necessary to convert an optical bar model to pinhole.

--image-size <"int int" (default: "")>
    Image width and height, specified as two numbers in quotes and separated 
    by a space, unless the input image file is provided.

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
