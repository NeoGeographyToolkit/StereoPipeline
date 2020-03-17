.. _add_spot_rpc:

add_spot_rpc
------------

The ``add_spot_rpc`` tool creates an RPC model to approximate a SPOT5
sensor model. The RPC model can be appended to the end of a SPOT5
metadata file, allowing it to be used with the RPC session type in other
ASP tools. The most important application is to map project SPOT5
images, then to perform stereo on the map projected images with the
``spot5maprpc`` session type.

If the output file does not exist, a new file is created containing the
RPC model. Otherwise the RPC model is appended to an existing file. When
an existing SPOT5 metadata file is the output file, the new RPC model is
properly inserted into the file so that it is ready to use.

An example for how to use this tool is given in :numref:`spot5`.

Usage::

     add_spot_rpc <input metadata file> -o <output file>

It is important to note that the tool expects the minimum and maximum
simulation box heights (in meters, above the datum) in which to compute
the RPC approximation. The defaults are 0 and 8000, corresponding to sea
level and the highest location on Earth. Narrowing down these numbers
(if it is known what range of terrain heights is expected) may result in
slightly more accurate models.

Command-line options for add_spot_rpc:

-o, --output-prefix <arg>
    Specify the output prefix.

--min-height <arg (default: 0)>
    The minimum height (in meters) above the WGS84 datum of the
    simulation box in which to compute the RPC approximation.

--max-height <arg (default: 8000)>
    The maximum height (in meters) above the WGS84 datum of the
    simulation box in which to compute the RPC approximation.

--num-samples <arg (default: 100)>
    How many samples to use between the minimum and maximum heights.

--penalty-weight <arg (default: 0.1)>
    Penalty weight to use to keep the higher-order RPC coefficients
    small. Higher penalty weight results in smaller such coefficients.

-v, --version
    Display the version of software.

-h, --help
    Display this help message.
