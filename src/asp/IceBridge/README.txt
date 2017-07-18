This document describes how to process IceBridge images to create DEMs
that are aligned to the LIDAR ground reference.

The data is available from

https://nsidc.org/data/icebridge/data_summaries.html#camera

including camera information, raw imagery, orthorectified imagery,
LIDAR point clouds, and pre-existing DEMs for some of the flights.

To be able to fetch any data, one should create an account with NSIDC,
and then set up bulk downloading by following the instructions at

https://nsidc.org/support/faq/what-options-are-available-bulk-downloading-data-https-earthdata-login-enabled

For some flights, the camera (and hence its calibration file) changed
in mid-flight, detailed info about this is available at

https://nsidc.org/data/IODCC0/versions/1

in the "User Guide" tab. The camera calibration files can be fetched
recursively using the command:

wget -e robots=off --load-cookies ~/.urs_cookies --save-cookies ~/.urs_cookies --keep-session-cookies --no-check-certificate --auth-no-challenge=on --no-parent -r -l 10 https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/IODCC0.001/

These files are in the PDF format, and can be converted to the .tsai
format used by ASP using the script

StereoPipeline/src/asp/IceBridge/process_calibration_file.py 

The images can then be processed via the tool:

StereoPipeline/src/asp/IceBridge/full_processing_script.py 

(see inside this tool for an example usage).

The complete list of cameras for all flights is in camera_lookup.txt
which is parsed automatically by full_processing_script.py,
handling cameras that changed in mid-flight (in directory 
StereoPipeline/src/asp/IceBridge).

See also the notes at special_cases.txt at the same location.

The data are archived on lou, at 

/u/oalexan1/projects/data/icebridge

The DEMs that were used to create the orthoimages and which we use to
create the camera models are in the reference_dems/ subdirectory.

