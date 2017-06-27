This document describes how to process IceBridge images to create DEMs
that are aligned to the LIDAR ground reference.

The data is available from

https://nsidc.org/data/icebridge/data_summaries.html#camera

including camera information, raw imagery, orthorectified imagery,
LIDAR point clouds, and pre-existing DEMs for some of the flights.

To be able to fetch any data, one should create an account with NSIDC,
and then set up bulk downloading by following the instructions at

https://nsidc.org/support/faq/what-options-are-available-bulk-downloading-data-https-earthdata-login-enabled

For some flights, the camera (and hence its calibration file) changed in mid-flight, detailed info about this is available at 

https://nsidc.org/data/IODCC0/versions/1

in the "User Guide" tab. The camera calibration files can be fetched
recursively using the command:

wget -e robots=off --load-cookies ~/.urs_cookies --save-cookies ~/.urs_cookies --keep-session-cookies --no-check-certificate --auth-no-challenge=on --no-parent -r -l 10 https://n5eil01u.ecs.nsidc.org/ICEBRIDGE/IODCC0.001/

These files are in the PDF format, and can be converted to the .tsai
format used by ASP using the script

StereoPipeline/src/asp/IceBridge/process_calibration_file.py 

The images can then be processed via the tool:

StereoPipeline/src/asp/IceBridge/full_processing_script.py 

It is important to note that sometimes the camera changes in
mid-flight. One should not process together images that do not all
have the same camera calibration file.

