This document describes how to process IceBridge images to create DEMs
that are aligned to the LIDAR ground reference.

1. Data location

The data is available from

https://nsidc.org/data/icebridge/data_summaries.html#camera

including camera information, raw imagery, orthorectified imagery,
LIDAR point clouds, and pre-existing DEMs for some of the flights.

Flight paths can be visualized here:

https://asapdata.arc.nasa.gov/dms/missions.html

2. Data fetching preliminaries

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

The complete list of cameras for all flights is in camera_lookup.txt
which is parsed automatically by full_processing_script.py,
handling cameras that changed in mid-flight (in directory 
StereoPipeline/src/asp/IceBridge).

3. Data fetching and archiving

The images can be fetched via the tool:

StereoPipeline/src/asp/IceBridge/full_processing_script.py 

(see inside this tool for an example usage).

To only fetch, validate, and do some conversions, use the option:

  --stop-after-fetch

To control which files are fetched and later processed use:

 --start-frame and --stop-frame 

These params do not apply to the lidar files. If you want to limit the
number of those, you can specify a small --max-num-to-fetch (note this will
limit the number of all types of files). 
  
4. Peculiar cases. 

See the notes at special_cases.txt about some things to be aware of. 

5. Working the archived data

The fetched data are stored on lou, at 

/u/oalexan1/projects/data/icebridge

The DEMs that were used to create the orthoimages and which we use to
create the camera models are in the reference_dems/ subdirectory.

To archive data to lou after fetching, use:

  --tar-and-wipe

To start with an existing lou archive, use:

  --start-with-lou-archive

These assume user name is oalexan1 for reasons too long to describe. 

The tool will attempt to update the dataset once fetched and
untarred. If necessary, it can be pushed back to lou afterwards as
described above.

Alternatively, to bring data back to pfe for processing, connect to
lou and run from the data directory a command similar to:

tar xfv AN_20091026.tar -C /nobackupnfs2/your/work/dir

It is suggested you re-run the the fetch command once you unpack it,
as some datasets that were fetched earlier may be less checked for
validity and may be missing some data.

