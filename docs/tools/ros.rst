.. _ros_tools:

ROS bag handling tools
----------------------

ASP provides a couple of Python scripts that can query and extract
images and depth clouds from a ROS bag. These are stored in the
``libexec`` directory of the ASP distribution, as they use the local
system's Python rather than the Python shipped with ASP. It is
assumed that ROS is installed on the local system.

These tools are motivated by the fact that robot Structure-from-Motion
datasets often come in ROS bags. For context and examples, see
:numref:`sfm_isis_data_prep`.

.. _ros_tools_list:

list_timestamps
^^^^^^^^^^^^^^^

Write the list of timestamps for all data in the bag for given
topics. Example usage::

    /usr/bin/python                        \
      /path/to/ASP/libexec/list_timestamps \
      --bag bag.bag --list bag_list.txt    \
      --topics "/topic1 /topic2"
 
The output list will have entries of the form::

    1654694879.1690574 /topic1

The first value is a timestamp, in seconds since epoch. It is not
the message timestamp, which is the time the message arrived at,
but the timestamp saved in the message header, which is assumed
to have the creation time of an image or depth cloud. 

.. _ros_tools_extract:

extract_bag
^^^^^^^^^^^

Extract images and depth clouds from a bag. See
:numref:`sfm_isis_data_prep` for usage.

Command line options:


``--bag`` <string>
  Input bag.
``--topics`` <string>
  A list of topics, in quotes, having image, compressed image, or
  point cloud (depth) data.
``--dirs``  <string>
  A list of directories, in quotes, one for each topic, 
  in which to save the data for that topic.
``--timestamp_list`` <string>
  Extract data for the timestamps in this list. If not set,
  extract all data. Each line in this file must contain
  a number of the form <digits>.<digits> (the timestamp) and perhaps
  other text as well, or it will be ignored. So, a filename containing
  a timestamp as part of its name will be accepted.
``--timestamp_tol`` <float>
  If set, extract the data for each the given topics whose timestamps are closest
  to the ones in the input list, within this tolerance, in
  seconds. This should be kept small. It is assumed
  the bag stores the data for each topic in increasing value of
  timestamp.
``--approx_timestamp`` 
  If using ``--timestamp_tol``, change the timestamp of the data being
  saved (which becomes part of the output filename) to be the closest
  timestamp in the input list. 
