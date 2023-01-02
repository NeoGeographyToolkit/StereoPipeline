.. _installation:

.. include:: ../INSTALLGUIDE.rst

.. _vwrc:

Settings optimization
---------------------

Finally, the last thing to be done for Stereo Pipeline is to setup up
Vision Workbench's render and logging settings. This step is optional,
but for best performance some thought should be applied here.

Vision Workbench is a multi-threaded image processing library used by
Stereo Pipeline. The settings by which Vision Workbench processes data
are configurable by having a ``.vwrc`` file hidden in your home directory.
Below is an example::

  # This is an example VW configuration file. Save this file to
  # ~/.vwrc to adjust the VW log settings, even if the program is
  #already running.

  # General settings
  [general]
  default_num_threads = 16
  write_pool_size = 40
  system_cache_size = 1073741824 # ~ 1 GB
  
  # The following integers are associated with the log levels
  # throughout the Vision Workbench.  Use these in the log rules
  # below.
  #
  #    ErrorMessage = 0
  #    WarningMessage = 10
  #    InfoMessage = 20
  #    DebugMessage = 30
  #    VerboseDebugMessage = 40
  #    EveryMessage = 100
  #
  # You can create a new log file or adjust the settings 
  # for the console log:
  #   logfile <filename> 
  #       - or -
  #   logfile console
  
  # Once you have created a logfile (or selected the console), you
  # can add log rules using the following syntax. (Note that you
  # can use wildcard characters '*' to catch all log_levels for a
  # given log_namespace, or vice versa.)
  
  # <log_level> <log_namespace>
  
  # Below are examples of using the log settings.
  
  # Turn on various logging levels for several subsystems, with
  # the output going to the console (standard output).
  [logfile console]
  # Turn on error and warning messages for the thread subsystem.
  10 = thread
  # Turn on error, warning, and info messages for the 
  # asp subsystem.
  20 = asp
  # Turn on error, warning, info, and debug messages for the 
  # stereo subsystem.
  30 = stereo
  # Turn on every single message for the cache subsystem (this will
  # be extremely verbose and is not recommended).
  # 100 = cache
  # Turn off all progress bars to the console (not recommended).
  # 0 = *.progress
  
  # Turn on logging of error and warning messages to a file for the
  # stereo subsystem. Warning: This file will be always appended
  # to, so it should be deleted periodically.
  # [logfile /tmp/vw_log.txt]
  # 10 = stereo

There are a lot of possible options that can be implemented in the above
example. Let's cover the most important options and the concerns the
user should have when selecting a value.

Performance settings
~~~~~~~~~~~~~~~~~~~~

``default_num_threads`` (default=2)
   This sets the maximum number of threads that can be used for
   rendering. When stereo's ``subpixel_rfne`` is running you'll
   probably notice 10 threads are running when you have
   ``default_num_threads`` set to 8. This is not an error, you are
   seeing 8 threads being used for rendering, 1 thread for holding
   ``main()``'s execution, and finally 1 optional thread acting as
   the interface to the file driver.

   It is usually best to set this parameter equal to the number of
   processors on your system. Be sure to include the number of logical
   processors in your arithmetic if your system supports
   hyper-threading. Adding more threads for rasterization increases the
   memory demands of Stereo Pipeline. If your system is memory limited,
   it might be best to lower the ``default_num_threads`` option.

``write_pool_size`` (default=21)
   The ``write_pool_size`` option represents the max waiting pool size
   of tiles waiting to be written to disk. Most file formats do not
   allow tiles to be written arbitrarily out of order. Most however
   will let rows of tiles to be written out of order, while tiles
   inside a row must be written in order. Because of the previous
   constraint, after a tile is rasterized it might spend some time
   waiting in the write pool before it can be written to disk. If
   the write pool fills up, only the next tile in order can be
   rasterized. That makes Stereo Pipeline perform like it is only
   using a single processor.

   Increasing the ``write_pool_size`` makes Stereo Pipeline more able to
   use all processing cores in the system. Having this value too large
   can mean excessive use of memory as it must keep more portions of the
   image around in memory while they wait to be written. This number
   should be larger than the number of threads, perhaps by about 20.

``system_cache_size`` (default=1073741824)
   Accessing a file from the hard drive can be very slow. It is
   especially bad if an application needs to make multiple passes over
   an input file. To increase performance, Vision Workbench will
   usually leave an input file stored in memory for quick access. This
   file storage is known as the 'system cache' and its max size is
   dictated by ``system_cache_size``. The default value is 1 GB.

   Setting this value too high can cause your application to crash. It
   is usually recommend to keep this value around 1/4 of the maximum
   available memory on the system. The units of this property is in
   bytes.

   All tools shipped with ASP have the option ``--cache-size-mb`` to
   override the value of ``system_cache_size``. Its default value is
   1024 MB (1 GB).
 
   The recommendations for these values are based on use of the block
   matching algorithm in ASP. When using memory intensive algorithms
   such as SGM you may wish to lower some of these values (such as the
   cache size) to leave more memory available for the algorithm to use.

.. _logging:

Logging settings
~~~~~~~~~~~~~~~~

The messages displayed in the console by Stereo Pipeline are grouped
into several namespaces, and by level of verbosity. An example of
customizing Stereo Pipeline's output is given in the ``.vwrc`` file
shown above.

Several of the tools in Stereo Pipeline, including ``parallel_stereo``,
automatically append the information displayed in the console to a log
file in the current output directory. These logs contain in addition
some data about your system and settings, which may be helpful in
resolving problems with the tools (:numref:`outputfiles`).

It is also possible to specify a global log file to which all tools will
append to, as illustrated in ``.vwrc``.
