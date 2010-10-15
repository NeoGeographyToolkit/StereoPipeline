
=========================================
PhotometryTK -- Usage Notes
=========================================

.. sectnum::

.. contents:: Contents

Usage
~~~~~

Set up your environment variables, for example:

::

  export PATH="$HOME/projects/VisionWorkbench/build/x86_64_linux_gcc4.1/bin:$PATH"
  export PATH="$HOME/projects/StereoPipeline/build/x86_64_linux_gcc4.1/bin:$PATH"
  export DATA="$HOME/projects/StereoPipeline/reconstructTest/data/orbit33"
  export DRG="$DATA/DRG_sub4"
  export DEM="$DATA/DEM_sub4"
  export PLATES="$DATA/plates"

Run the following to check if the RabbitMQ server is running:

:: 

  ps auxww | grep rabbitmq_server

If it is not running, bug Nick.  The server is run as root and you cannot start it yourself on
lunokhod1.

Run the following to init and solve:

::

  # In terminal 2: (cd $PLATES && index_server .)
  cd $DEM
  ls *DEM.tif | xargs -n 1 -P 10 grassfirealpha --nodata -10000
  ls *grass.tif | xargs -n 1 -P 10 image2plate -o pf://index/DEM_blend.plate --file tif -m equi
  for i in {0..8}; do platereduce --end_t 1999 -t 2000 pf://index/DEM_blend.plate -l $i; done
  plate2plate -o pf://index/DEM.plate -i pf://index/DEM_blend.plate --filter identity --bottom 10 --skim
  # In terminal 2: kill the index server
  rm -rf $PLATES/DEM_blend.plate
  # In terminal 2: (cd $PLATES && index_server .)
  phoinitfile project
  # In terminal 3: (cd $PLATES && ptk_server .)
  echo $DEM/*.tif | xargs -n1 echo | xargs -n1 -P10 phodrg2plate pf://ptk/project.ptk
  echo {0..9} | xargs -n1 echo | xargs -n1 -P10 -I{} phoitalbedo -j {} -n 10 pf://ptk/project.ptk
  phosolve.py -i 1 -l 8 pf://ptk/project.ptk

Something for us to look at: phosolve.py uses the ``mipmap`` utility
which is single-threaded.  Zack has a multi-threaded version called
``pmipmap.py`` which may be better.
