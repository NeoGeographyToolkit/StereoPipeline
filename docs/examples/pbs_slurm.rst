.. _pbs_slurm:

Using PBS and SLURM
-------------------

Running ``parallel_stereo`` can be very computationally expensive, so
often it is launched on high-performance multi-machine systems. Here
it will be shown how to run this program on a *Portable Batch System*
(*PBS*) setup, such as NASA Pleiades, and on a *Simple Linux Utility
for Resource Management* (*SLURM*) system.

In either of these, it is assumed that all compute nodes share storage
space and are able communicate with ssh without password.

On a PBS system, one can have a script as follows::

    #!/bin/bash

    # Change to current directory
    cd $PBS_O_WORKDIR

    # Set the path to ASP tools 
    export PATH=/path/to/ASP/bin:$PATH

    # Run parallel_stereo
    parallel_stereo --stereo-algorithm asp_mgm \
      --processes 4 --subpixel-mode 3 -t rpc   \
      --nodes-list $PBS_NODEFILE               \
      left.tif right.tif left.xml right.xml    \
      run/run

    # Run point2dem
    point2dem --auto-proj-center run/run-PC.tif 

Note the two special environmental variables ``PBS_O_WORKDIR`` and ``PBS_NODEFILE``
which refer to the current work directory in which the script is started, and the
list of nodes allocated for the job.

Ensure the option ``--nodes-list`` is set, otherwise only the head node
will be used.
  
This script, named for example, ``run.sh``, can be launched as::

    qsub -m n -r n -N jobName -l walltime=12:00:00           \
       -W group_list=yourGroup -j oe -S /bin/bash            \
       -l select=8:ncpus=20:model=ivy -- $(pwd)/run.sh <args>

Additional arguments can be passed in on this line to ``run.sh``,
which can be accessed from within that script as ``$1``, ``$2``, etc.,
per bash shell conventions.

It is strongly suggested to learn what each of the above options does
and adjust them for your needs.

With SLURM, a script as follows can work::

    #!/bin/bash
    
    #SBATCH --job-name=asp
    #SBATCH --output=asp.log
    #SBATCH --nodes=4
    #SBATCH --ntasks-per-node=36
    #SBATCH --time=50:00:00
    #SBATCH --partition=queue1
    
    # Change to the directory in which the job was submitted
    cd $SLURM_SUBMIT_DIR
 
    # Create a temporary list of nodes in current directory
    nodesList=$(mktemp -p $(pwd))

    # Set up the nodes list
    scontrol show hostname $SLURM_NODELIST | tr ' ' '\n' > $nodesList
    
    # Run parallel_stereo. (Ensure that this program is in the path.)
    parallel_stereo --nodes-list $nodesList  \
      --processes 4                          \
      --parallel-options '--sshdelay 0.1'    \
      <other ASP options> 

   # Delete the temporary list of nodes
   /bin/rm -fv $nodesList
 
As before, the options and values above should be adjusted for your needs.

Ensure the option ``--nodes-list`` is set, otherwise only the head node
will be used.

If your SLURM setup requires a custom ssh port, set in the list of nodes
the full ssh command to each node, rather than the node name. Example::

  ssh -p port1 node1
  ssh -p port2 node2

