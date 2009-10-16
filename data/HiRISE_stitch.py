#!/usr/bin/python

import os, glob, subprocess, sys;

# Note: Feed this one IMG file. This expects to find other 19 IMG
# files in the same directory. Apply this only from the directory of
# the file.

job_pool = [];
num_working_threads = 8;

def add_job( cmd ):
    if ( len(job_pool) >= num_working_threads):
        job_pool[0].wait();
        job_pool.pop(0);
    print cmd;
    job_pool.append( subprocess.Popen(cmd, shell=True) );

def wait_on_all_jobs():
    print "Waiting for jobs to finish";
    while len(job_pool) > 0:
        job_pool[0].wait();
        job_pool.pop(0);

def read_flatfile( flat ):
    f = open(flat,'r')
    averages = [0.0,0.0]
    for line in f:
        if ( line.rfind("Average Sample Offset:") > 0 ):
            index = line.rfind("Offset:");
            index_e = line.rfind("StdDev:");
            crop = line[index+7:index_e];
            averages[0] = float(crop);
        elif ( line.rfind("Average Line Offset:") > 0 ):
            index = line.rfind("Offset:");
            index_e = line.rfind("StdDev:");
            crop = line[index+7:index_e];
            averages[1] = float(crop);
    return averages;

#----------------------------

if (len(sys.argv) != 2 ):
    print "Input error:";
    print "Usage:";
    print "    " + sys.argv[0] + " ESP_011595_1725_RED0_0.IMG";
    sys.exit();

right_index = sys.argv[1].rfind("_");
prefix = sys.argv[1][:right_index-1];

# Step 1 - hi2isis
for i in range(10):
    cmd = "hi2isis from="+prefix+str(i)+"_0.IMG to="+prefix+str(i)+"_0.cub"
    add_job(cmd);
    cmd = "hi2isis from="+prefix+str(i)+"_1.IMG to="+prefix+str(i)+"_1.cub"
    add_job(cmd);
wait_on_all_jobs();

# Step 2 - spiceinit
for i in range(10):
    cmd = "spiceinit from="+prefix+str(i)+"_0.cub";
    add_job(cmd);
    cmd = "spiceinit from="+prefix+str(i)+"_1.cub";
    add_job(cmd);
wait_on_all_jobs();

# Step 3 - hical
for i in range(10):
    cmd = "hical from="+prefix+str(i)+"_0.cub to="+prefix+str(i)+"_0.cal.cub";
    add_job(cmd);
    cmd = "hical from="+prefix+str(i)+"_1.cub to="+prefix+str(i)+"_1.cal.cub";
    add_job(cmd);
wait_on_all_jobs();

# Step 4 - histitch
for i in range(10):
    cmd = "histitch from1="+prefix+str(i)+"_0.cal.cub from2="+prefix+str(i)+"_1.cal.cub to="+prefix+str(i)+".cub";
    add_job(cmd);
wait_on_all_jobs();

# Step 5 - cubenorm
for i in range(10):
    cmd = "cubenorm from="+prefix+str(i)+".cub to="+prefix+str(i)+".norm.cub";
    add_job(cmd);
wait_on_all_jobs();

# Step 6 - spiceinit & spicefit for good luck in noproj
for i in range(10):
    cmd = "spiceinit from="+prefix+str(i)+".norm.cub";
    add_job(cmd);
wait_on_all_jobs();
for i in range(10):
    cmd = "spicefit from="+prefix+str(i)+".norm.cub";
    add_job(cmd);
wait_on_all_jobs();

# Step 7 - noproj - must be done sequentially
for i in range(10):
    cmd = "noproj from="+prefix+str(i)+".norm.cub match="+prefix+"5.norm.cub to="+prefix+str(i)+".noproj.cub source=frommatch";
    os.system(cmd);

# Step 8 - hijitreg
for i in range(9):
    j = i + 1;
    cmd = "hijitreg from="+prefix+str(i)+".noproj.cub match="+prefix+str(j)+".noproj.cub flatfile=flat_"+str(i)+"_"+str(j)+".txt";
    add_job(cmd);
wait_on_all_jobs();

# Step 9 - Read averages
average_sample = [];
average_line = [];
for i in range(9):
    flat_file = "flat_"+str(i)+"_"+str(i+1)+".txt";
    averages = read_flatfile( flat_file );
    average_sample.append(averages[0]);
    average_line.append(averages[1]);

# Step 10 - Handmos
os.system("cp "+prefix+"9.noproj.cub "+prefix+"mosaic.cub");
for i in range(9):
    j = 8-i;
    sample_sum = 0.0;
    line_sum = 0.0;
    for s in range(i+1):
        sample_sum += average_sample[8-s];
        line_sum += average_line[8-s];
    cmd = "handmos from="+prefix+str(j)+".noproj.cub mosaic="+prefix+"mosaic.cub outsample="+str(int(round(sample_sum)))+" outline="+str(int(round(line_sum)))+" outband=1";
    os.system(cmd);

# A little clean up
os.system("rm "+prefix+"?_?.c*b "+prefix+"?.cub" );
os.system("ls "+prefix+"?.norm.cub > fromlist" );
os.system("rm flat_?_?.txt");
os.system("hiccdstitch fromlist=fromlist shiftdef=hijit_stitch.def to="+prefix+"mosaic.cub");

# Step 11 - cubenorm mosaic
os.system("cubenorm from="+prefix+"mosaic.cub to="+prefix+"mosaic.norm.cub");
os.system("rm "+prefix+"mosaic.cub");
os.system("rm "+prefix+"?.norm.cub");
os.system("rm fromlist");

print "Finished"
