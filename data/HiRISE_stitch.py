#!/usr/bin/python

import os, glob, subprocess, sys, string;

# Note: Feed this one IMG file. This expects to find other 19 IMG
# files in the same directory. Apply this only from the directory of
# the file.

job_pool = [];
num_working_threads = 4;

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

def is_post_isis_3_1_20():
    # Weird method to find to location of ISIS
    p = subprocess.Popen("echo $ISISROOT", shell=True, stdout=subprocess.PIPE);
    path = p.stdout.read();
    path = path.strip()

    print path;
    f = open(path+"/inc/Constants.h",'r');
    for line in f:
        if ( line.rfind("std::string version(") > 0 ):
            index = line.find("\"3");
            index_e = line.rfind("|");
            version = line[index+1:index_e];
            print "\tFound Isis Version: "+version;
            index = version.rfind(".");
            if ( int(version[index+1:].strip(string.letters+string.whitespace)) > 20 ):
                return True;
            else:
                return False;
    return False;

#----------------------------

if (len(sys.argv) != 2 ):
    print "Input error:";
    print "Usage:";
    print "    " + sys.argv[0] + " ESP_011595_1725_RED0_0.IMG";
    sys.exit();

right_index = sys.argv[1].rfind("_");
prefix = sys.argv[1][:right_index-1];

# Prestep 1 - Determine Isis Version
post_isis_20 = is_post_isis_3_1_20();

# Prestep 2 - Determine the number range of files
list_of_files = glob.glob(prefix+"*.IMG");
ccd_min = 10;
ccd_max = 0;
for line in list_of_files:
    index = line.find("RED");
    ccd_number = int(line[index+3:index+4]);
    if ( ccd_number > ccd_max ):
        ccd_max = ccd_number;
    if ( ccd_number < ccd_min ):
        ccd_min = ccd_number;
print "\tMin CCD index: "+str(ccd_min);
print "\tMax CCD index: "+str(ccd_max);
print;

# Step 1 - hi2isis
for i in range(ccd_min,ccd_max+1):
    cmd = "hi2isis from="+prefix+str(i)+"_0.IMG to="+prefix+str(i)+"_0.cub"
    add_job(cmd);
    cmd = "hi2isis from="+prefix+str(i)+"_1.IMG to="+prefix+str(i)+"_1.cub"
    add_job(cmd);
wait_on_all_jobs();

# Step 2 - spiceinit
for i in range(ccd_min,ccd_max+1):
    cmd = "spiceinit from="+prefix+str(i)+"_0.cub";
    add_job(cmd);
    cmd = "spiceinit from="+prefix+str(i)+"_1.cub";
    add_job(cmd);
wait_on_all_jobs();

# Step 3 - hical
for i in range(ccd_min,ccd_max+1):
    cmd = "hical from="+prefix+str(i)+"_0.cub to="+prefix+str(i)+"_0.cal.cub";
    add_job(cmd);
    cmd = "hical from="+prefix+str(i)+"_1.cub to="+prefix+str(i)+"_1.cal.cub";
    add_job(cmd);
wait_on_all_jobs();
os.system("rm "+prefix+"?_?.cub");

# Step 4 - histitch
for i in range(ccd_min,ccd_max+1):
    cmd = "histitch from1="+prefix+str(i)+"_0.cal.cub from2="+prefix+str(i)+"_1.cal.cub to="+prefix+str(i)+".cub";
    add_job(cmd);
wait_on_all_jobs();
os.system("rm "+prefix+"?_?.cal.cub");

# Step 5 - cubenorm
for i in range(ccd_min,ccd_max+1):
    cmd = "cubenorm from="+prefix+str(i)+".cub to="+prefix+str(i)+".norm.cub";
    add_job(cmd);
wait_on_all_jobs();
os.system("rm "+prefix+"?.cub");

# Step 6 - spiceinit & spicefit
for i in range(ccd_min,ccd_max+1):
    cmd = "spiceinit from="+prefix+str(i)+".norm.cub";
    add_job(cmd);
wait_on_all_jobs();
for i in range(ccd_min,ccd_max+1):
    cmd = "spicefit from="+prefix+str(i)+".norm.cub";
    add_job(cmd);
wait_on_all_jobs();

# Step 7 - noproj - must be done sequentially
for i in range(ccd_min,ccd_max+1):
    cmd = "noproj from="+prefix+str(i)+".norm.cub match="+prefix+"5.norm.cub to="+prefix+str(i)+".noproj.cub source=frommatch";
    os.system(cmd);
os.system("rm "+prefix+"?.norm.cub")

# Step 8 - hijitreg
for i in range(ccd_min,ccd_max):
    j = i + 1;
    cmd = "hijitreg from="+prefix+str(i)+".noproj.cub match="+prefix+str(j)+".noproj.cub flatfile=flat_"+str(i)+"_"+str(j)+".txt";
    add_job(cmd);
wait_on_all_jobs();

# Step 9 - Read averages
average_sample = [];
average_line = [];
for i in range(ccd_min,ccd_max):
    flat_file = "flat_"+str(i)+"_"+str(i+1)+".txt";
    averages = read_flatfile( flat_file );
    average_sample.append(averages[0]);
    average_line.append(averages[1]);

# Step 10 - Handmos
os.system("cp "+prefix+"5.noproj.cub "+prefix+"mosaic.cub");
for i in range(4,ccd_min-1,-1):
    sample_sum = 0.0;
    line_sum = 0.0;
    for s in range(4,i-1,-1):
        sample_sum += average_sample[s-ccd_min];
        line_sum += average_line[s-ccd_min];
    cmd = "";
    if ( post_isis_20 ):
        # ISIS 3.1.21+
        cmd = "handmos from="+prefix+str(i)+".noproj.cub mosaic="+prefix+"mosaic.cub outsample="+str(int(round(sample_sum)))+" outline="+str(int(round(line_sum)))+" outband=1 priority=beneath";
    else:
        # ISIS 3.1.20-
        cmd = "handmos from="+prefix+str(i)+".noproj.cub mosaic="+prefix+"mosaic.cub outsample="+str(int(round(sample_sum)))+" outline="+str(int(round(line_sum)))+" outband=1 input=beneath";
    os.system(cmd);

for i in range(6,ccd_max+1,1):
    sample_sum = 0.0;
    line_sum = 0.0;
    for s in range(5,i,1):
        sample_sum -= average_sample[s-ccd_min];
        line_sum -= average_line[s-ccd_min];
    cmd = "";
    if ( post_isis_20 ):
        # ISIS 3.1.21+
        cmd = "handmos from="+prefix+str(i)+".noproj.cub mosaic="+prefix+"mosaic.cub outsample="+str(int(round(sample_sum)))+" outline="+str(int(round(line_sum)))+" outband=1 priority=beneath";
    else:
        # ISIS 3.1.20-
       cmd = "handmos from="+prefix+str(i)+".noproj.cub mosaic="+prefix+"mosaic.cub outsample="+str(int(round(sample_sum)))+" outline="+str(int(round(line_sum)))+" outband=1 input=beneath"; 
    os.system(cmd);

# Clean up
os.system("rm "+prefix+"?.noproj.cub" );
os.system("rm flat_?_?.txt");

# Step 11 - cubenorm mosaic
os.system("cubenorm from="+prefix+"mosaic.cub to="+prefix+"mosaic.norm.cub");
os.system("rm "+prefix+"mosaic.cub");

print "Finished"
