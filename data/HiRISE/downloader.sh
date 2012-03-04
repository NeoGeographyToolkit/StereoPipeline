#!/bin/bash

orbit_prefix=${1:4:4}
obsv_prefix=${1:0:15}
wget -O $1 http://hirise-pds.lpl.arizona.edu/PDS/EDR/PSP/ORB_${orbit_prefix}00_${orbit_prefix}99/$obsv_prefix/$1