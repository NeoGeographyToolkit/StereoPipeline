#!/bin/bash

# Simple shell script for running command line calls

echo $PATH

# Grab the only input argument
TOOL=$1
COMMAND=$2

echo $TOOL
echo $COMMAND

# TODO: Verify that the log file goes to the correct place

# Call the function
$TOOL $COMMAND




