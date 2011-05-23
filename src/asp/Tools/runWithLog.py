#!/usr/bin/env python
# __BEGIN_LICENSE__
# Copyright (C) 2006-2011 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


import sys
import datetime
import os

if len(sys.argv) != 3:
    raise Exception('usage: %s <dir> "<cmd> [arg1] [arg2] ..."' % sys.argv[0])

logDir = sys.argv[1]
fullCmd = sys.argv[2]

timeStr = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
cmdArgv = fullCmd.split(' ')
cmdShort = os.path.splitext(os.path.basename(cmdArgv[0]))[0]
logFname = '%s/%s_%s.txt' % (logDir, timeStr, cmdShort)
print >>sys.stderr, '%s: logging to %s' % (sys.argv[0], logFname)

# open log
if not os.path.exists(logDir):
    os.system('mkdir -p %s' % logDir)
logFile = file(logFname, 'w')

# close stdin
devNull = file('/dev/null', 'rw')
os.dup2(devNull.fileno(), 0)

# redirect stdout and stderr to log
os.dup2(logFile.fileno(), 1)
os.dup2(logFile.fileno(), 2)

quotedArgs = ['"%s"' % arg for arg in cmdArgv]
print >>sys.stderr, '%s: running: %s' % (sys.argv[0], ' '.join(quotedArgs))

# run desired command
os.execvp(cmdArgv[0], cmdArgv)
