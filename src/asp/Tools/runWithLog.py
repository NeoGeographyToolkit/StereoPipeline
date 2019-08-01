#!/usr/bin/env python
# __BEGIN_LICENSE__
#  Copyright (c) 2009-2013, United States Government as represented by the
#  Administrator of the National Aeronautics and Space Administration. All
#  rights reserved.
#
#  The NGT platform is licensed under the Apache License, Version 2.0 (the
#  "License"); you may not use this file except in compliance with the
#  License. You may obtain a copy of the License at
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# __END_LICENSE__

from __future__ import print_function
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
print('%s: logging to %s' % (sys.argv[0], logFname), file=sys.stderr)

# open log
if not os.path.exists(logDir):
    os.system('mkdir -p %s' % logDir)
logFile = open(logFname, 'w')

# close stdin
devNull = open('/dev/null', 'rw')
os.dup2(devNull.fileno(), 0)

# redirect stdout and stderr to log
os.dup2(logFile.fileno(), 1)
os.dup2(logFile.fileno(), 2)

quotedArgs = ['"%s"' % arg for arg in cmdArgv]
print('%s: running: %s' % (sys.argv[0], ' '.join(quotedArgs)), file=sys.stderr)

# run desired command
os.execvp(cmdArgv[0], cmdArgv)
