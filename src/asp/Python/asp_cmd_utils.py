#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

# TODO: Move this to asp_system_utils

"""General systems related utilities"""
from __future__ import print_function
import sys, os, re, shutil, subprocess, string, time, errno, multiprocessing, shlex

def wipe_option(options, opt, n):
    # In the array 'options', find the entry with value 'opt'.
    # Wipe this entry and the next n values.
    while opt in options:
        r = options.index(opt)
        if r < len(options):
            del options[r] # rm 'opt'
        for i in range(n):
            if r < len(options): del options[r]

class CmdRunException(Exception):
    '''Exception type indicating an error with a cmd call'''
    pass

def isCmdOption(arg):
    """Returns True if the string is a command line option,
    False otherwise (if it is an argument)"""

    # An option must start with '-' and not consist of all numbers
    if ( arg.startswith('-') and not re.match(r'^-[0-9.]+$', arg) ):
        return True
    else:
        return False

