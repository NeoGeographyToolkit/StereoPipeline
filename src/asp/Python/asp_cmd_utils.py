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

def replace_opt_val(argv, opt, old_val, new_val):
    # In the array 'options', find the entry with value 'opt'.
    # If the value that follows is 'old_val', replace it with 'new_val'.
    
    # Find the index of opt in argv
    try:
        r = argv.index(opt)
    except:
        return argv
    
    # There must be another value after the option
    if r+1 >= len(argv):
        raise CmdRunException("Option " + opt + " has no value")
  
    # Replace the value
    if argv[r+1] == str(old_val):
        argv[r+1] = str(new_val)
    
    return argv

def option_val(argv, opt):
  # Find the value that follows the option 'opt' in the array 'argv'.
  # If the value is not found, return None.
  try:
    r = argv.index(opt)
  except:
    return None
  
  # There must be another value after the option
  if r+1 >= len(argv):
     return None
    
  # Return the value
  return argv[r+1]   
  
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

# Utilities to ensure that the Python parser does not garble negative
# values such as '-365' into '-3'.
escapeStr='esc_rand_str'
def escape_vals(vals):
    for index, val in enumerate(vals):
        p = re.match(r"^-[\.\d]", val)
        if p:
            vals[index] = escapeStr + val
    return vals

def unescape_vals(vals):
    for index, val in enumerate(vals):
        p = re.match(r"^" + escapeStr + "(-.*?)$", val)
        if p:
            vals[index] = p.group(1)
    return vals

def clean_args(args):
    '''Fix various problems that can happen in the input args'''
    args = unescape_vals(args)
    argsout = []
    return args

def get_option(options, opt, n):
    # In the array 'options', find and return the entry with value 'opt'
    #  and the next n values.
    output = []
    r = options.index(opt)
    if r < len(options):
        output.append(options[r])
    for i in range(1,n+1):
        if r+i < len(options):
            output.append(options[r+i])
    return output

def set_option(options, opt, new_values):
    '''In the array 'options', find the entry with value 'opt'.
    Replace the next values with new_values.'''

    if opt in options:
        # The option is already included, update its value.
        r = options.index(opt)
        if r < len(options):
            r += 1
            for i in new_values:
                if r < len(options):
                  options[r] = str(i)
                r += 1
    else: # The option is not present, add it.
        options.append(opt)
        for i in new_values:
            options.append(str(i))

