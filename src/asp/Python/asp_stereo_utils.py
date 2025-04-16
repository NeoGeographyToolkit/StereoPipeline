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

# TODO(oalexan1): Move this to src/asp/Python and distribute most of
# these functions across the files in there depending on what these
# functions do.

from __future__ import print_function
import sys, optparse, subprocess, re, os, time, glob
import os.path as P

from asp_system_utils import *
from asp_alg_utils import *

import asp_system_utils, asp_string_utils
asp_system_utils.verify_python_version_is_supported()

# For consistency with C++
VW_CORRELATION_BM        = '0'
VW_CORRELATION_SGM       = '1'
VW_CORRELATION_MGM       = '2'
VW_CORRELATION_FINAL_MGM = '3'
VW_CORRELATION_OTHER     = '4'

def stereo_alg_to_num(alg):
    '''Convert, for example, 'asp_mgm' to '2'. This function has a C++
    analog in stereo.cc.'''

    alg = alg.lower()
    
    if alg.startswith('0') or alg.startswith('asp_bm'):
        return VW_CORRELATION_BM
    
    if alg.startswith('1') or alg.startswith('asp_sgm'):
        return VW_CORRELATION_SGM

    if alg.startswith('2') or alg.startswith('asp_mgm'):
        return VW_CORRELATION_MGM

    if alg.startswith('3') or alg.startswith('asp_final_mgm'):
        return VW_CORRELATION_FINAL_MGM

    # Use an external algorithm
    return VW_CORRELATION_OTHER

#===================================================================================
# Some of these functions is somewhat generic and could be moved.

# TODO(oalexan1): Move this to asp_system_utils.
def get_asp_version():
    '''Returns the current ASP version number'''
    prog = asp_system_utils.libexec_path("stereo_parse") # get the full path
    return asp_system_utils.get_prog_version(prog)

class Step:
    # The ids of individual stereo steps
    pprc  = 0
    corr  = 1
    blend = 2
    rfne  = 3
    fltr  = 4
    tri   = 5

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

# TODO(oalexan1): Move to asp_cmd_utils.py
# TODO(oalexan1): Run parallel_bundle_adjust with no options and see this fail.
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

# TODO(oalexan1): Move to asp_cmd_utils.py
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

# This is a bugfix for OpenBLAS on the Mac. It cannot handle
# too many threads.
def reduce_num_threads_in_pprc(cmd):

    prog = cmd[0]
    if os.path.basename(prog) == 'stereo_pprc' and sys.platform == 'darwin':

        os.environ['OPENBLAS_NUM_THREADS'] = '1'

        num_threads_arr = get_option(cmd, '--threads', 1)
        if len(num_threads_arr) > 1:
            num_threads = int(num_threads_arr[1])
        else:
            num_threads = 1

        num_threads = min(num_threads, get_num_cpus())
        num_threads = min(num_threads, 4)
        num_threads = max(num_threads, 1)

        set_option(cmd, '--threads', [num_threads])

    return cmd

#=====================================================================================
# Functions below here are purely stereo

# Run one of the stereo executables
def stereo_run(prog, args, opt, **kw):
    binpath = bin_path(prog)
    call = [binpath]
    call.extend(args)

    call = reduce_num_threads_in_pprc(call)

    # On Linux estimate the memory usage and runtime
    if 'linux' in sys.platform and prog == 'stereo_corr' and \
           hasattr(opt, 'mem_usage') and opt.mem_usage:
        call = ['/usr/bin/time',  '-f', '"elapsed=%E mem=%M (kb) time_stereo_corr"'] + call
        print(" ".join(call))
        
    if opt.dryrun or opt.verbose:
        print(" ".join(call))
        
    if opt.dryrun: return
    try:
        t_start = time.time()
        code = subprocess.call(call)
        if opt.verbose:
            wall_s = time.time() - t_start
            print('Wall time (s): {0:.1f}\n'.format(wall_s))
    except OSError as e:
        raise Exception('%s: %s' % (binpath, e))
    if code != 0:
        raise Exception('Stereo step ' + kw['msg'] + ' failed')

def run_sparse_disp(args, opt):

    settings   = run_and_parse_output( "stereo_parse", args, ",", opt.verbose )
    left_img   = settings["trans_left_image"]
    right_img  = settings["trans_right_image"]
    out_prefix = settings["out_prefix"]

    sparse_args = left_img + right_img + out_prefix + ['--nodata-value', str(0)]
    if opt.sparse_disp_options is not None:
        sparse_args += opt.sparse_disp_options.split()

    # Pass the number of threads to sparse_disp
    # sparse_disp_options should trump
    if not any('--processes' in s for s in sparse_args):
        num_threads = 0
        if hasattr(opt, 'threads') and opt.threads is not None and opt.threads > 0:
            num_threads = opt.threads
        if hasattr(opt, 'threads_single')       and \
               (opt.threads_single is not None) and \
               (opt.threads_single > 0):
            num_threads = opt.threads_single
        if num_threads > 0:
            sparse_args += ['--processes', str(num_threads)]

    # If the user set ASP_PYTHON_MODULES_PATH using the custom ASP python installation script,
    #  set the LD_LIBRARY_PATH and PYTHON_PATH accordingly.  Otherwise the user's default
    #  Python installation will be used.
    if os.environ.get('ASP_PYTHON_MODULES_PATH') is not None:
        if os.environ.get('LD_LIBRARY_PATH') is None:
            os.environ['LD_LIBRARY_PATH'] = os.environ.get('ASP_PYTHON_MODULES_PATH')
        else:
            os.environ['LD_LIBRARY_PATH'] = os.environ.get('LD_LIBRARY_PATH') + \
                                            ":" + os.environ.get('ASP_PYTHON_MODULES_PATH')
        if os.environ.get('PYTHONPATH') is None:
            os.environ['PYTHONPATH'] = os.environ.get('ASP_PYTHON_MODULES_PATH')
        else:
            os.environ['PYTHONPATH'] = os.environ.get('ASP_PYTHON_MODULES_PATH') + ":" + \
                                      os.environ.get('PYTHONPATH')

    print('Running sparse disp with arguments: ' + str(sparse_args) +'\nand options: ' + str(opt))

    stereo_run('sparse_disp', sparse_args, opt,
               msg='%d: Low-res correlation with sparse_disp' % Step.corr)

# Do low-res correlation.
def calc_lowres_disp(args, opt, sep, resume = False):

    will_run = True
    if resume:
        settings     = run_and_parse_output("stereo_parse", args, ",", opt.verbose)
        out_prefix   = settings["out_prefix"][0]
        d_sub        = out_prefix + '-D_sub.tif'
        d_sub_spread = out_prefix + '-D_sub_spread.tif'
    
        if (opt.seed_mode != 3):
            if asp_system_utils.is_valid_image(d_sub):
                will_run = False
            else:
                print("Will recreate: " + d_sub)
        else:
            if asp_system_utils.is_valid_image(d_sub) and \
                   asp_system_utils.is_valid_image(d_sub_spread):
                will_run = False
            else:
                print("Will recreate: " + d_sub + " and " + d_sub_spread)

    if will_run:
        if (opt.seed_mode == 3):
            # This uses the python sparse_disp tool
            run_sparse_disp(args, opt)
        else:
            local_args = args[:] # deep copy to use locally
            local_args.extend(['--compute-low-res-disparity-only'])
            # invoke here stereo_run to be able to see the output on screen
            stereo_run('stereo_corr', local_args, opt, msg='')

    # Attach a georef to D_sub and D_sub_spread if appropriate
    local_args = args[:] # deep copy to use locally
    local_args.append('--attach-georeference-to-lowres-disparity')
    run_and_parse_output("stereo_parse", local_args, sep, opt.verbose)

def parse_corr_seed_mode(filename):

    mode = None

    # Extract corr-seed-mode from filename.
    if not os.path.isfile(filename):
        return mode

    # Care with encoding
    fh = open(filename, 'r', encoding='utf8', errors='ignore')
    
    for line in fh:
        line = re.sub(r'\#.*?$', '', line) # wipe comments
        matches = re.match(r'^\s*corr-seed-mode\s+(\d+)', line)
        if matches:
            mode = int(matches.group(1))
    fh.close()

    return mode

def run_multiview(prog_name, args, extra_args, entry_point, stop_point,
                  verbose, settings):

    # Invoke multiview stereo processing, either using 'stereo', or
    # using 'parallel_stereo', depending on the caller of this function.

    # Doing multi-view stereo amounts to doing all the steps of stereo
    # save for triangulation for stereo pairs made up of first image
    # and another image in the sequence. Then, triangulation is done
    # using all images. The precise stereo command to use for each
    # pair was already generated by stereo_parse, and here we extract
    # it from 'settings'.

    # We must respect caller's entry and stop points.

    # Must make sure to use the same Python invoked by parent
    python_path = sys.executable

    # Run all steps but tri
    for s in sorted(settings.keys()):

        m = re.match(r'multiview_command', s)
        if not m: continue

        local_args    = settings[s][:] # the current two-image stereo command
        local_args[0] = prog_name
        local_entry   = entry_point
        local_stop    = stop_point
        if local_stop > Step.tri:
            local_stop = Step.tri
        local_args.extend(['--entry-point', str(local_entry)])
        local_args.extend(['--stop-point',  str(local_stop)])
        local_args.extend(extra_args)
        cmd = [python_path] + local_args
        # Go on even if some of the runs fail
        try:
            asp_system_utils.generic_run(cmd, verbose)
        except:
            pass

    # Run tri
    local_args  = [prog_name]
    local_args.extend(args)
    local_entry = Step.tri
    local_stop  = stop_point
    local_args.extend(['--entry-point', str(local_entry)])
    local_args.extend(['--stop-point', str(local_stop)])
    local_args.extend(extra_args)
    cmd = [python_path] + local_args
    asp_system_utils.generic_run(cmd, verbose)

    # Here we ensure that in the run directory
    # the L.tif file is present, by sym-linking
    # from the subdirectory of one of the pairs.
    # It is nice to have L.tif for when we later
    # do point2dem PC.tif --orthoimage L.tif

    # The code below amounts to:
    # ln -s out_prefix-pair1/1-L.tif out_prefix-L.tif
    out_prefix = settings['out_prefix'][0]
    sym_f      = out_prefix + '-L.tif'
    if not os.path.lexists(sym_f):
        files = glob.glob(out_prefix + '-pair*/*-L.tif')
        if len(files) > 0:
            f = files[0]
            run_dir = os.path.dirname(out_prefix)
            rel_f   = os.path.relpath(f, run_dir)
            os.symlink(rel_f, sym_f)
