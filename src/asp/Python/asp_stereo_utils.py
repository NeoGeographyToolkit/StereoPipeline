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

# TODO(oalexan1): Distribute some of the more low-level functions from here to 
# other modules such as asp_system_utils, etc.

from __future__ import print_function
import sys, optparse, subprocess, re, os, time, glob
import fcntl # for file locking
import os.path as P

from asp_system_utils import *
from asp_alg_utils import *

import asp_system_utils, asp_string_utils, asp_cmd_utils
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

class Step:
    # The ids of individual stereo steps
    pprc  = 0
    corr  = 1
    blend = 2
    rfne  = 3
    fltr  = 4
    tri   = 5
    clean = 6

def stereoProgName(step):
    '''
    Return the stereo program name for a given step.
    '''
    if step == Step.pprc:
        return 'stereo_pprc'
    elif step == Step.corr:
        return 'stereo_corr'
    elif step == Step.blend:
        return 'stereo_blend'
    elif step == Step.rfne:
        return 'stereo_rfne'
    elif step == Step.fltr:
        return 'stereo_fltr'
    elif step == Step.tri:
        return 'stereo_tri'
    elif step == Step.clean:
        # Add this for convenience, when querying the latest stage in the status file
        return 'stereo_parse'
    else:
        return 'stereo_unknown'

# This is a bugfix for OpenBLAS on the Mac. It cannot handle
# too many threads. Never use more than 128 threads in either
# case, as openblas can't  handle it.
def reduce_num_threads(cmd):

    # This is only needed if the user specified --threads
    if '--threads' not in cmd:
        return cmd

    num_threads_arr = asp_cmd_utils.get_option(cmd, '--threads', 1)
    if len(num_threads_arr) > 1:
        num_threads = int(num_threads_arr[1])
    else:
        num_threads = 1

    prog = cmd[0]
    if os.path.basename(prog) == 'stereo_pprc' and sys.platform == 'darwin':
        os.environ['OPENBLAS_NUM_THREADS'] = '1'
        num_threads = min(num_threads, get_num_cpus())
        num_threads = min(num_threads, 4)

    num_threads = min(num_threads, 128)
    num_threads = max(num_threads, 1)
    asp_cmd_utils.set_option(cmd, '--threads', [num_threads])

    return cmd

def stereo_run(prog, args, opt, **kw):
    '''Run a stereo executable. Output is shown on screen as it happens.
       Optional kw args: extra_args (list), msg (string for error message).'''
    extra_args = kw.get('extra_args', [])
    msg = kw.get('msg', prog)

    binpath = bin_path(prog)
    call = [binpath] + args + extra_args

    call = reduce_num_threads(call)

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
        raise Exception('Stereo step ' + msg + ' failed')

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

def stereoTilesIndex(out_prefix):
    '''
    Form the index of tiles.
    '''
    return out_prefix + "-tiles-index.txt"

def stereoStatusFile(out_prefix):
    '''
    The file having the status of how many tiles were successfully processed.
    '''
    return out_prefix + "-stereo-status.txt"


def numTiles(out_prefix):
    '''
    Return the total number of tiles for current stage of processing.
    '''
    
    tiles_index = stereoTilesIndex(out_prefix)
    
    num_tiles = 0
    if not os.path.exists(tiles_index):
        return num_tiles
        
    # Read this file and count how many lines
    with open(tiles_index, 'r') as f:
        lines = f.readlines()
        num_tiles = len(lines)
    return num_tiles
    
            
def updateNumDoneTiles(out_prefix, latest_stage_name, reset):
    '''
    Update the number of done tiles for current stage of processing. Care is taken
    that multiple processes can both update the same status file without conflicts.
    '''

    num_tiles = numTiles(out_prefix)
    status_file = stereoStatusFile(out_prefix)
    
    if latest_stage_name in ['stereo_pprc', 'stereo_fltr']:
        # There is no tiling for these stages
        num_tiles = 1
        
    # If this is a sym link, wipe it. Symlinks can happen if resuming from a prev run.
    if os.path.islink(status_file):
        os.unlink(status_file)

    # Ensure file exists so r+ mode does not fail
    if not os.path.exists(status_file):
        with open(status_file, 'a') as f:
            pass

    # Open handle for both reading and writing
    with open(status_file, 'r+') as f:
        # Get exclusive lock to prevent race conditions
        # lockf is used for better compatibility with network storage
        fcntl.lockf(f, fcntl.LOCK_EX)
        try:
            # read current status
            lines = f.readlines()
            stage_name = ""
            num_done = 0
            
            if len(lines) > 1:
                # First line is a comment, second one has the data
                statusParts = lines[1].strip().split(' ')
                if len(statusParts) > 1:
                    stage_name = statusParts[0]
                    num_done = int(statusParts[1])

            # update logic
            if stage_name != latest_stage_name or reset:
                num_done = 0
                stage_name = latest_stage_name
            
            if not reset:
                num_done += 1

            # Write new status
            f.seek(0) # return to start of file
            f.truncate() # wipe old data
            f.write("# processingStep numDoneTiles numTotalTiles\n")
            f.write(f"{stage_name} {num_done} {num_tiles}\n")
            
            # Ensure data reaches the disk/server before unlocking
            f.flush()
            os.fsync(f.fileno()) 
            
            # It is helpful to print this info, but not for pprc, etc.
            if num_tiles > 1:
                print(f"Processing status: {stage_name} {num_done} / {num_tiles} tiles")

        finally:
            # release the lock
            fcntl.lockf(f, fcntl.LOCK_UN)

def produce_tiles(opt, args, settings, tile_w, tile_h):
    '''
    Generate a list of bounding boxes for each output tile. Use stereo_parse for
    that. Skip the tiles that have no valid low-res disparity (D_sub.tif).
    '''
    
    # This cannot happen before we know trans_left_image_size
    key = 'trans_left_image_size'
    if key not in settings:
        raise Exception("Cannot produce tiles until L.tif is known.")
        
    sep = ","
    tile_opt = ['--parallel-tile-size', str(tile_w), str(tile_h)]
    verbose = False
    num_pairs = int(settings['num_stereo_pairs'][0]) # for multiview
    
    if opt.seed_mode != 0 and num_pairs == 1:
      # D_sub must exist
      d_sub_file = settings['out_prefix'][0] + '-D_sub.tif'
      if not os.path.exists(d_sub_file):
        print("WARNING: The file " + d_sub_file + " does not exist. " + \
              "Will not be able to exclude tiles with no data.")
    
    # Create the tiles with stereo_parse This can handle D_sub.
    run_and_parse_output("stereo_parse", args + tile_opt, sep, verbose)
    tiles = readTiles(settings['out_prefix'][0])
    return tiles

def tile_dir(prefix, tile):
    return prefix + '-' + tile.name_str()

def use_padded_tiles(settings):
    '''
    Tiles with padding are needed for local epipolar alignment and for non-asp_bm 
    stereo algorithms.
    '''
    
    alg = stereo_alg_to_num(settings['stereo_algorithm'][0])
    return (alg > VW_CORRELATION_BM or settings['alignment_method'][0] == 'local_epipolar')

def grow_crop_tile_maybe(settings, prog, tile):
    '''
    The tile, perhaps with padding added, and perhaps cropped to
    desired region, on which to run one of the stereo tools.
    '''

    # Make a local copy of the tile, as we may grow it
    local_tile = BBox(tile.x, tile.y, tile.width, tile.height)
    if use_padded_tiles(settings) and prog == 'stereo_corr':
        collar_size = int(settings['collar_size'][0])
        local_tile.add_collar(collar_size)

    # Will do only the tiles intersecting user's crop window.
    w = settings['transformed_window']
    user_crop_win = BBox(int(w[0]), int(w[1]), int(w[2]), int(w[3]))
    
    crop_box = intersect_boxes(user_crop_win, local_tile)
    if crop_box.width <= 0 or crop_box.height <= 0: 
        return [] # Don't need to process this

    return crop_box

def build_vrt(prog, opt, args, settings, georef, postfix, tile_postfix, contract_tiles=False):
    '''Generate a VRT file to treat the separate image tiles as one large image.'''

    image_size = settings["trans_left_image_size"]

    vrt_file = settings['out_prefix'][0]+postfix
    print("Writing: " + vrt_file)
    f = open(vrt_file,'w')
    f.write("<VRTDataset rasterXSize=\"%i\" rasterYSize=\"%i\">\n" %
            (int(image_size[0]),int(image_size[1])))

    # Write the datum, projection, and georeference transform in XML format
    f.write("  <SRS>" + georef["WKT"] + "</SRS>\n")
    f.write("  <GeoTransform>" + georef["GeoTransform"] + "</GeoTransform>\n")

    # Locate a known good tile
    tiles = produce_tiles(opt, args, settings, opt.job_size_w, opt.job_size_h)
    goodFilename = ""
    for tile in tiles:
        directory = tile_dir(settings['out_prefix'][0], tile)
        filename  = directory + "/" + tile.name_str() + tile_postfix
        if os.path.isfile(filename):
            goodFilename = filename
            break
    if goodFilename == "":
        raise Exception('No tiles were generated.')

    # Do gdalinfo on the good tile to get metadata
    args = [goodFilename]
    sep = "="
    gdal_settings = run_and_parse_output("gdalinfo", args, sep, opt.verbose)
    
    # Extract the data type (e.g., Float32 or Float64)
    data_type = "Float32"
    for s in gdal_settings:
        val = " ".join(gdal_settings[s])
        m = re.match(r'^.*? Type (\w+)', val)
        if m:
            data_type = m.group(1)
            break

    # Extract the block size, e.g., 256x256, from 'Band 2 Block=256x256'
    BlockXSize = 0
    BlockYSize = 0
    for s in gdal_settings:
        val = " ".join(gdal_settings[s])
        if re.match(r"^\s*Band\s+\d", s):
            m = re.match(r'^\s*(\d+)x(\d+)', val)
            if m:
                BlockXSize = m.group(1)
                BlockYSize = m.group(2)
                break

    # Extract no-data value, if present
    has_nodata = False
    key = 'NoData Value'
    nodata_value = 0
    if key in gdal_settings:
        has_nodata = True
        nodata_value = gdal_settings[key][0]

    # Find how many bands are in the file
    num_bands = 0
    for s in gdal_settings:
        m = re.match(r'^.*?Band\s+(\d+)', s)
        if m:
            b = int(m.group(1))
            if num_bands < b:
                num_bands = b

    # Copy some keys over to the vrt
    keys = ["POINT_OFFSET", "AREA_OR_POINT", "BAND1", "BAND2", "BAND3", "BAND4", "BAND5", "BAND6"]
    for key in keys:
        if key in gdal_settings:
            f.write("  <Metadata>\n    <MDI key=\"" + key + "\">" +
                    gdal_settings[key][0] + "</MDI>\n  </Metadata>\n")

    # Write each band
    for b in range(1, num_bands + 1):
        f.write("  <VRTRasterBand dataType=\"%s\" band=\"%i\">\n" % (data_type,b))

        for tile in tiles:
            directory = tile_dir(settings['out_prefix'][0], tile)
            filename  = directory + "/" + tile.name_str() + tile_postfix

            # Handle non-existent tiles
            if not os.path.isfile(filename):
                continue # Leave empty

            relative  = os.path.relpath(filename, os.path.dirname(settings['out_prefix'][0]))
            f.write("    <SimpleSource>\n")
            f.write("       <SourceFilename relativeToVRT=\"1\">%s</SourceFilename>\n" % relative)
            f.write("       <SourceBand>%i</SourceBand>\n" % b)

            # Bugfix for the "too many open files" problem.
            # Per https://lists.osgeo.org/pipermail/gdal-dev/2017-October/047370.html
            adjusted_tile = grow_crop_tile_maybe(settings, prog, tile)
            f.write(('       <SourceProperties RasterXSize="%i" RasterYSize="%i" ' + \
                     'DataType="%s" BlockXSize="%s" BlockYSize="%s"/>\n') \
                    % (adjusted_tile.width, adjusted_tile.height, data_type, BlockXSize, BlockYSize))

            min_x  = 0
            min_y  = 0
            if (contract_tiles):
                # Need to account for the padding
                pad_amount = int(settings['collar_size'][0])
                user_min_x = int(settings['transformed_window'][0])
                user_min_y = int(settings['transformed_window'][1])
                # For the tiles not starting at zero, account for the fact that they
                #  have padding at the top and/or right of the images.
                if (tile.x > user_min_x):
                    min_x  += pad_amount
                if (tile.y > user_min_y):
                    min_y  += pad_amount

            f.write('       <SrcRect xOff="%i" yOff="%i" xSize="%i" ySize="%i"/>\n' %
                    (min_x, min_y, tile.width, tile.height))

            f.write('       <DstRect xOff="%i" yOff="%i" xSize="%i" ySize="%i"/>\n' %
                    (tile.x, tile.y, tile.width, tile.height))
            f.write("    </SimpleSource>\n")
        # End tile loop

        if has_nodata:
            f.write("  <NoDataValue>" + nodata_value + "</NoDataValue>\n")
        
        f.write("  </VRTRasterBand>\n")
    # End band loop
    f.write("</VRTDataset>\n")
    f.close()

