#!/usr/bin/env python
# __BEGIN_LICENSE__
#  Copyright (c) 2009-2026, United States Government as represented by the
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

"""
Utility functions for stereo_dist (distributed stereo processing).
"""

import sys, subprocess, os, math, shlex

import asp_system_utils
from asp_stereo_utils import Step, readDistTileList

def tileGridSize(tiles):
    '''Given a list of (BBox, padding) tuples from readDistTileList,
    return (num_cols, num_rows) of the tile grid.'''
    xs = set(tile.x for (tile, padding) in tiles)
    ys = set(tile.y for (tile, padding) in tiles)
    return (len(xs), len(ys))

def numMosaicJobs(nodesListPath, numProcesses):
    '''Compute the number of parallel dem_mosaic jobs for DEM assembly.
    Uses nodes * processes, capped at 32.'''
    numNodes = asp_system_utils.getNumNodesInList(nodesListPath)
    if numProcesses is not None and numProcesses > 0:
        numJobs = numNodes * numProcesses
    else:
        numJobs = numNodes * asp_system_utils.get_num_cpus()
    return min(numJobs, 32)

def buildMosaicBlockLists(outPrefix, numJobs, suffix='DEM'):
    '''Partition tile raster files into blocks for parallel mosaicking. Reads
    the distributed tile list, collects existing files with the given suffix
    (e.g., DEM, DRG, IntersectionErr), divides the tile grid into blocks,
    writes per-block list files, and returns a list of (blockListFile,
    blockOutputFile) tuples.'''

    # Read the list of tiles
    tiles = readDistTileList(outPrefix)
    numTiles = len(tiles)
    if numTiles == 0:
        raise Exception('No tiles found in tile list.')

    # Collect files with given suffix that exist
    demFiles = []
    for (tile, padding) in tiles:
        tileSubdir = outPrefix + '-' + tile.name_str()
        tilePrefix = tileSubdir + '/' + os.path.basename(outPrefix)
        demFile = tilePrefix + '-' + suffix + '.tif'
        if os.path.exists(demFile):
            demFiles.append(demFile)
    if len(demFiles) == 0:
        raise Exception('No ' + suffix + ' files found in tile directories.')
    print('Found ' + str(len(demFiles)) + ' ' + suffix + ' files to mosaic.')

    # Compute tile grid size and number of parallel jobs
    (numCols, numRows) = tileGridSize(tiles)
    print('Tile grid: ' + str(numCols) + ' cols x ' + str(numRows) + ' rows')

    # Each block must have at least 2 tiles. Use this to limit the number of jobs
    maxJobs = (numCols * numRows) // 2
    numJobs = max(min(numJobs, maxJobs), 1)
    print('Num mosaic jobs: ' + str(numJobs))

    # Divide tile grid into blocks matching the aspect ratio
    aspect = float(numCols) / float(max(numRows, 1))

    blockCols = int(round(math.sqrt(numJobs * aspect)))
    blockRows = int(round(numJobs / float(max(blockCols, 1))))
    blockCols = max(blockCols, 1)
    blockRows = max(blockRows, 1)
    blockCols = min(blockCols, numCols)
    blockRows = min(blockRows, numRows)
    print('Block grid: ' + str(blockCols) + ' x ' + str(blockRows) +
          ' = ' + str(blockCols * blockRows) + ' blocks')

    # Compute tiles per block in each dimension
    tilesPerBlockX = int(math.ceil(numCols / blockCols))
    tilesPerBlockY = int(math.ceil(numRows / blockRows))
    print('Tiles per block: ' + str(tilesPerBlockX) + ' x ' +
          str(tilesPerBlockY))

    # Build sorted lists of distinct tile starting corner coordinates
    tileStartCornerX = sorted(set(tile.x for (tile, padding) in tiles))
    tileStartCornerY = sorted(set(tile.y for (tile, padding) in tiles))

    # Build a map from (x, y) origin to file path
    demMap = {}
    for (tile, padding) in tiles:
        tileSubdir = outPrefix + '-' + tile.name_str()
        tilePrefix = tileSubdir + '/' + os.path.basename(outPrefix)
        demFile = tilePrefix + '-' + suffix + '.tif'
        if os.path.exists(demFile):
            demMap[(tile.x, tile.y)] = demFile

    # Keep track of each partial DEM mosaic name and of the list of DEMs
    # that goes into making it.
    masterList = []

    # Assign tiles to blocks. For each block, find which tile grid columns
    # and rows it covers, convert grid indices to pixel origins via
    # tileStartCornerX/Y, and look up the DEM file in demMap.
    for by in range(blockRows):
        for bx in range(blockCols):
            # Range of tile indices for this block
            colStart = bx * tilesPerBlockX
            colEnd = min((bx + 1) * tilesPerBlockX, numCols)
            rowStart = by * tilesPerBlockY
            rowEnd = min((by + 1) * tilesPerBlockY, numRows)

            blockDems = []
            for ci in range(colStart, colEnd):
                for ri in range(rowStart, rowEnd):
                    # Tile corner in pixel coordinates
                    key = (tileStartCornerX[ci], tileStartCornerY[ri])
                    if key in demMap:
                        # Collect DEMs that exist
                        blockDems.append(demMap[key])

            # Skip empty blocks
            if len(blockDems) == 0:
                continue

            # Write this block's DEM list file
            blockIndex = len(masterList)
            blockDemListFile = (outPrefix + '-mosaicBlock_' +
                                str(blockIndex) + '_' + suffix + '.txt')
            blockOutputDem = (outPrefix + '-mosaicBlock_' +
                              str(blockIndex) + '-' + suffix + '.tif')
            with open(blockDemListFile, 'w') as f:
                for d in blockDems:
                    f.write(d + '\n')

            masterList.append((blockDemListFile, blockOutputDem))
            print('Block ' + str(blockIndex) + ' (' + str(bx) + ', ' +
                  str(by) + '): ' + str(len(blockDems)) + ' DEMs, list: ' +
                  blockDemListFile)

    return masterList

def queryMapproject(dem, image, camera, outputImage, extraArgs):
    '''Run mapproject_single --query-projection and return parsed settings.'''
    queryArgs = ['--query-projection', dem, image]
    if camera:
        queryArgs.append(camera)
    queryArgs.append(outputImage)
    queryArgs += extraArgs
    sep = ","
    return asp_system_utils.run_and_parse_output(
        'mapproject_single', queryArgs, sep, verbose=False)

def runMapprojection(opt, settings, outPrefix, entryPoint):
    '''Mapproject both input images onto the DEM with a common projection,
    grid size, and bounding box. Return the paths to the mapprojected images.
    If entryPoint > 0, skip mapprojection and just return the expected paths,
    verifying that the mapprojected images already exist.'''

    in_file1  = settings['in_file1'][0]
    in_file2  = settings['in_file2'][0]
    cam_file1 = settings['cam_file1'][0]
    cam_file2 = settings['cam_file2'][0]

    leftMap  = outPrefix + '-left_map.tif'
    rightMap = outPrefix + '-right_map.tif'

    # If resuming from a later entry point, just verify the mapprojected
    # images exist and return their paths.
    if entryPoint > Step.pprc:
        for f in [leftMap, rightMap]:
            if not os.path.exists(f):
                raise Exception('Mapprojected image not found: ' + f +
                    '. Run from entry point 0 first.')
        print('Skipping mapprojection (entry point > ' + str(Step.pprc) + ').')
        return (leftMap, rightMap)

    # Create the output directory if needed
    outDir = os.path.dirname(outPrefix)
    if outDir and not os.path.exists(outDir):
        os.makedirs(outDir)

    # Step 1: Query left image to get auto projection, GSD, and bounding box.
    # If --t_srs is provided, use it for the left query too.
    leftExtraArgs = []
    if opt.t_srs is not None:
        leftExtraArgs = ['--t_srs', opt.t_srs]
    print('Querying projection for the left image.')
    leftQuery = queryMapproject(opt.dem, in_file1, cam_file1, leftMap,
                                leftExtraArgs)

    leftGsd = float(leftQuery['pixel_size'][0])
    leftWkt = leftQuery['projection_wkt_file'][0]
    leftXmin = float(leftQuery['proj_box_xmin'][0])
    leftYmin = float(leftQuery['proj_box_ymin'][0])
    leftXmax = float(leftQuery['proj_box_xmax'][0])
    leftYmax = float(leftQuery['proj_box_ymax'][0])

    print('Left GSD: ' + str(leftGsd))
    print('Left WKT: ' + leftWkt)
    print('Left proj box: ' + str(leftXmin) + ' ' + str(leftYmin) +
          ' ' + str(leftXmax) + ' ' + str(leftYmax))

    # Step 2: Query right image with the left projection forced
    print('Querying projection for the right image.')
    rightQuery = queryMapproject(opt.dem, in_file2, cam_file2, rightMap,
                                ['--t_srs', leftWkt])

    rightGsd = float(rightQuery['pixel_size'][0])
    rightXmin = float(rightQuery['proj_box_xmin'][0])
    rightYmin = float(rightQuery['proj_box_ymin'][0])
    rightXmax = float(rightQuery['proj_box_xmax'][0])
    rightYmax = float(rightQuery['proj_box_ymax'][0])

    print('Right GSD: ' + str(rightGsd))
    print('Right proj box: ' + str(rightXmin) + ' ' + str(rightYmin) +
          ' ' + str(rightXmax) + ' ' + str(rightYmax))

    # Step 3: Use the finer (smaller) GSD
    gsd = min(abs(leftGsd), abs(rightGsd))

    # Step 4: Intersect the two bounding boxes
    xmin = max(leftXmin, rightXmin)
    ymin = max(leftYmin, rightYmin)
    xmax = min(leftXmax, rightXmax)
    ymax = min(leftYmax, rightYmax)

    if xmin >= xmax or ymin >= ymax:
        raise Exception('The two images do not overlap in projected space.')

    # Step 5: Override with user-provided values
    if opt.tr is not None:
        gsd = opt.tr
        print('Using input GSD: ' + str(gsd))
    else:
        print('Using min estim GSD: ' + str(gsd))

    if opt.t_projwin is not None:
        xmin = opt.t_projwin[0]
        ymin = opt.t_projwin[1]
        xmax = opt.t_projwin[2]
        ymax = opt.t_projwin[3]
        print('Using input box: ' + str(xmin) + ' ' + str(ymin) +
          ' ' + str(xmax) + ' ' + str(ymax))
    else:
        print('Using intersection box: ' + str(xmin) + ' ' + str(ymin) +
          ' ' + str(xmax) + ' ' + str(ymax))

    if opt.t_srs is not None:
       leftWkt = opt.t_srs
       print('Using input projection: ' + leftWkt)

    # Step 6: Run mapproject on both images with common parameters
    commonArgs = ['--t_srs', leftWkt, '--tr', str(gsd),
                  '--t_projwin', str(xmin), str(ymin), str(xmax), str(ymax)]

    mapprojectPath = asp_system_utils.bin_path('mapproject')

    for (image, camera, outputPath, label) in \
        [(in_file1, cam_file1, leftMap, 'left'),
         (in_file2, cam_file2, rightMap, 'right')]:

        cmd = [mapprojectPath] + commonArgs + [opt.dem, image]
        if camera:
            cmd.append(camera)
        cmd.append(outputPath)
        if opt.nodes_list:
            cmd += ['--nodes-list', opt.nodes_list]

        print('Mapprojecting ' + label + ' image.')
        if opt.verbose or opt.dryrun:
            print(' '.join(cmd))
        if not opt.dryrun:
            subprocess.check_call(cmd)

    return (leftMap, rightMap)

def mapprojAndUpdateArgs(opt, settings, outPrefix, sep):
    '''Mapproject both images onto the DEM with consistent parameters, then
    rebuild the argument list to use the mapprojected images instead of the
    originals. Return updated (args, settings, outPrefix).'''

    (leftMap, rightMap) = runMapprojection(opt, settings, outPrefix,
                                           opt.entry_point)

    # Care is needed when the images are .cub so have the cameras embedded. For RPC,
    # the mapprojected images will be .tif but will hve internally the cameras, so no
    # need for extra args.
    in_file1  = settings['in_file1'][0]
    in_file2  = settings['in_file2'][0]
    cam_file1 = settings['cam_file1'][0]
    cam_file2 = settings['cam_file2'][0]
    if settings['stereo_session'][0] == 'isis':
        if not cam_file1:
            cam_file1 = in_file1
        if not cam_file2:
            cam_file2 = in_file2

    # named_options has all the options and values we need to pass through
    named_opts = settings.get('named_options', [''])
    if named_opts == ['']:
        named_opts = []
    args = [leftMap, rightMap]
    if cam_file1:
        args.append(cam_file1)
    if cam_file2:
        args.append(cam_file2)
    args.append(outPrefix)
    args += named_opts

    # Add the DEM for the stereo pipeline, if not already present
    if '--dem' not in args:
        args.extend(['--dem', opt.dem])

    # Re-parse with updated args. Likely redundant, but good to have as a
    # sanity check that we reconnected correctly the args.
    settings = asp_system_utils.run_and_parse_output("stereo_parse", args,
                                                      sep, opt.verbose)
    outPrefix = settings['out_prefix'][0]

    return (args, settings, outPrefix)

def runStereoTiles(opt, args, outPrefix, tileEntryPoint, tileStopPoint):
    '''Run stereo_tile on all tiles using GNU Parallel.'''

    tiles = readDistTileList(outPrefix)
    numTiles = len(tiles)
    if numTiles == 0:
        raise Exception('No tiles found in tile list.')

    print('Running stereo_tile on ' + str(numTiles) + ' tiles.')

    # Write argument file (one tile index per line)
    argumentFilePath = outPrefix + '-tileArgs.txt'
    with open(argumentFilePath, 'w') as f:
        for i in range(numTiles):
            f.write(str(i) + '\n')

    # Set up GNU parallel arguments
    parallelArgs = ['--will-cite', '--env', 'ASP_DEPS_DIR',
                    '--env', 'PATH', '--env', 'LD_LIBRARY_PATH',
                    '--env', 'ASP_LIBRARY_PATH', '--env', 'ISISROOT']

    if opt.parallel_options is not None:
        parallelArgs += opt.parallel_options.split(' ')

    # Determine number of processes
    numProcesses = opt.processes
    if numProcesses is None:
        numProcesses = asp_system_utils.get_num_cpus()

    # Build stereo_tile command with placeholder for tile index
    pythonPath = sys.executable
    stereoTilePath = asp_system_utils.libexec_path('stereo_tile')
    commandList = [pythonPath, stereoTilePath,
                   '--tile-index', '{1}',
                   '--entry-point', str(tileEntryPoint),
                   '--stop-point', str(tileStopPoint)]

    if opt.dem:
        commandList += ['--dem', opt.dem]
    if opt.point2dem_options:
        commandList += ['--point2dem-options', opt.point2dem_options]

    if opt.verbose:
        commandList.append('--verbose')
    if opt.dryrun:
        commandList.append('--dry-run')

    commandList += args

    # Append the run directory so the child can chdir to it
    runDir = asp_system_utils.escape_token(os.getcwd())
    commandList += ['--run-dir', runDir]

    # Prepend parallel args to command
    fullCommandList = parallelArgs + commandList

    if opt.dryrun:
        print('Run GNU parallel with ' + str(numProcesses) + ' processes')
        print('Command: ' + ' '.join(fullCommandList))
        return

    asp_system_utils.runInGnuParallel(numProcesses, argumentFilePath,
                                      fullCommandList, opt.nodes_list, opt.verbose)

def runBlockMosaic(opt, masterFile):
    '''Run dem_mosaic in parallel on blocks via stereo_tile --dem-mosaic-index.'''

    # Count blocks in master file
    with open(masterFile, 'r') as f:
        lines = [l.strip() for l in f if l.strip()]
    numBlocks = len(lines)
    if numBlocks == 0:
        raise Exception('No blocks in master file: ' + masterFile)

    print('Running dem_mosaic on ' + str(numBlocks) + ' blocks.')

    # Write argument file (one block index per line, 0-based)
    argumentFilePath = masterFile.replace('.txt', '-args.txt')
    with open(argumentFilePath, 'w') as f:
        for i in range(numBlocks):
            f.write(str(i) + '\n')

    # Set up GNU parallel arguments
    parallelArgs = ['--will-cite', '--env', 'ASP_DEPS_DIR',
                    '--env', 'PATH', '--env', 'LD_LIBRARY_PATH',
                    '--env', 'ASP_LIBRARY_PATH', '--env', 'ISISROOT']

    if opt.parallel_options is not None:
        parallelArgs += opt.parallel_options.split(' ')

    # Determine number of processes
    numProcesses = opt.processes
    if numProcesses is None:
        numProcesses = asp_system_utils.get_num_cpus()

    # Build stereo_tile command with --dem-mosaic-index placeholder
    pythonPath = sys.executable
    stereoTilePath = asp_system_utils.libexec_path('stereo_tile')
    commandList = [pythonPath, stereoTilePath,
                   '--dem-mosaic-index', '{1}',
                   '--dem-mosaic-master', masterFile]

    if opt.verbose:
        commandList.append('--verbose')
    if opt.dryrun:
        commandList.append('--dry-run')

    # Append the run directory so the child can chdir to it
    runDir = asp_system_utils.escape_token(os.getcwd())
    commandList += ['--run-dir', runDir]

    # Prepend parallel args to command
    fullCommandList = parallelArgs + commandList

    if opt.dryrun:
        print('Run GNU parallel with ' + str(numProcesses) + ' processes')
        print('Command: ' + ' '.join(fullCommandList))
        return

    asp_system_utils.runInGnuParallel(numProcesses, argumentFilePath,
                                      fullCommandList, opt.nodes_list,
                                      opt.verbose)

def mosaicProduct(opt, outPrefix, numJobs, suffix):
    '''Mosaic blocks of per-tile products (DEM, DRG, or IntersectionErr) in
    parallel, then mosaic the block results into a final output.'''

    # Partition tile files into blocks for parallel mosaicking
    masterList = buildMosaicBlockLists(outPrefix, numJobs, suffix)

    # Write master file having the list of lists to mosaic
    masterFile = outPrefix + '-mosaicMaster_' + suffix + '.txt'
    with open(masterFile, 'w') as f:
        for (listFile, outputFile) in masterList:
            f.write(listFile + ' ' + outputFile + '\n')
    print('Wrote master mosaic file: ' + masterFile + ' with ' +
          str(len(masterList)) + ' blocks')

    # Run parallel dem_mosaic per block
    runBlockMosaic(opt, masterFile)

    # Final merge of block files
    blockFiles = [outputFile for (listFile, outputFile) in masterList]
    blockFiles = [f for f in blockFiles if os.path.exists(f)]
    if len(blockFiles) == 0:
        raise Exception('No block ' + suffix + ' files were produced.')

    finalFile = outPrefix + '-' + suffix + '.tif'
    print('Merging ' + str(len(blockFiles)) + ' block ' + suffix +
          ' files into ' + finalFile)
    demMosaicPath = asp_system_utils.bin_path('dem_mosaic')
    commandList = [demMosaicPath] + blockFiles + ['-o', finalFile]
    if not opt.dryrun:
        if opt.verbose:
            print(' '.join(commandList))
        subprocess.call(commandList)

    # Clean up partial block mosaics and helper lists
    print('Removing partial ' + suffix + ' mosaics and helper lists.')
    for (listFile, outputFile) in masterList:
        if os.path.exists(outputFile):
            os.remove(outputFile)
        if os.path.exists(listFile):
            os.remove(listFile)
    # Remove the args file created by runBlockMosaic
    argsFile = masterFile.replace('.txt', '-args.txt')
    if os.path.exists(argsFile):
        os.remove(argsFile)
    if os.path.exists(masterFile):
        os.remove(masterFile)

def mosaicDems(opt, outPrefix):
    '''Mosaic per-tile DEMs, and optionally DRG and IntersectionErr files.'''

    numJobs = numMosaicJobs(opt.nodes_list, opt.processes)

    # Always mosaic DEMs
    mosaicProduct(opt, outPrefix, numJobs, 'DEM')

    # Mosaic ortho images if --orthoimage was requested
    if opt.point2dem_options and '--orthoimage' in opt.point2dem_options:
        mosaicProduct(opt, outPrefix, numJobs, 'DRG')

    # Mosaic error images if --errorimage was requested
    if opt.point2dem_options and '--errorimage' in opt.point2dem_options:
        mosaicProduct(opt, outPrefix, numJobs, 'IntersectionErr')
