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

# Plot roll, pitch, and yaw of ASP Pinhole .tsai cameras, and/or of CSM 
# Frame/Linescan .json cameras, before and after bundle adjustment.
# See the documentation for more info.

import sys, os, re, math, json, argparse, shutil, glob
import numpy as np
import matplotlib.pyplot as plt
from pyproj import Proj, transform, Transformer
from scipy.spatial.transform import Rotation as R

def produce_m(lon,lat,m_meridian_offset=0):
    """
    Produce M matrix which facilitates conversion from Lon-lat (NED) to ECEF coordinates
    From https://github.com/visionworkbench/visionworkbench/blob/master/src/vw/Cartography/Datum.cc#L249
    This is known as direction cosie matrix
    
    Parameters
    ------------
    lon: numeric
        longitude of spacecraft
    lat: numeric
        latitude of spacecraft
    m_meridian_offset: numeric
        set to zero
    Returns
    -----------
    R: np.array
        3 x 3 rotation matrix representing the m-matrix aka direction cosine matrix
    """
    if lat < -90:
        lat = -90
    if lat > 90:
        lat = 90
    
    rlon = (lon + m_meridian_offset) * (np.pi/180)
    rlat = lat * (np.pi/180)
    slat = np.sin(rlat)
    clat = np.cos(rlat)
    slon = np.sin(rlon)
    clon = np.cos(rlon)
    
    R = np.ones((3,3),dtype=float)
    R[0,0] = -slat*clon
    R[1,0] = -slat*slon
    R[2,0] = clat
    R[0,1] = -slon
    R[1,1] = clon
    R[2,1] = 0.0
    R[0,2] = -clon*clat
    R[1,2] = -slon*clat
    R[2,2] = -slat
    return R

def convert_ecef2NED(asp_rotation,lon,lat):
    """
    convert rotation matrices from ECEF to North-East-Down convention
    Parameters
    -------------
    asp_rotation: np.array
        3 x 3 rotation matrix from ASP
    lon: numeric
        longitude for computing m matrix
    lat: numeric
        latitude for computing m matrix
    
    Returns
    --------------
    r_ned: np.array
        3 x 3 NED rotation matrix 
    """
    m = produce_m(lon,lat)
    r_ned = np.matmul(np.linalg.inv(m),asp_rotation)  # this is the cam to ned transform
    #r_ned = np.matmul(np.transpose(m),asp_rotation)
    #r_ned = np.matmul(m,asp_rotation)
    return r_ned

def read_frame_cam_dict(cam):

    # Invoke the appropriate reader for .tsai and .json frame cameras
    if cam.endswith('.tsai'):
        return read_tsai_cam(cam)
    elif cam.endswith('.json'):
        return read_frame_csm_cam(cam)
    else:
        raise Exception('Unknown camera file extension: ' + cam)

def read_tsai_cam(tsai):
    """
    read tsai frame model from asp and return a python dictionary containing the parameters
    See ASP's frame camera implementation here: https://stereopipeline.readthedocs.io/en/latest/pinholemodels.html
    Parameters
    ----------
    tsai: str
        path to ASP frame camera model
    Returns
    ----------
    output: dictionary
        dictionary containing camera model parameters
    #TODO: support distortion model
    """
    camera = os.path.basename(tsai)
    with open(tsai, 'r') as f:
        content = f.readlines()
    content = [x.strip() for x in content]
    fu = np.float64(content[2].split(' = ', 4)[1]) # focal length in x
    fv = np.float64(content[3].split(' = ', 4)[1]) # focal length in y
    cu = np.float64(content[4].split(' = ', 4)[1]) # optical center in x
    cv = np.float64(content[5].split(' = ', 4)[1]) # optical center in y
    cam = content[9].split(' = ', 10)[1].split(' ')
    cam_cen = [np.float64(x) for x in cam] # camera center coordinates in ECEF
    rot = content[10].split(' = ', 10)[1].split(' ')
    rot_mat = [np.float64(x) for x in rot] # rotation matrix for camera to world coordinates transformation

    # Reshape as 3x3 matrix
    rot_mat = np.reshape(rot_mat,(3,3))

    pitch = np.float64(content[11].split(' = ', 10)[1]) # pixel pitch
    
    ecef_proj = 'EPSG:4978'
    geo_proj = 'EPSG:4326'
    ecef2wgs = Transformer.from_crs(ecef_proj, geo_proj)
    cam_cen_lat_lon = ecef2wgs.transform(cam_cen[0], cam_cen[1], cam_cen[2]) # this returns lat, lon and height
    # cam_cen_lat_lon = geolib.ecef2ll(cam_cen[0], cam_cen[1], cam_cen[2]) # camera center coordinates in geographic coordinates
    tsai_dict = {'camera':camera, 'focal_length':(fu, fv), 'optical_center':(cu, cv), 'cam_cen_ecef':cam_cen, 'cam_cen_wgs':cam_cen_lat_lon, 'rotation_matrix':rot_mat, 'pitch':pitch}
    return tsai_dict

def read_frame_csm_cam(json_file):
    """
    Read rotation from a CSM Frame json state file.
    """

    with open(json_file, 'r') as f:
        data = f.read()

    # Find first occurrence of open brace. This is needed because the CSM
    # state has some text before the JSON object.
    pos = data.find('{')
    # do substring from pos to the end, if pos was found
    if pos != -1:
        data = data[pos:]

    # parse the json from data
    j = json.loads(data)
    # print the json 
    # print(json.dumps(j, indent=4, sort_keys=True))

    # Print all keys in the json
    # print("will print all keys in the json")
    # for key in j.keys():
    #     print(key)

    # Read the entry having the translation and rotation
    params = j['m_currentParameterValue']

    # First three entries are the translation
    dict = {}
    dict['cam_cen_ecef'] = params[0:3]
    
    # Next four entries are the quaternion
    quat = params[3:7]

    # Convert the quaternion to rotation matrix
    r = R.from_quat(quat)
    mat = r.as_matrix()
    dict['rotation_matrix'] = mat

    return dict

def read_linescan_csm_cam(json_file):
    """
    Read rotation from a CSM linescan json state file.
    """

    with open(json_file, 'r') as f:
        data = f.read()

    # Find first occurrence of open brace. This is needed because the CSM
    # state has some text before the JSON object.
    pos = data.find('{')
    # do substring from pos to the end, if pos was found
    if pos != -1:
        data = data[pos:]

    # parse the json from data
    j = json.loads(data)
    # print the json 
    # print(json.dumps(j, indent=4, sort_keys=True))

    # Print all keys in the json
    # print("will print all keys in the json")
    # for key in j.keys():
    #     print(key)

    # Read the positions
    positions_vec = j['m_positions']

    # Reshape to Nx3 matrix using the reshape function
    positions_vec = np.reshape(positions, (-1, 3))

    # Create a vector of vectors
    positions = []
    for i in range(positions_vec.shape[0]):
        positions.append(positions_vec[i, :])

    # Read the quaternions
    quats = j['m_quaternions']

    # Reshape to Nx4 matrix using the reshape function
    quats = np.reshape(quats, (-1, 4))
    
    # Iterate over the rows and convert to rotation matrix
    rotations = []
    for i in range(quats.shape[0]):
        r = R.from_quat(quats[i, :])
        # print the rotation matrix
        rotations.append(r.as_matrix())

    return (positions, rotations)

def Rroll(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, math.cos(theta),-math.sin(theta)],
                   [ 0, math.sin(theta), math.cos(theta)]])
  
def Rpitch(theta):
  return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-math.sin(theta), 0, math.cos(theta)]])
  
def Ryaw(theta):
  return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
                   [ math.sin(theta), math.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

def isLinescan(cam_file):
    """
    Read the first line from cam_file which tells if the sensor is linescan.
    """
    lineScan = False
    with open(cam_file, 'r') as f:
        line = f.readline()
        if 'LINE_SCAN' in line:
            lineScan = True
    
    return lineScan

def roll_pitch_yaw(rot_mat, ref_rot_mat):

    # rotate about z axis by 90 degrees
    # This will be a problem for non-sat_sim cameras
    T = np.zeros((3,3),float)
    T[0, 1] = 1
    T[1, 0] = -1
    T[2, 2] = 1
    Tinv = np.linalg.inv(T)

    inv_ref_rot_mat = np.linalg.inv(ref_rot_mat)
    N = np.matmul(inv_ref_rot_mat, rot_mat)

    return R.from_matrix(np.matmul(N, Tinv)).as_euler('XYZ',degrees=True)

# Return at most this many elements from an array
def getFirstN(arr, N):
    if N >= 0 and len(arr) > N:
        return arr[:N]
    else:
        return arr

def poly_fit(X, Y):
    """
    Fit a polynomial of degree 1 and return the fitted Y values.
    """
    fit = np.poly1d(np.polyfit(X, Y, 1))
    return fit(X)

def multi_glob(prefix, extensions):
    """
    Return a list of files matching the given prefix and extensions.
    """
    files = []
    for ext in extensions:
        files += glob.glob(prefix + '*' + ext)  
    return files

# Read the positions and rotations from the given files. For linescan we will
# have a single camera, but with many poses in it. For Pinhole we we will have
# many cameras, each with a single pose.
def read_positions_rotations_from_file(cam_file):
    
    # Read the first line from cam_file
    lineScan = isLinescan(cam_file)

    positions = []
    rotations = []

    if lineScan:
        # Read linescan data
        (positions, rotations) = read_linescan_csm_cam(cam_file)
    else:   
        # read Pinhole (Frame) files in ASP .tsai or CSM .json format
        asp_dict = read_frame_cam_dict(cam_file)
        # get camera rotation
        position = asp_dict['cam_cen_ecef']
        rot_mat = asp_dict['rotation_matrix']
        positions.append(position)
        rotations.append(rot_mat)

    return (positions, rotations)

# Read the positions, rotations, and reference rotations from the given files
def read_positions_rotations(cams):

  (positions, rotations) = ([], [])
  for i in range(len(cams)):
      (p, r) = read_positions_rotations_from_file(cams[i])
      positions += p
      rotations += r

  return (positions, rotations)

# Get rotations, then convert to NED.  That's why the loops below. 
def read_angles(orig_cams, opt_cams, ref_cams):

  # orig_cams and ref_cams must be the same size
  if len(orig_cams) != len(ref_cams):
      print("Number of input and reference cameras must be the same. Got: ", \
           len(ref_cams), " and ", len(opt_cams))
      sys.exit(1)

  (orig_positions, orig_rotations) = read_positions_rotations(orig_cams)
  (opt_positions, opt_rotations)   = read_positions_rotations(opt_cams)
  (ref_positions, ref_rotations)   = read_positions_rotations(ref_cams)

  orig_rotation_angles = []
  opt_rotation_angles = []
  for i in range(len(orig_rotations)):
      angles = roll_pitch_yaw(orig_rotations[i], ref_rotations[i])
      orig_rotation_angles.append(angles)
  for i in range(len(opt_rotations)):
      angles = roll_pitch_yaw(opt_rotations[i], ref_rotations[i])
      opt_rotation_angles.append(angles)

  return (orig_rotation_angles, opt_rotation_angles)

# Main function

usage  = "python orbit_plot.py <options>"

parser = argparse.ArgumentParser(usage=usage,
                                 formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument('--dataset', dest = 'dataset', default = "", 
                    help='The dataset to plot. If more than one, separate them by comma, with no spaces in between. The dataset is the prefix of the cameras, such as  "cameras/" or "opt/run-". It is to be followed by the orbit id, such as, "nadir" or "aft". If more than one dataset, they will be plotted on top of each other.')

parser.add_argument('--orbit-id', dest = 'orbit_id', default = "", 
                    help='The id (a string) that determines an orbital group of cameras. If more than one, separate them by comma, with no spaces in between.')

parser.add_argument('--label', dest = 'label', default = "", 
                    help='The label to use for each dataset in the legend. If more than one, separate them by comma, with no spaces in between. If not set, will use the dataset name.')

parser.add_argument("--num-cameras",  dest="num_cameras", type=int, default = -1,
                    help="Plot only the first this many cameras from each orbital sequence. By default, plot all of them.")

parser.add_argument("--trim-ratio",  dest="trim_ratio", type=float, default = 0.0,
                    help="Trim ratio. Given a value between 0 and 1 (inclusive), remove this fraction of camera poses from each sequence, with half of this amount for poses at the beginning and half at the end of the sequence. This is used only for linescan, to not plot camera poses beyond image lines. For cameras created with sat_sim, a value of 0.5 should be used.")

parser.add_argument('--plot-size', dest = 'plot_size', default = "15,15", 
                    help='Specify the width and height of the figure having the plots, in inches. Use two numbers with comma as separator (no spaces).')

parser.add_argument('--subtract-line-fit', dest = 'subtract_line_fit', action='store_true',
                    help='If set, subtract the best line fit from the curves being plotted.')

(options, args) = parser.parse_known_args(sys.argv)

if options.orbit_id == "" or options.dataset == "":
    print("Must set the --orbit-id and --dataset options.")
    parser.print_help()
    sys.exit(1)

if options.trim_ratio < 0:
    print("The value of --trim-ratio must be non-negative.")
    parser.print_help()
    sys.exit(1)

# Split by comma
Types = options.orbit_id.split(',')
datasets = options.dataset.split(',')
labels = options.label.split(',')
if len(labels) == 0 or (len(labels) == 1 and labels[0] == ""):
    labels = datasets[:]
if len(labels) != len(datasets):
    print("Number of datasets and labels must agree. Got ", datasets, " and ", labels)
    sys.exit(1)

plot_size = options.plot_size.split(',')
if len(plot_size) != 2:
    print("The --plot-size option must have two values. Got: ", plot_size)
    sys.exit(1)
# Convert to float
plot_size = [float(x) for x in plot_size]

# We assume we have one or two datasets that we want to plot on top of each other.
numSets = len(datasets)
if numSets < 1 or numSets > 2:
    print("Can only plot one or two datasets, but their number is ", numSets)
    sys.exit(1)

origPrefix = datasets[0]
origTag = labels[0]
if numSets == 2:
    optPrefix  = datasets[1]
    optTag  = labels[1]

f, ax = plt.subplots(len(Types), 3, sharex=True, sharey = False, 
                     figsize = (plot_size[0], plot_size[1]))

# Set up the legend in the upper right corner. We will have text in upper-left
plt.rcParams["legend.loc"] = 'upper right' 

# Set up the font for all elements
fs = 14
plt.rcParams.update({'font.size': fs})
plt.rc('axes', titlesize = fs)   # fontsize of the axes title
plt.rc('axes', labelsize = fs)   # fontsize of the x and y labels
plt.rc('xtick', labelsize = fs)  # fontsize of the tick labels
plt.rc('ytick', labelsize = fs)  # fontsize of the tick labels
plt.rc('legend', fontsize = fs)  # legend fontsize
plt.rc('figure', titlesize = fs) # fontsize of the figure title

# This tool can mix and match ASP Pinhole .tsai files and CSM frame/linescan .json files.
extensions = ['.tsai', '.json']

for row in range(len(Types)):
    s = Types[row]

    # Based on opt cameras find the original cameras. That because
    # maybe we optimized only a subset
    # Keep ref cams separate from actual cameras
    opt_cams = []
    if numSets == 2:
        all_opt_cams = sorted(multi_glob(optPrefix + s, extensions))
        ref_cams     = sorted(multi_glob(optPrefix + s + '-ref', extensions))
        camMap = set()
        # Add ref_cams to camMap set
        for c in ref_cams:
            camMap.add(c)
        # Add to opt_cams only those cameras that are not in camMap
        # This is to avoid adding ref_cams to opt_cams
        for c in all_opt_cams:
            if c not in camMap:
                opt_cams.append(c) 
    
    # Same for orig cams. Overwrite the earlier ref cams, if present,
    # as we will use the orig ref cams
    all_orig_cams = sorted(multi_glob(origPrefix + s, extensions))
    ref_cams      = sorted(multi_glob(origPrefix + s + '-ref', extensions))

    camMap = set()
    # Add ref_cams to camMap set
    for c in ref_cams:
        camMap.add(c)
    orig_cams = []
    # Add to orig_cams only those cameras that are not in camMap
    # This is to avoid adding ref_cams to orig_cams
    for c in all_orig_cams:
        if c not in camMap:
            orig_cams.append(c) 

    # Reduce the number of cameras to options.num_cameras
    orig_cams = getFirstN(orig_cams, options.num_cameras)
    ref_cams  = getFirstN(ref_cams, options.num_cameras)
    if numSets == 2:
        opt_cams  = getFirstN(opt_cams, options.num_cameras)
 
    # Check that these sets are the same size
    if len(orig_cams) != len(ref_cams):
        print("Number of input and reference cameras must be thee same. Got: ", \
             len(ref_cams), " and ", len(opt_cams))
        sys.exit(1)
    if numSets == 2 and len(orig_cams) != len(opt_cams):
        print("Number of cameras in both datasets must be the same. Got: ", \
            len(orig_cams), " and ", len(opt_cams))
        sys.exit(1)

    print("number of cameras for view " + s + ': ' + str(len(orig_cams)))

    # Read the rotations and convert them to NED
    (orig_rotation_angles, opt_rotation_angles) = read_angles(orig_cams, opt_cams, ref_cams)

    # Eliminate several first and last few values, based on options.trim_ratio
    if isLinescan(orig_cams[0]):
        totalNum = len(orig_rotation_angles)
        removeNum = int(options.trim_ratio * totalNum)
        removeNumBefore = int(removeNum / 2)
        removeNumAfter = removeNum - removeNumBefore
        b = removeNumBefore
        e = totalNum - removeNumAfter
        orig_rotation_angles = orig_rotation_angles[b:e]
        if numSets == 2:
            opt_rotation_angles = opt_rotation_angles[b:e]
        print("Plotting the most central %d out of %d poses for linescan cameras." % \
            (len(orig_rotation_angles), totalNum))  

    # The order is roll, pitch, yaw, as returned by
    # R.from_matrix().as_euler('XYZ',degrees=True)
    orig_roll  = [r[0] for r in orig_rotation_angles]
    orig_pitch = [r[1] for r in orig_rotation_angles]
    orig_yaw   = [r[2] for r in orig_rotation_angles]
    opt_roll   = [r[0] for r in opt_rotation_angles]
    opt_pitch  = [r[1] for r in opt_rotation_angles]
    opt_yaw    = [r[2] for r in opt_rotation_angles]

    residualTag = ''
    if options.subtract_line_fit:
        residualTag = ' residual'
        fit_roll = poly_fit(np.array(range(len(orig_roll))), orig_roll)
        fit_pitch = poly_fit(np.array(range(len(orig_pitch))), orig_pitch)
        fit_yaw = poly_fit(np.array(range(len(orig_yaw))), orig_yaw)

        orig_roll = orig_roll - fit_roll
        orig_pitch = orig_pitch - fit_pitch
        orig_yaw = orig_yaw - fit_yaw
        if numSets == 2:
            opt_roll = opt_roll - fit_roll
            opt_pitch = opt_pitch - fit_pitch
            opt_yaw = opt_yaw - fit_yaw

    # Tag for the title
    t = s 
    
    fmt = "{:.2e}" # 2 digits of precision are enough for display 
    orig_roll_std = fmt.format(np.std(orig_roll))
    orig_pitch_std = fmt.format(np.std(orig_pitch))
    orig_yaw_std = fmt.format(np.std(orig_yaw))
    print(origTag + " " + t + " roll std: " + orig_roll_std + " degrees")
    print(origTag + " " + t + " pitch std: " + orig_pitch_std + " degrees")
    print(origTag + " " + t + " yaw std: " + orig_yaw_std + " degrees")
    if numSets == 2:
        opt_roll_std = fmt.format(np.std(opt_roll))
        opt_pitch_std = fmt.format(np.std(opt_pitch))
        opt_yaw_std = fmt.format(np.std(opt_yaw))
        print(optTag + " " + t + " roll std: " + opt_roll_std + " degrees")
        print(optTag + " " + t + " pitch std: " + opt_pitch_std + " degrees")
        print(optTag + " " + t + " yaw std: " + opt_yaw_std + " degrees")

    # Find the handle to the axis object for the current row
    if len(ax.shape) == 1:
        A = ax # otherwise get an indexing error
    else:
        A = ax[row]

    # Plot residuals
    A[0].plot(np.arange(len(orig_roll)), orig_roll, label=origTag, color = 'r')
    A[1].plot(np.arange(len(orig_pitch)), orig_pitch, label=origTag, color = 'r')
    A[2].plot(np.arange(len(orig_yaw)), orig_yaw, label=origTag, color = 'r')
    if numSets == 2:
        A[0].plot(np.arange(len(opt_roll)), opt_roll, label=optTag, color = 'b')
        A[1].plot(np.arange(len(opt_pitch)), opt_pitch, label=optTag, color = 'b')
        A[2].plot(np.arange(len(opt_yaw)), opt_yaw, label=optTag, color = 'b')

    A[0].set_title(t + ' roll'  + residualTag)
    A[1].set_title(t + ' pitch' + residualTag)
    A[2].set_title(t + ' yaw '  + residualTag)

    A[0].set_ylabel('Degrees')
    #A[1].set_ylabel('Degrees') # don't repeat this as it takes space
    #A[2].set_ylabel('Degrees')

    for index in range(3):
        A[index].set_xlabel('Frame index')
        # Calc stddev text
        if numSets == 1:
            txt = 'StDev:' + orig_roll_std
        else: 
            txt = 'StDev before/after:' + orig_roll_std + ", " + opt_roll_std
        # Add stdev values as text
        A[index].text(0.05, 0.05, txt,
            va='top', color='k', transform=A[index].transAxes, fontsize=fs)    
        # legend
        A[index].legend()
        # Se the font size
        ac = A[index]
        for item in ([ac.title, ac.xaxis.label, ac.yaxis.label] +
                     ac.get_xticklabels() + ac.get_yticklabels()):
          item.set_fontsize(fs)

plt.tight_layout()
plt.show()


