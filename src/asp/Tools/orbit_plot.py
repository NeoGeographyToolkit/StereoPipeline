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

# Plot roll, pitch, and yaw of ASP Pinhole .tsai cameras, and/or of CSM #
# Frame/Linescan .json cameras, before and after bundle adjustment.

# The naming convention used is that Forward images and cameras are # named like
#${prefix}f-00000.tif, ${prefix}f-00000.tsai, and analogously for Nadir and # Aft
#(use 'n' and 'a'). If there is a dash before "f", it must be part of the prefix.
# For CSM cameras, the extension is .json instead of .tsai.

# Also can handle any other single-character above, such as '1', '2', etc.
# Ensure these entries are used as part of the 'types' variable below.

# To get cameras named this way, can create symlinks from original data.

# For every camera before bundle-adjustment named a-10000.tsai, there must be a
# reference camera named a-ref-10000.tsai. This is the camera that will be used
# to convert from ECEF to NED coordinates. This camera is created by sat_sim #
#with the option --save-ref-cams. 

# This tool works the same way for linescan cameras. Then, instead of many .tsai
# files there exists a single .json file for every orbital segment. 

# Inputs

# numCams=1000 # how many cameras to plot (if a large number is used, plot all)
#types="a,f,n" # Plot only given types. Use any strings separated by commas.
#beforeOpt="inputs/" # Cameras must be ${beforeOpt}type*{.tsai,.json}, with type as above.
#afterOpt="ba/run-" # Cameras must be ${afterOpt}type*{.tsai,.json} 
# beforeCaption="PlanetOrig"
# afterCaption="BundleAdjust"
# subtractLineFit=1 # if to subtract the best line fit before plotting (same fit for before/after).

# trimRatio=0.5. Trim ratio. If not set, assume 0. Given a value between 0 and 1
# (inclusive), remove this fraction of camera poses from each sequence, with
# half of this amount for poses at the beginning and half at the end of the
# sequence. This is usually set to 0.5 for linescan, as then the total number of
# cameras is double the amount needed to cover the image, with the extra cameras
# making it easier to solve for jitter and interpolate the camera poses.
# This parameter is not used for frame cameras.

# Usage:

# python plot_orient.py $numCams $types $beforeOpt $afterOpt \
#    $beforeCaption $afterCaption $subtractLineFit $TrimRatio

# This tool needs python 3, and a handful of python packages, as seen below.
# Install them conda.

import sys, os, re, math, json, argparse, shutil, glob
import numpy as np
import matplotlib.pyplot as plt
from pyproj import Proj, transform, Transformer
from scipy.spatial.transform import Rotation as R

usage  = "python orbit_plot.py <options>"

parser = argparse.ArgumentParser(usage=usage,
                                 formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument('--image',  dest = 'image', default = "", 
                    help='The single-channel image to use to find the water-land threshold.')

parser.add_argument("--num-samples",  dest="num_samples", type=int, default = 1000000,
                    help="The number of samples to pick from the image (more samples " +
                    "will result in more accuracy but will be slower).")

parser.add_argument("--no-plot", action="store_true", default=False,
                        dest="no_plot",  help="Do not show the plot.")

(options, args) = parser.parse_known_args(sys.argv)

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

    return rotations

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

def isLinenscan(cam_file):
    """
    Read the first line from cam_file which tells if the sensor is linescan.
    """
    lineScan = False
    with open(cam_file, 'r') as f:
        line = f.readline()
        if 'LINE_SCAN' in line:
            lineScan = True
    
    return lineScan

def ned_rotation_from_cam(cam_file, ref_cam_file):
    #coordinate conversion step
    #from pyproj import Transformer
    #ecef_proj = 'EPSG:4978'
    #geo_proj = 'EPSG:4326'
    #ecef2wgs = Transformer.from_crs(ecef_proj,geo_proj)
    
    # Read the first line from cam_file
    lineScan = isLinenscan(cam_file)

    rotations = []
    ref_rotations = []

    if lineScan:
        # Read linescan data
        rotations = read_linescan_csm_cam(cam_file)
        ref_rotations = read_linescan_csm_cam(ref_cam_file)
    else:   
        # read Pinhole (Frame) files in ASP .tsai or CSM .json format
        asp_dict = read_frame_cam_dict(cam_file)
        ref_asp_dict = read_frame_cam_dict(ref_cam_file)
        # get camera rotation
        rot_mat = asp_dict['rotation_matrix']
        ref_rot_mat = ref_asp_dict['rotation_matrix']
        rotations.append(rot_mat)
        ref_rotations.append(ref_rot_mat)
    
    # rotate about z axis by 90 degrees
    # This will be a problem for non-sat_sim cameras
    T = np.zeros((3,3),float)
    T[0, 1] = 1
    T[1, 0] = -1
    T[2, 2] = 1
    Tinv = np.linalg.inv(T)

    # Sanity check
    if len(rotations) != len(ref_rotations):
        raise Exception("Number of rotations (%d) and reference rotations (%d) do not match. This is likely due to orientations being resampled in jitter_solve." % (len(rotations), len(ref_rotations)))

    angles = []
    for i in range(len(rotations)):
        rot_mat = rotations[i]
        ref_rot_mat = ref_rotations[i]

        inv_ref_rot_mat = np.linalg.inv(ref_rot_mat)
        N = np.matmul(inv_ref_rot_mat, rot_mat)

        angles.append(R.from_matrix(np.matmul(N, Tinv)).as_euler('XYZ',degrees=True))

    return angles

# Return at most this many elements from an array
def getFirstN(arr, N):
    if len(arr) > N:
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

# Main function

if len(sys.argv) < 8:
    print("Usage: " + argv.sys[0] + " <num> <camera types> <orig prefix> <opt prefix> <orig tag> <opt tag> <subtract line fit> [<line scan>]")
    sys.exit(1)

Num   = int(sys.argv[1]) # How many to plot
Types = sys.argv[2] # camera types, can be 'n', 'f,n,a', etc.

# Assume cameras are named ${origPrefix}n1352.tsai and same for optPrefix
origPrefix = sys.argv[3]
optPrefix  = sys.argv[4]

origTag = sys.argv[5]
optTag = sys.argv[6]

subtractLineFit = int(sys.argv[7])

trimRatio = 0.0
if len(sys.argv) > 8:
    trimRatio = float(sys.argv[8])

# Split by comma
Types = Types.split(',')

print("Camera types are: ", ','.join(Types))
print("orig prefix ", origPrefix)
print("opt prefix ", optPrefix)
print("Subtract line fit: ", subtractLineFit)
print("Trim ratio (for linescan): ", trimRatio)

# Below ensure we have at least two plots, or else ax[i][j] will fail below
f, ax = plt.subplots(max(2, len(Types)), 3, sharex=True, sharey = False, figsize = (15, 15))

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

count = -1
for s in Types:

    count += 1

    # Based on opt cameras find the original cameras. That because
    # maybe we optimized only a subset
    # Keep ref cams separate from actual cameras
    all_opt_cams = sorted(multi_glob(optPrefix + s, extensions))
    ref_cams     = sorted(multi_glob(optPrefix + s + '-ref', extensions))
    camMap = set()
    # Add ref_cams to camMap set
    for c in ref_cams:
        camMap.add(c)
    opt_cams = []
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

    #print ("orig cams are ", orig_cams)
    #print ("opt cams are ", opt_cams)
    #print ("ref cams are ", ref_cams)

    # Reduce the number of cameras to Num
    orig_cams = getFirstN(orig_cams, Num)
    opt_cams  = getFirstN(opt_cams, Num)
    ref_cams  = getFirstN(ref_cams, Num)
 
    # Check that these sets are the same size
    if len(orig_cams) != len(opt_cams):
        print("Number of orig and opt cameras must be the same, got",  len(orig_cams), " and ", len(opt_cams))
        sys.exit(1)
    if len(ref_cams) != len(opt_cams):
        print("Number of ref and opt cameras must be the same, got",  len(ref_cams), " and ", len(opt_cams))
        sys.exit(1)

    print("number of cameras for view " + s + ': ' + str(len(orig_cams)))

    # Get rotations, then convert to NED. For linescan we will have a single
    # camera, but with many poses in it. For Pinhole we we will have many
    # cameras, each with a single pose. That's why the loops below. 
    orig_rotation_angles = []
    I = range(len(orig_cams))
    opt_rotation_angles = []
    for i in I:
        angles = ned_rotation_from_cam(orig_cams[i], ref_cams[i])
        for angle in angles:
            orig_rotation_angles.append(angle)
    for i in I:
        angles = ned_rotation_from_cam(opt_cams[i], ref_cams[i])
        for angle in angles:
            opt_rotation_angles.append(angle)

    lineScan = isLinenscan(orig_cams[0])

    # Eliminate the first and last few values, based on trimRatio
    if lineScan:
        totalNum = len(orig_rotation_angles)
        removeNum = int(trimRatio * totalNum)
        removeNumBefore = int(removeNum / 2)
        removeNumAfter = removeNum - removeNumBefore
        b = removeNumBefore
        e = totalNum - removeNumAfter
        orig_rotation_angles = orig_rotation_angles[b:e]
        opt_rotation_angles = opt_rotation_angles[b:e]
        print("Plotting the most central %d out of %d poses for linescan cameras." % \
            (len(orig_rotation_angles), totalNum))  

    # The order is roll, pitch, yaw, as returned by R.from_matrix().as_euler('XYZ',degrees=True)
    orig_roll  = [r[0] for r in orig_rotation_angles]
    orig_pitch = [r[1] for r in orig_rotation_angles]
    orig_yaw   = [r[2] for r in orig_rotation_angles]
    opt_roll  = [r[0] for r in opt_rotation_angles]
    opt_pitch = [r[1] for r in opt_rotation_angles]
    opt_yaw   = [r[2] for r in opt_rotation_angles]

    residualTag = ''
    if subtractLineFit:
        fit_roll = poly_fit(np.array(range(len(orig_roll))), orig_roll)
        fit_pitch = poly_fit(np.array(range(len(orig_pitch))), orig_pitch)
        fit_yaw = poly_fit(np.array(range(len(orig_yaw))), orig_yaw)

        orig_roll = orig_roll - fit_roll
        orig_pitch = orig_pitch - fit_pitch
        orig_yaw = orig_yaw - fit_yaw
        
        opt_roll = opt_roll - fit_roll
        opt_pitch = opt_pitch - fit_pitch
        opt_yaw = opt_yaw - fit_yaw

        residualTag = ' residual'

    t = s # if no match below, use the same string
    if s == 'a':
        t = 'aft'
    if s == 'n':
        t = 'nadir'
    if s == 'f':
        t = 'fwd'
    
    fmt = "{:.2e}" 
    print("format is", fmt)
    orig_roll_std = fmt.format(np.std(orig_roll))
    orig_pitch_std = fmt.format(np.std(orig_pitch))
    orig_yaw_std = fmt.format(np.std(orig_yaw))

    opt_roll_std = fmt.format(np.std(opt_roll))
    opt_pitch_std = fmt.format(np.std(opt_pitch))
    opt_yaw_std = fmt.format(np.std(opt_yaw))

    print(origTag + " " + t + " roll std: " + orig_roll_std + " degrees")
    print(origTag + " " + t + " pitch std: " + orig_pitch_std + " degrees")
    print(origTag + " " + t + " yaw std: " + orig_yaw_std + " degrees")

    print(optTag + " " + t + " roll std: " + opt_roll_std + " degrees")
    print(optTag + " " + t + " pitch std: " + opt_pitch_std + " degrees")
    print(optTag + " " + t + " yaw std: " + opt_yaw_std + " degrees")

    # Plot residuals
    ax[count][0].plot(np.arange(len(orig_roll)), orig_roll, label=origTag, color = 'r')
    ax[count][0].plot(np.arange(len(opt_roll)), opt_roll, label=optTag, color = 'b')

    ax[count][1].plot(np.arange(len(orig_pitch)), orig_pitch, label=origTag, color = 'r')
    ax[count][1].plot(np.arange(len(opt_pitch)), opt_pitch, label=optTag, color = 'b')

    ax[count][2].plot(np.arange(len(orig_yaw)), orig_yaw, label=origTag, color = 'r')
    ax[count][2].plot(np.arange(len(opt_yaw)), opt_yaw, label=optTag, color = 'b')

    ax[count][0].set_title(t + ' roll'  + residualTag)
    ax[count][1].set_title(t + ' pitch' + residualTag)
    ax[count][2].set_title(t + ' yaw '  + residualTag)

    ax[count][0].set_ylabel('Degrees')
    #ax[count][1].set_ylabel('Degrees') # don't repeat this as it takes space
    #ax[count][2].set_ylabel('Degrees')

    ax[count][0].set_xlabel('Frame index')
    ax[count][1].set_xlabel('Frame index')
    ax[count][2].set_xlabel('Frame index')
    
    # Add stdev values as text
    ax[count][0].text(0.05, 0.05, 
     'StDev before/after:' + orig_roll_std + ", " + opt_roll_std, 
        va='top', color='k', transform=ax[count][0].transAxes, fontsize=fs)    
    ax[count][1].text(0.05, 0.05, 
        'StDev before/after:' + orig_pitch_std + ", " + opt_pitch_std,
        va='top', color='k', transform=ax[count][1].transAxes, fontsize=fs)    
    ax[count][2].text(0.05, 0.05,
        'StDev before/after:' + orig_yaw_std + ", " + opt_yaw_std,
        va='top', color='k', transform=ax[count][2].transAxes, fontsize=fs)    

    ax[count][0].legend()
    ax[count][1].legend()
    ax[count][2].legend()

    for index in range(3):
        ac = ax[count][index]
        for item in ([ac.title, ac.xaxis.label, ac.yaxis.label] +
             ac.get_xticklabels() + ac.get_yticklabels()):
          item.set_fontsize(fs)

plt.tight_layout()
plt.show()


