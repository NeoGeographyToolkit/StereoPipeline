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

# TODO(oalexan1): Fix this to be aware of the fact that the Earth is not a sphere.
# See the relevant WV code.
def produce_m(lon, lat, m_meridian_offset=0):
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

def estim_satellite_orientation(positions):
    """
    Given a list of satellite positions, estimate the satellite
    orientation at each position. The x axis is the direction of
    motion, z points roughly down while perpendicular to x, and
    y is the cross product of z and x.
    """
    num = len(positions)
    rotations = []
    for i in range(num):
        prev_i = i - 1
        if prev_i < 0:
            prev_i = 0
        next_i = i + 1
        if next_i >= num:
            next_i = num - 1

        # x is tangent to orbit, z goes down    
        x = np.array(positions[next_i]) - np.array(positions[prev_i])
        z = -np.array(positions[i])

        # Normalize
        z = z / np.linalg.norm(z)
        x = x / np.linalg.norm(x)

        # Make sure z is perpendicular to x
        z = z - np.dot(z, x) * x
        z = z / np.linalg.norm(z)

        # Find y as the cross product
        y = np.cross(z, x)

        # Make these as columns in a matrix r
        r = np.column_stack((x, y, z))
        rotations.append(r)
    
    return rotations

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
    tsai_dict = {
      'camera': camera, 
      'focal_length': (fu, fv),
      'optical_center': (cu, cv), 
      'cam_cen_ecef': cam_cen, 
      'rotation_matrix': rot_mat, 
      'pitch': pitch}
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
    positions_vec = np.reshape(positions_vec, (-1, 3))

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

    # Rotate about z axis by 90 degrees. This must be synched up with 
    # sat_sim. This will be a problem for non-sat_sim cameras.
    T = np.zeros((3,3),float)
    T[0, 1] = -1
    T[1, 0] = 1
    T[2, 2] = 1
    Tinv = np.linalg.inv(T)

    inv_ref_rot_mat = np.linalg.inv(ref_rot_mat)
    N = np.matmul(inv_ref_rot_mat, rot_mat)

    return R.from_matrix(np.matmul(N, Tinv)).as_euler('XYZ', degrees=True)

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

# Read a list. Return only the files that match the given pattern and have the given extension.
def read_list(list_file, pattern, extensions):
    files = []
    with open(list_file, 'r') as f:
        for line in f:
            line = line.strip()
            for ext in extensions:
              if pattern in line and line.endswith(ext):
                files.append(line)
    return files
    
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

# Read the positions and rotations from the given files
def read_positions_rotations(cams):

  (positions, rotations) = ([], [])
  for i in range(len(cams)):
      (p, r) = read_positions_rotations_from_file(cams[i])
      positions += p
      rotations += r

  # Must have as many rotations as positions. That is needed as later
  # we build ref rotations from positions.
  if len(rotations) != len(positions):
    print("Number of camera positions and orientations must be the same.")
    sys.exit(1)

  return (positions, rotations)

def exclude_ref_cams(all_cams, ref_cams):

    cams = []
    camMap = set()
    # Add ref_cams to camMap set
    for c in ref_cams:
        camMap.add(c)
    
    for c in all_cams:
        if c not in camMap:
            cams.append(c) 
    
    return cams

# Get rotations, then convert to NED.  That's why the loops below. 
def read_angles(orig_cams, opt_cams, ref_cams):

  # orig_cams and ref_cams must be the same size
  if len(ref_cams) > 0 and len(orig_cams) != len(ref_cams):
      print("Number of input and reference cameras must be the same. Got: ", \
           len(ref_cams), " and ", len(opt_cams))
      sys.exit(1)

  (orig_positions, orig_rotations) = read_positions_rotations(orig_cams)
  (opt_positions, opt_rotations)   = read_positions_rotations(opt_cams)
  (ref_positions, ref_rotations)   = read_positions_rotations(ref_cams)
  
  # If we do not have ref cameras that determine the satellite orientation,
  # estimate them from the camera positions
  if len(ref_cams) == 0:
    orig_ref_rotations = estim_satellite_orientation(orig_positions)
    opt_ref_rotations  = estim_satellite_orientation(opt_positions)
  else:
    orig_ref_rotations = ref_rotations[:]
    opt_ref_rotations  = ref_rotations[:]

  orig_rotation_angles = []
  opt_rotation_angles = []
  for i in range(len(orig_rotations)):
      angles = roll_pitch_yaw(orig_rotations[i], orig_ref_rotations[i])
      orig_rotation_angles.append(angles)
  for i in range(len(opt_rotations)):
      angles = roll_pitch_yaw(opt_rotations[i], opt_ref_rotations[i])
      opt_rotation_angles.append(angles)

  return (orig_rotation_angles, opt_rotation_angles)

# Logic for when we have N datasets to plot, not just one and two.
# TODO(oalexan1): This function must replace the above. This was not tested.
# Then rewrite the rest of the code to use an array of arrays instead of two arrays.
# def read_angles(camSet, ref_cams): 

#     if len(ref_cams) > 0:
#         # Check length of each array in 'camSet' against reference
#         ref_cams_length = len(ref_cams)  # Store the reference length once
#         for cam in camSet:
#             if len(cam) != ref_cams_length:
#                 print(f"Number of reference cameras input cameras do not match.")
#                 sys.exit(1)

#     # If we do not have ref cameras that determine the satellite orientation,
#     # estimate them from the camera positions
#     refRotationsSet = []
#     for cam in camSet:
#         # First the case of length 0 for ref_cams
#         if len(ref_cams) == 0:
#             refRotationsSet.append(estim_satellite_orientation(cam))
#         else:
#             refRotationsSet.append(ref_cams) 
    
#     # Read the positions and rotations   
#     positions = []
#     rotations = []
#     for cam in camSet:
#         cam_positions, cam_rotations = read_positions_rotations(cam)
#         positions.append(cam_positions)
#         rotations.append(cam_rotations)
#     (ref_positions, ref_rotations) = read_positions_rotations(ref_cams)
    
#     rotation_angles = []  # Store angles for all camera arrays
#     for j in range(len(rotations)):  # Outer loop over cameras sets
#         camera_angles = []  # Store angles for a single camera set
#         for i in range(len(rotations[j])):  # Inner loop over rotations within a set
#             angles = roll_pitch_yaw(rotations[j][i], refRotationsSet[j][i])
#             camera_angles.append(angles)
#         rotation_angles.append(camera_angles)
        
#     return rotation_angles 

def err_fun(vals, opt):
  ''' Find the standard deviation or RMSE of the given values. '''
  if opt.use_rmse:
    return np.sqrt(np.multiply(vals, vals).mean())
  return np.std(vals)
                
# Load and plot each row in the figure given by 'ax'
def plot_row(ax, row, orbits, hasList, datasets, orbit_labels, dataset_labels,
             ref_list, opt):

  # We assume we have one or two datasets that we want to plot on top of each other.
  numSets = len(datasets)
  if numSets < 1:
    print("Must specify at least one dataset.")
    sys.exit(1)

  origPrefix = datasets[0]
  origTag = dataset_labels[0]
  optPrefix = ""
  optTag = ""
  if numSets == 2:
      optPrefix  = datasets[1]
      optTag  = dataset_labels[1]

  camType = orbits[row]
  camLabel = orbit_labels[row]

  # This tool can mix and match ASP Pinhole .tsai files and CSM frame/linescan .json files.
  extensions = ['.tsai', '.json']

  # Read the opt cameras and their ref cameras. The latter may not exist as
  # bundle adjusted does not create them. We will use the ref cams for orig cams.
  opt_cams = []
  ref_cams = []
  print_ref_cam_warning = False
  if numSets == 2:
    if hasList:
      opt_cams = read_list(optPrefix, camType, extensions)
      ref_cams = []
      if ref_list != "":
        ref_cams = read_list(ref_list, camType, extensions)
    else:
      all_opt_cams = sorted(multi_glob(optPrefix + camType, extensions))
      ref_cams     = sorted(multi_glob(optPrefix + camType + '-ref', extensions))
      opt_cams     = exclude_ref_cams(all_opt_cams, ref_cams)
      if (not opt.use_ref_cams) and len(ref_cams) > 0:
          print_ref_cam_warning = True

  # Same for orig cams. Overwrite the earlier ref cams, if present,
  # as we will use the orig ref cams
  if hasList:
    orig_cams = read_list(origPrefix, camType, extensions)
    ref_cams = []
    if ref_list != "":
      ref_cams = read_list(ref_list, camType, extensions)

  else: 
    all_orig_cams = sorted(multi_glob(origPrefix + camType, extensions))
    ref_cams      = sorted(multi_glob(origPrefix + camType + '-ref', extensions))
    orig_cams     = exclude_ref_cams(all_orig_cams, ref_cams)
  
  if (not opt.use_ref_cams) and len(ref_cams) > 0:
      print_ref_cam_warning = True

  # If not using ref cams, wipe them
  if not opt.use_ref_cams:
      if (print_ref_cam_warning):
          print("Found reference cameras but will not use them.")
      ref_cams = []

  # Reduce the number of cameras to opt.num_cameras
  orig_cams = getFirstN(orig_cams, opt.num_cameras)
  if opt.use_ref_cams:
      ref_cams  = getFirstN(ref_cams, opt.num_cameras)
  if numSets == 2:
      opt_cams  = getFirstN(opt_cams, opt.num_cameras)

  # Check that these sets are the same size
  if opt.use_ref_cams and len(orig_cams) != len(ref_cams):
      print("Number of input and reference cameras must be thee same. See the option --use-ref-cams for more info. For these numbers, got: ", \
              len(ref_cams), " and ", len(orig_cams))
      sys.exit(1)
  if numSets == 2 and opt.use_ref_cams and len(orig_cams) != len(opt_cams):
      print("Number of cameras in both datasets must be the same when using " + \
        "reference cameras. Got: ", len(orig_cams), " and ", len(opt_cams))
      sys.exit(1)

  print("Number of cameras for view " + camType + ': ' + str(len(orig_cams)))

  # Read the rotations and convert them to roll, pitch, yaw
  (orig_rotation_angles, opt_rotation_angles) = read_angles(orig_cams, opt_cams, ref_cams)

  # Eliminate several first and last few values, based on opt.trim_ratio
  if isLinescan(orig_cams[0]):
      totalNum = len(orig_rotation_angles)
      removeNum = int(opt.trim_ratio * totalNum)
      removeNumBefore = int(removeNum / 2)
      removeNumAfter = removeNum - removeNumBefore
      b = removeNumBefore
      e = totalNum - removeNumAfter
      orig_rotation_angles = orig_rotation_angles[b:e]
      if numSets == 2:
          opt_rotation_angles = opt_rotation_angles[b:e]
      print("Plotting the most central %d out of %d poses for linescan cameras." % \
          (len(orig_rotation_angles), totalNum))  
  
  if numSets == 2:
    # Must check that we get same length as for orig rotations
    # Print here the length of opt_rotation_angles
    if len(opt_rotation_angles) != len(orig_rotation_angles):
      print("The sizes of the two input datasets do not agree. " + \
            "Got: ", len(opt_rotation_angles), " and ", len(orig_rotation_angles))
      sys.exit(1)

  # The order is roll, pitch, yaw, as returned by
  # R.from_matrix().as_euler('XYZ',degrees=True)
  orig_roll  = [r[0] for r in orig_rotation_angles]
  orig_pitch = [r[1] for r in orig_rotation_angles]
  orig_yaw   = [r[2] for r in orig_rotation_angles]
  opt_roll   = [r[0] for r in opt_rotation_angles]
  opt_pitch  = [r[1] for r in opt_rotation_angles]
  opt_yaw    = [r[2] for r in opt_rotation_angles]

  residualTag = ''
  if opt.subtract_line_fit:
      # Same line fit will be subtracted from all datasets
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

  fmt = "{:.2e}" # 2 digits of precision are enough for display 
  orig_roll_err = fmt.format(err_fun(orig_roll, opt))
  orig_pitch_err = fmt.format(err_fun(orig_pitch, opt))
  orig_yaw_err = fmt.format(err_fun(orig_yaw, opt))
  err_str = 'StDev: '
  if opt.use_rmse:
        err_str = ' RMSE: '
  print(origTag + " " + camType + " roll " + err_str + orig_roll_err + " degrees")
  print(origTag + " " + camType + " pitch " + err_str + orig_pitch_err + " degrees")
  print(origTag + " " + camType + " yaw " + err_str + orig_yaw_err + " degrees")
  (opt_roll_err, opt_pitch_err, opt_yaw_err) = (0, 0, 0) # initialize
  if numSets == 2:
      opt_roll_err = fmt.format(err_fun(opt_roll, opt))
      opt_pitch_err = fmt.format(err_fun(opt_pitch, opt))
      opt_yaw_err = fmt.format(err_fun(opt_yaw, opt))
      print(optTag + " " + camType + " roll " + err_str + opt_roll_err + " degrees")
      print(optTag + " " + camType + " pitch " + err_str + opt_pitch_err + " degrees")
      print(optTag + " " + camType + " yaw " + err_str + opt_yaw_err + " degrees")

  # Find the handle to the axis object for the current row
  if len(ax.shape) == 1:
      A = ax # otherwise get an indexing error
  else:
      A = ax[row]

  # Plot residuals
  lw = opt.line_width
  A[0].plot(np.arange(len(orig_roll)), orig_roll, label=origTag, color = 'r', linewidth = lw)
  A[1].plot(np.arange(len(orig_pitch)), orig_pitch, label=origTag, color = 'r', 
            linewidth = lw)
  A[2].plot(np.arange(len(orig_yaw)), orig_yaw, label=origTag, color = 'r', linewidth = lw)
  if numSets == 2:
      A[0].plot(np.arange(len(opt_roll)), opt_roll, label=optTag, color = 'b', linewidth = lw)
      A[1].plot(np.arange(len(opt_pitch)), opt_pitch, label=optTag, color = 'b', 
                linewidth = lw)
      A[2].plot(np.arange(len(opt_yaw)), opt_yaw, label=optTag, color = 'b', linewidth = lw)

  A[0].set_title(camLabel + ' roll'  + residualTag)
  A[1].set_title(camLabel + ' pitch' + residualTag)
  A[2].set_title(camLabel + ' yaw '  + residualTag)

  A[0].set_ylabel('Degrees')
  #A[1].set_ylabel('Degrees') # don't repeat this as it takes space
  #A[2].set_ylabel('Degrees')

  err = ((orig_roll_err, opt_roll_err), (orig_pitch_err, opt_pitch_err), (orig_yaw_err, opt_yaw_err))
  for index in range(3):
      A[index].set_xlabel('Frame index')
      # Calc err text
      if numSets == 1:
          txt = err_str + err[index][0]
      else: 
          txt = err_str + err[index][0] + ", " + err[index][1]
      # Add err values as text
      A[index].text(0.05, 0.05, txt,
          va='top', color='k', transform=A[index].transAxes, fontsize=fs)    
      # legend
      A[index].legend()
      # Se the font size
      ac = A[index]
      for item in ([ac.title, ac.xaxis.label, ac.yaxis.label] +
                      ac.get_xticklabels() + ac.get_yticklabels()):
          item.set_fontsize(fs)

# Main function. Set up the arguments.
usage = "python orbit_plot.py <options>\n"

parser = argparse.ArgumentParser(usage=usage,
                                 formatter_class=argparse.RawTextHelpFormatter)

parser.add_argument('--dataset', dest = 'dataset', default = '',
                    help='The dataset to plot. Only one or two datasets are supported '
                    '(for example, before and after optimization). Each dataset can have '
                    'several types of images, given by --orbit-id. The dataset is the prefix '
                    'of the cameras, such as "cameras/" or "opt/run-". It is to be followed '
                    'by the orbit id, such as, "nadir" or "aft". If more than one dataset, '
                    'they will be plotted on top of each other.')

parser.add_argument('--orbit-id', dest = 'orbit_id', default = '',
                    help='The id (a string) that determines an orbital group of ' + 
                    'cameras. If more than one, separate them by comma, with no ' + 
                    'spaces in between.') 

parser.add_argument('--dataset-label', dest = 'dataset_label', default = '',
                    help='The label to use for each dataset in the legend. If more than ' + 
                    'one, separate them by comma, with no spaces in between. If not '     + 
                    'set, will use the dataset name.')

parser.add_argument('--list', dest = 'list', default = '',
                    help='Instead of specifying --dataset, load the cameras listed '     + 
                    'in this file (one per line). Only the names matching --orbit-id '   + 
                    'will be read. If more than one list, separate them by comma, with ' + 
                    'no spaces in between.')

parser.add_argument('--ref-list', dest = 'ref_list', default = '',
                    help='When --list is specified, read the ref cams from here.')

parser.add_argument('--orbit-label', dest = 'orbit_label', default = '',
                    help='The label to use for each orbital group (will be shown as '
                    'part of the title). If more than one, separate them by comma, with ' + 
                    'no spaces in between. If not set, will use the orbit id.')

parser.add_argument('--use-ref-cams', dest = 'use_ref_cams', action='store_true',
                    help='Read from disk reference cameras that determine the satellite ' + 
                    'orientation. This assumes the first dataset was created with '       + 
                    'sat_sim with the option --save-ref-cams. Otherwise do not use '      + 
                    'this option. In that case the satellite orientation is estimated '   + 
                    'based on camera positions.') 

parser.add_argument('--subtract-line-fit', dest = 'subtract_line_fit', action='store_true',
                    help='If set, subtract the best line fit from the curves being '      + 
                    'plotted. If more than one dataset is present, the same line ' +
                    'fit (for the first one) will be subtracted from all of them. '     +
                    'This is useful for inspecting subtle changes.')

parser.add_argument('--use-rmse', dest = 'use_rmse', action='store_true',
                    help='Compute and display the root mean square error (RMSE) ' +
                    'rather than the standard deviation. This is useful when a ' +
                    'systematic shift is present. See also --subtract-line-fit.')

parser.add_argument('--num-cameras',  dest='num_cameras', type=int, default = -1,
                    help='Plot only the first this many cameras from each orbital ' + 
                    'sequence. By default, plot all of them.')

parser.add_argument('--trim-ratio',  dest='trim_ratio', type=float, default = 0.0,
                    help='Trim ratio. Given a value between 0 and 1 (inclusive), '        + 
                    'remove this fraction of camera poses from each sequence, with half ' + 
                    'of this amount for poses at the beginning and half at the end of '   + 
                    'the sequence. This is used only for linescan cameras, to not plot '  + 
                    'camera poses beyond image lines. For cameras created with sat_sim, ' + 
                    'a value of 0.5 should be used.')

parser.add_argument('--figure-size', dest = 'figure_size', default = '15,15',
                    help='Specify the width and height of the figure having the plots, ' + 
                    'in inches. Use two numbers with comma as separator (no spaces).')

parser.add_argument('--title', dest = 'title', default = '',
                    help='Set this as the figure title, to be shown on top of all plots.')

parser.add_argument('--line-width', dest = 'line_width', type=float, default = 1.5,
                    help='Line width for the plots.')

parser.add_argument('--font-size', dest = 'font_size', type=int, default = 14,
                    help='Font size for the plots.')

(opt, args) = parser.parse_known_args(sys.argv)

# Throw an error if not all args have been parsed
if len(args) > 1:
    print("Not all arguments were parsed. Unprocessed values:", args[1:])
    sys.exit(1)

if opt.dataset == "" and opt.list == "": 
    print("Must set --dataset or --list.")
    parser.print_help()
    sys.exit(1)

if opt.dataset != "" and opt.list != "": 
    print("Cannot set both --dataset and --list.")
    parser.print_help()
    sys.exit(1)

hasList = False
if opt.dataset == "":
    # If the dataset label is not set, use the list
    hasList = True
    opt.dataset = opt.list

if hasList and opt.use_ref_cams and opt.ref_list == "":
    print("Must set --ref-list when using --use-ref-cams with --list.")
    parser.print_help()
    sys.exit(1)

if opt.orbit_id == "":
    print("Must set --orbit-id.")
    parser.print_help()
    sys.exit(1)

if opt.trim_ratio < 0:
    print("The value of --trim-ratio must be non-negative.")
    parser.print_help()
    sys.exit(1)

if opt.line_width <= 0:
    print("The value of --line-width must be positive.")
    parser.print_help()
    sys.exit(1)

if opt.font_size <= 0:
    print("The value of --font-size must be positive.")
    parser.print_help()
    sys.exit(1)

# Parse the datasets and their labels. Split by comma.
datasets = opt.dataset.split(',')
dataset_labels = opt.dataset_label.split(',')
if len(dataset_labels) == 0 or (len(dataset_labels) == 1 and dataset_labels[0] == ""):
    dataset_labels = datasets[:]
if len(dataset_labels) != len(datasets):
    print("Number of datasets/lists and dataset_labels must agree. Got ", datasets, " and ", dataset_labels)
    sys.exit(1)

# Parse the orbits and their labels. Split by comma.
orbits = opt.orbit_id.split(',')
orbit_labels = opt.orbit_label.split(',')
if len(orbit_labels) == 0 or (len(orbit_labels) == 1 and orbit_labels[0] == ""):
    orbit_labels = orbits[:]
if len(orbit_labels) != len(orbits):
    print("Number of orbits and orbit_labels must agree. Got ", orbits, " and ", orbit_labels)
    sys.exit(1)

# Read the figure dimensions
figure_size = opt.figure_size.split(',')
if len(figure_size) != 2:
    print("The --figure-size option must have two values. Got: ", figure_size)
    sys.exit(1)
figure_size = [float(x) for x in figure_size]

# We assume we have one or two datasets that we want to plot on top of each other.
numSets = len(datasets)
if numSets < 1:
    print("Must specify at least one dataset.")
    sys.exit(1)

f, ax = plt.subplots(len(orbits), 3, sharex=True, sharey = False, 
                     figsize = (figure_size[0], figure_size[1]))

# Set up the legend in the upper right corner. We will have text in upper-left
plt.rcParams["legend.loc"] = 'upper right' 

# Set up the font for all elements
fs = opt.font_size
plt.rcParams.update({'font.size': fs})
plt.rc('axes', titlesize = fs)   # fontsize of the axes title
plt.rc('axes', labelsize = fs)   # fontsize of the x and y labels
plt.rc('xtick', labelsize = fs)  # fontsize of the tick labels
plt.rc('ytick', labelsize = fs)  # fontsize of the tick labels
plt.rc('legend', fontsize = fs)  # legend fontsize
plt.rc('figure', titlesize = fs) # fontsize of the figure title

# Plot each row in the figure
for row in range(len(orbits)):
  plot_row(ax, row, orbits, hasList, datasets, orbit_labels, dataset_labels, 
           opt.ref_list, opt)

# Show a title if set
if opt.title != "":
    f.suptitle(opt.title, fontsize=fs)

plt.tight_layout()
plt.show()
