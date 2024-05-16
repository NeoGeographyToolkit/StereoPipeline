Frame camera models
===================

Ames Stereo Pipeline supports a generic Pinhole camera model with several lens
distortion models which cover common calibration methods
(:numref:`pinholemodels`), the somewhat more complicated panoramic (*optical
bar*) camera model (:numref:`panoramic`), and the CSM Frame model, that has
several lens distortion implementations (:numref:`csm_frame_def`).

Bundle adjustment can refine the intrinsic and extrinsic camera parameters,
including the lens distortion model (:numref:`floatingintrinsics`).

.. _pinholemodels:

Pinhole models
--------------

Overview
~~~~~~~~

The generic Pinhole model uses the following parameters:

-  *fu* = The focal length in horizontal pixel units.

-  *fv* = The focal length in vertical pixel units.

-  *cu* = The horizontal offset of the principal point of the camera in
   the image plane in pixel units, from 0,0.

-  *cv* = The vertical offset of the principal point of the camera in
   the image plane in pixel units, from 0,0.

-  *pitch* = The size of each pixel in the units used to specify the
   four parameters listed above. This will usually either be 1.0 if they
   are specified in pixel units or alternately the size of a pixel in
   millimeters or meters.

The focal length is sometimes known as the *principal distance*. The
value :math:`cu` is usually approximately half the image width in pixels
times the pitch, while :math:`cv` is often the image height in pixels
times the pitch, though there are situations when these can be quite
different.

A few sample Pinhole models are shown later in the text. The underlying
mathematical model is described in :numref:`pinholemath`.

Along with the basic Pinhole camera parameters, a lens distortion model
can be added. Note that the units used in the distortion model must
match the units used for the parameters listed above. For example, if
the camera calibration was performed using units of millimeters the
focal lengths etc. must be given in units of millimeters and the pitch
must be equal to the size of each pixel in millimeters. Alternatively,
units of meter can be used, and the choice of unit must be documented
by the creators of the models.

The following lens distortion models are currently supported. (The
formulas below may miss some small details; the implementation in
``LensDistortion.cc`` in VisionWorkbench should be the final
reference.)

Note that the values below change drastically depending on whether the
model creator chooses pixel units, or if measuring in millimeters or
meters. In either case, all lengths must be consistent and the units
documented by the model creator.

Lens distortion models
~~~~~~~~~~~~~~~~~~~~~~

Here are the lens distortion models supported by ASP. Samples for each
model are shown in :numref:`file_format`.

Null
^^^^

A placeholder model that applies no distortion.

Tsai
^^^^

A common distortion model :cite:`tsai1987`. In the most recent builds (after ASP
3.3.0) this was made to agree precisely with the `OpenCV radial-tangential lens
distortion model <https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html>`_.
This model uses the following parameters:
  
*K1, K2, K3* = Radial distortion parameters. The last one is optional.
  
*P1, P2* = Tangential distortion parameters.

The lens distortion operation is computed via an explicit formula, and for undistortion
a nonlinear solver is used based on Newton's method.

This is the preferred model, unless the lens has a wide field of view, when
the Fisheye model should be used (described further below).

Adjustable Tsai
^^^^^^^^^^^^^^^^
  
A variant of the Tsai model where any number of *K* terms and a skew term
(alpha) can be used. Can apply the AgiSoft Lens calibration parameters.

.. _brown_conrady:

Brown-Conrady
^^^^^^^^^^^^^

This is an older model based on a centering angle :cite:`brown1966,brown1971`.
Example usage is in :numref:`sfmgeneric`.

This model uses the following parameters:
  
*K1, K2, K3* = Radial distortion parameters.
  
*P1, P2* = Tangential distortion parameters.
  
*xp, yp* = Principal point offset.
  
*phi* = Tangential distortion angle in radians.

The following equations describe the distortion. Note that this model uses
*non-normalized* pixel units, so they can be in millimeters or meters:

.. math::

    x = x_{dist} - xp

    y = y_{dist} - yp

    r^{2} = x^{2} + y^{2}

    dr = K_{1}r^{3} + K_{2}r^{5} + K_{3}r^{7}

    x_{undist} = x + x\frac{dr}{r} - (P_{1}r^{2} +P_{2}r^{4})\sin(phi)

    y_{undist} = y + y\frac{dr}{r} + (P_{1}r^{2} +P_{2}r^{4})\cos(phi)

The formulas start with distorted pixels that are then undistorted. This is not
preferable with ASP, as then the distortion operation requires a solver, which
makes bundle adjustment and mapprojection very slow. Use instead the Tsai model. 

A Brown-Conrady model can be converted to a Tsai model with
``convert_pinhole_model`` (:numref:`convert_pinhole_model`). The produced model
can be refined with bundle adjustment (:numref:`floatingintrinsics`), if having
several images and many interest point matches.

Photometrix
^^^^^^^^^^^

A model matching the conventions used by the Australis
software from Photometrix.
  
*K1, K2, K3* = Radial distortion parameters.
  
*P1, P2* = Tangential distortion parameters.
  
*xp, yp* = Principal point offset.
  
*B1, B2* = Unused parameters.

The following equations describe the undistortion. Note that this
model uses non-normalized pixel units, so they are in mm.

.. math::

    x = x_{dist} - xp

    y = y_{dist} - yp

    r^{2} = x^{2} + y^{2}

    dr = K_{1}r^{3} + K_{2}r^{5} + K_{3}r^{7}

    x_{undist} = x + x\frac{dr}{r} + P_{1}(r^{2} +2x^{2}) + 2P_{2}xy

    y_{undist} = y + y\frac{dr}{r} + P_{2}(r^{2} +2y^{2}) + 2P_{1}xy

These formulas also start with distorted pixels and undistort them, just as
the Brown-Conrady model. This is not preferred. Use instead the Tsai model.

Fisheye
^^^^^^^

A four-parameter model for wide field-of-view lenses, with the `same
implementation as OpenCV
<https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html>`_ and
``rig_calibrator`` (:numref:`rig_calibrator`).
  
The parameters are named ``k1, k2, k3, k4``. 
  
To apply the lens distortion with this model, the undistorted pixels are first
shifted relative to the optical center, divided by the focal length, producing
pixel (*x, y*), and then the following equations are applied:
  
  .. math::
  
    r = \sqrt{x^2 + y^2}
    
    \theta = \arctan(r)
    
    \theta_d = \theta (1 + k_1 \theta^2 + k_2 \theta^4 + k_3 \theta^6 + k_4 \theta^8)
    
    s = \frac{\theta_d}{r}
    
    x_{dist} = s \cdot x
    
    y_{dist} = s \cdot y
  
These values are then multiplied by the focal length, and the optical center is
added back in.

The undistortion operation goes in the opposite direction. It requires inverting
a nonlinear function, which is done with Newton's method.

Care is needed around the origin to avoid division of small numbers.

FOV
^^^

A field-of-view model with a single parameter, for wide-angle lenses.

This is in agreement with ``rig_calibrator`` (:numref:`rig_calibrator`).

The implementation is as follows. Let ``k1`` by the distortion parameter. Given
an undistorted pixel, shift it relative to the optical center, divide by the
focal length, producing pixel (*x, y*). Then, the following equations are
applied:

  .. math::
  
    p_1 = 1 / k_1
    
    p_2 = 2 \tan(k_1 / 2)

    r_u = \sqrt{x^2 + y^2}
    
    r_d = p_1 \arctan(r_u p_2)
    
    s = r_d / r_u

    x_{dist} = s \cdot x
    
    y_{dist} = s \cdot y

These values are then multiplied by the focal length, and the optical center is
added back in.

The undistortion operation goes in the opposite direction, and an explicit formula
exists for that. 

Care is needed around the origin to avoid division of small numbers.

.. _rpc_distortion:

RPC
^^^
    
A rational polynomial coefficient (RPC) model is employed for distortion. The
degree can be arbitrary. This is different than going from ground to image
coordinates via RPC (:numref:`rpc`).

In this model, the transform from undistorted *normalized* pixels :math:`(x, y)`
to distorted normalized pixels is via the formulas

.. math::

    x_{dist} = \frac{P_1(x, y)}{Q_1(x, y)}

    y_{dist} = \frac{P_2(x, y)}{Q_2(x, y)}

The functions in the numerator and denominator are polynomials in
:math:`x` and :math:`y` with certain coefficients. The degree of
polynomials can be any positive integer. A degree of 3 or 4 is usually 
more than sufficient.

The inputs and output pixels are normalized, that is, shifted relative to the
optical center, and (in the most latest builds) are also divided by the focal
length. Such normalizations are applied before distortion / undistortion
operations, and then undone after them. This is consistent with the
radial-tangential and fisheye models.

RPC distortion models can be generated as approximations to other
pre-existing models with the tool ``convert_pinhole_model``
(:numref:`convert_pinhole_model`).

In the latest builds, the RPC undistortion is computed via a solver based on 
Newton's method, as for the fisheye lens distortion model.

An illustration for how to use the RPC lens distortion is in
:numref:`ba_rpc_distortion`.

.. _file_format:

File formats
~~~~~~~~~~~~

ASP Pinhole model files are written in an easy to work with plain text
format using the extension ``.tsai``. A sample file is shown below.

::

   VERSION_4
   PINHOLE
   fu = 28.429
   fv = 28.429
   cu = 17.9712
   cv = 11.9808
   u_direction = 1  0  0
   v_direction = 0  1  0
   w_direction = 0  0  1
   C = 266.943 -105.583 -2.14189
   R = 0.0825447 0.996303 -0.0238243 -0.996008 0.0832884 0.0321213 0.0339869 0.0210777 0.9992
   pitch = 0.0064
   TSAI
   k1 = -0.094196634563
   k2 = 0.115036424262
   k3 = -0.032238313341
   p1 = -0.000256622541
   p2 = -0.000353613460

The first half of the file is the same for all Pinhole models:

* ``VERSION_X`` A header line used to track the format of the file.

* ``PINHOLE`` The type of camera model, so that other types can be
  stored with the .tsai extension.

* ``fu, fv, cu, cv`` The first four intrinsic parameters described in
  the previous section.

* ``u, v, and w_direction`` These lines allow an additional
  permutation of the axes of the camera coordinates. By default, the
  positive column direction aligns with x, the positive row direction
  aligns with y, and downward into the image aligns with z.
  
  It is strongly suggested to not modify these directions, and they 
  will be removed from the file in future versions of ASP. The camera
  rotation alone (below) should be sufficient.

* ``C`` The location of the camera center, usually in the geocentric
  coordinate system (GCC/ECEF).

* ``R`` The rotation matrix describing the camera's absolute pose in the world
  coordinate system (camera-to-world rotation, :numref:`pinholemath`).

* ``pitch`` The pitch intrinsic parameter described in the previous
  section.

The second half of the file describes the lens distortion model
being used. The name of the distortion model appears first, followed
by a list of the parameters for that model. The number of parameters
may be different for each distortion type. 

Partial samples of each format are shown below. *The part up to and including
the line having the pitch is the same for all models and not shown in the examples.*

* **Null**
  ::

      NULL

* **Tsai**
  ::

      TSAI
      k1 = 1.31024e-04
      k2 = -2.05354e-07
      p1 = 0.5
      p2 = 0.4
      k3 = 1e-3

The ``k3`` parameter is optional in the Tsai model. It is stored last,
as done in OpenCV.

* **Adjustable Tsai**
  ::

      AdjustableTSAI
      Radial Coeff: Vector3(1.31024e-04, 1.31024e-07, 1.31024e-08)
      Tangential Coeff: Vector2(-2.05354e-07, 1.05354e-07)
      Alpha: 0.4

* **Brown-Conrady**
  ::

      BrownConrady
      xp = 0.5
      yp = 0.4
      k1 = 1.31024e-04
      k2 = -2.05354e-07
      k3 = 1.31024e-08
      p1 = 0.5
      p2 = 0.4
      phi = 0.001

* **Photometrix**
  ::

      Photometrix
      xp = 0.004
      yp = -0.191
      k1 = 1.31024e-04
      k2 = -2.05354e-07
      k3 = -5.28558e-011
      p1 = 7.2359e-006
      p2 = 2.2656e-006
      b1 = 0.0
      b2 = 0.0

* **Fisheye**
  ::

      FISHEYE
      k1 = -0.036031089735101024
      k2 = 0.038013929764216248
      k3 = -0.058893197165394658
      k4 = 0.02915171342570104
      
* **RPC**
  ::

      RPC
      rpc_degree = 1
      distortion_num_x = 0 1 0
      distortion_den_x = 1 0 0
      distortion_num_y = 0 0 1
      distortion_den_y = 1 0 0

This sample RPC lens distortion model represents the case of no distortion, when
the degree of the polynomials is 1, and both the distortion and undistortion
formula leave the pixels unchanged, that is, the distortion transform is

  .. math:: (x, y) \to (x, y) = \left(\frac{ 0 + 1\cdot x + 0\cdot y}{1 + 0\cdot x + 0\cdot y}, \frac{0 + 0\cdot x + 1\cdot y}{1 + 0\cdot x + 0\cdot y}\right).

In general, if the degree of the polynomials is :math:`n`, there are
:math:`2(n+1)(n+2)` coefficients. The zero-th degree coefficients in
the denominator are always set to 1.

Notes
~~~~~

For several years Ames Stereo Pipeline generated Pinhole files in the
binary ``.pinhole`` format. That format is no longer supported.

Also in the past Ames Stereo Pipeline has generated a shorter version of
the current file format, also with the extension ``.tsai``, which only
supported the TSAI lens distortion model. Existing files in that format
can still be used by ASP.

Note that the ``orbitviz`` tool can be useful for checking the
formatting of ``.tsai`` files you create and to estimate the position
and orientation. To inspect the orientation use the optional ``.dae``
model file input option and observe the rotation of the 3D model.

.. _pinholemath:

How the pinhole model is applied
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As mentioned in :numref:`file_format`, the ASP Pinhole models
store the focal length as :math:`fu` and :math:`fv`, the optical center
:math:`(cu, cv)` (which is the pixel location at which the ray coming
from the center of the camera is perpendicular to the image plane, in
units of the pixel pitch), the vector :math:`C` which is the camera
center in world coordinates system, and the matrix :math:`R` that is the
transform from camera to world coordinates.

To go in more detail, a point :math:`Q` in the camera coordinate system
gets transformed to a point :math:`P` in the world coordinate system
via:

.. math:: P = RQ + C

Hence, to go from world to camera coordinates one does:

.. math:: Q = R^{-1}  P - R^{-1}  C

From here the ``undistorted`` pixel location is computed as:

.. math:: \frac{1}{p} \left(fu \frac{Q_1}{Q_3} + cu, fv \frac{Q_2}{Q_3} + cv\right)

where :math:`p` is the pixel pitch. Next, a distortion model may be
applied, as discussed earlier.

.. _panoramic:

Panoramic Camera Model
----------------------

ASP also supports a simple panoramic/optical bar camera model for use
with images such as the declassified Corona KH4 and Keyhole KH9 images.
It implements the model from :cite:`schenk2003rigorous` with
the motion compensation from :cite:`sohn2004mathematical`.

Such a model looks as follows:

::

   VERSION_4
   OPTICAL_BAR
   image_size = 110507 7904
   image_center = 55253.5 3952
   pitch = 7.0e-06
   f = 0.61000001430511475
   scan_time = 0.5
   forward_tilt = -0.261799
   iC = -1047140.9611702315 5508464.4323527571 3340425.4078937685
   iR = -0.96635634448923746 -0.16918164442572045 0.1937343197650008 -0.23427205529446918 0.26804084264169648 -0.93448954557235941 0.10616976770014927 -0.94843643849513648 -0.29865750042675621
   speed = 7700
   mean_earth_radius = 6371000
   mean_surface_elevation = 4000
   motion_compensation_factor = 1.0
   scan_dir = left

Here, the image size and center are given in pixels, with the width
followed by the height. The pixel pitch and focal length ``f`` are in
meters. The scan time is seconds, the forward tilt is in radians, the
speed is in meters per second, and the Earth radius and mean surface
elevation are in meters. The initial camera center ``iC`` is in meters,
and the rotation matrix ``iR`` stores the absolute pose. ``scan_dir``
must be set to ``left`` or ``right``. The values ``scan_dir`` and
``use_motion_compensation`` control how the sensor model accounts
accounts for the motion of the satellite during the image scan. Without
the benefit of detailed historical documents it may require
experimentation to find the good initial values for these cameras. When
using ``bundle_adjust``, the intrinsic parameters that are solved for
are ``speed``, ``motion_compensation_factor``, and ``scan_time``.

.. _csm_frame_def:

CSM frame camera
----------------

ASP supports the CSM (:numref:`csm`) frame camera model. This is analogous to the 
ASP Pinhole model (:numref:`pinholemodels`). 

The CSM frame camera model has its own collection of 
lens distortion models. Those include the `OpenCV radial-tangential distortion model
<https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html>`_ (it has 5
parameters, in the order k1, k2, p1, p2, k3), *transverse* distortion, which is
a pair of full polynomials of degree 3 in both x and y (20 coefficients), and
various other `specialized models
<https://github.com/DOI-USGS/usgscsm/blob/main/include/usgscsm/Distortion.h>`_.

In latest ASP builds, the Pinhole model with no distortion or with radial-tangential
(tsai) distortion operates as the CSM frame camera model with the same distortion
model and values, which can be verified with ``cam_test`` (:numref:`cam_test`).

In the CSM .json model state files (:numref:`csm_state`), the transverse and
radial-tangential distortion models have ``m_distortionType`` set to the values
of 1 and 7, respectively, with ``m_opticalDistCoeffs`` having the distortion
parameters.

ASP's ``cam_gen`` program (:numref:`cam_gen_frame`) can find the best-fit CSM
frame camera model with the OpenCV radial-tangential distortion and with the
transverse (3rd degree polynomial) distortion model. Then bundle adjustment can
be employed to refine the intrinsic and extrinsic camera parameters
(:numref:`ba_frame_linescan`).

The ``sat_sim`` program (:numref:`sat_sim`) can create CSM frame cameras
simulating a satellite in orbit.
