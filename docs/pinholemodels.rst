Frame camera models
===================

Ames Stereo Pipeline supports a generic Pinhole camera model with
several lens distortion models which cover common calibration methods,
and also the somewhat more complicated panoramic (*optical bar*) camera
model.

.. _pinholemodels:

Pinhole models
--------------

Overview
--------

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
   millimeters.

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
must be equal to the size of each pixel in millimeters. 

The following lens distortion models are currently supported. (The
formulas below may miss some small details; the implementation in
``LensDistortion.cc`` in VisionWorkbench should be the final
reference.)

* **Null** = A placeholder model that applies no distortion.

* **Tsai** = A common distortion model :cite:`tsai1987` similar to
  the one used by OpenCV and THEIA. This model uses the following
  parameters:
  
  *K1, K2* = Radial distortion parameters.
  
  *P1, P2* = Tangential distortion parameters.
  
  The following equations describe the distortion, starting with the
  undistorted pixel :math:`(Px, Py)`.

  .. math::

    (x, y) = \left(\frac{Px - cu}{fu}, \frac{Py-cv}{fv}\right)

    r^{2} = x^{2} + y^{2}

    x_{dist} = Px + x\left(K_{1}r^{2} + K_{2}r^{4} + 2P_{1}y + P_{2}\left(\frac{r^{2}}{x} + 2x\right)\right)

    y_{dist} = Py + y\left(K_{1}r^{2} + K_{2}r^{4} + 2P_{2}x + P_{1}\left(\frac{r^{2}}{y} + 2y\right)\right)


* **Adjustable Tsai** = A variant of the Tsai model where any number of
  *K* terms and a skew term (alpha) can be used. Can apply the AgiSoft
  Lens calibration parameters.

* **Brown-Conrady** = An older model based on a centering angle
  :cite:`brown1966,brown1971`.

  This model uses the following parameters:
  
  *K1, K2, K3* = Radial distortion parameters.
  
  *P1, P2* = Tangential distortion parameters.
  
  *xp, yp* = Principal point offset.
  
  *phi* = Tangential distortion angle in radians.

  The following equations describe the distortion, note that this
  model uses non-normalized pixel units, so they are in mm:

  .. math::
    x = x_{dist} - xp

    y = y_{dist} - yp

    r^{2} = x^{2} + y^{2}

    dr = K_{1}r^{3} + K_{2}r^{5} + K_{3}r^{7}

    x_{undist} = x + x\frac{dr}{r} - (P_{1}r^{2} +P_{2}r^{4})\sin(phi)

    y_{undist} = y + y\frac{dr}{r} + (P_{1}r^{2} +P_{2}r^{4})\cos(phi)


* **Photometrix** = A model matching the conventions used by the Australis
  software from Photometrix.
  
  *K1, K2, K3* = Radial distortion parameters.
  
  *P1, P2* = Tangential distortion parameters.
  
  *xp, yp* = Principal point offset.
  
  *B1, B2* = Unused parameters.
  
  The following equations describe the distortion, note that this
  model uses non-normalized pixel units, so they are in mm.

  .. math::

    x = x_{dist} - xp

    y = y_{dist} - yp

    r^{2} = x^{2} + y^{2}

    dr = K_{1}r^{3} + K_{2}r^{5} + K_{3}r^{7}

    x_{undist} = x + x\frac{dr}{r} + P_{1}(r^{2} +2x^{2}) + 2P_{2}xy

    y_{undist} = y + y\frac{dr}{r} + P_{2}(r^{2} +2y^{2}) + 2P_{1}xy


* **RPC** = A rational polynomial coefficient model.

In this model, one goes from distorted coordinates :math:`(x, y)` to
undistorted coordinates via the formula

.. math::

    x_{undist} = \frac{P_1(x, y)}{Q_1(x, y)}

    y_{undist} = \frac{P_2(x, y)}{Q_2(x, y)}

The functions in the numerator and denominator are polynomials in
:math:`x` and :math:`y` with certain coefficients. The degree of
polynomials can be any positive integer.

RPC distortion models can be generated as approximations to other
pre-existing models with the tool ``convert_pinhole_model``
(:numref:`convert_pinhole_model`).

This tool also creates RPC to speed up the reverse operation, of going
from undistorted to distorted pixels, and those polynomial coefficients
are also saved as part of the model.

--------------

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

The first half of the file is the same for all Pinhole models:

* ``VERSION_X`` = A header line used to track the format of the file.

* ``PINHOLE`` = The type of camera model, so that other types can be
  stored with the .tsai extension.

* ``fu, fv, cu, cv`` = The first four intrinsic parameters described in
  the previous section.

* ``u, v, and w_direction`` = These lines allow an additional
  permutation of the axes of the camera coordinates. By default, the
  positive column direction aligns with x, the positive row direction
  aligns with y, and downward into the image aligns with z.

* ``C`` = The location of the camera center, usually in the geocentric
  coordinate system (GCC/ECEF).

* ``R`` = The rotation matrix describing the camera’s absolute pose in
  the coordinate system (:numref:`pinholemath`).

* ``pitch`` = The pitch intrinsic parameter described in the previous
  section.

The second half of the file describes the lens distortion model
being used. The name of the distortion model appears first, followed
by a list of the parameters for that model. The number of parameters
may be different for each distortion type. Samples of each format
are shown below:

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

* **RPC**
  ::

      RPC
      rpc_degree = 1
      image_size = 5760 3840
      distortion_num_x   = 0 1 0
      distortion_den_x   = 1 0 0
      distortion_num_y   = 0 0 1
      distortion_den_y   = 1 0 0
      undistortion_num_x = 0 1 0
      undistortion_den_x = 1 0 0
      undistortion_num_y = 0 0 1
      undistortion_den_y = 1 0 0

  This sample RPC lens distortion model represents the case of no
  distortion, when the degree of the polynomials is 1, and both the
  distortion and undistortion formula leave the pixels unchanged, that
  is, the distortion transform is

  .. math:: (x, y) \to (x, y) = \left(\frac{ 0 + 1\cdot x + 0\cdot y}{1 + 0\cdot x + 0\cdot y}, \frac{0 + 0\cdot x + 1\cdot y)}{1 + 0\cdot x + 0\cdot y}\right).

  In general, if the degree of the polynomials is :math:`n`, there are
  :math:`2(n+1)(n+2)` coefficients. The zero-th degree coefficients in
  the denominator are always set to 1.

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

From here the pixel location is computed as:

.. math:: \frac{1}{p} \left(fu \frac{Q_1}{Q_3} + cu, fv \frac{Q_2}{Q_3} + cv\right)

where :math:`p` is the pixel pitch.

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
must be set to ’left’ or ’right’. ``scan_dir`` and
``use_motion_compensation`` control how the sensor model accounts
accounts for the motion of the satellite during the image scan. Without
the benefit of detailed historical documents it may require
experimentation to find the good initial values for these cameras. When
using ``bundle_adjust``, the intrinsic parameters that are solved for
are ``speed``, ``motion_compensation_factor``, and ``scan_time``.
