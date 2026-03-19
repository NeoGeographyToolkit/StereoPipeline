// __BEGIN_LICENSE__
//  Copyright (c) 2006-2026, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#include <asp/SfmView/SceneManager.h>
#include <asp/SfmView/SceneRenderer.h>

#include <QOpenGLShaderProgram>

#include <iostream>
#include <limits>

// GLSL shaders. Colors are baked into mesh vertices: green for the ground grid,
// white for camera frusta, yellow for view direction, RGB for camera coordinate
// axes.

static char const* const WIREFRAME_VERT =
    "#version 330 core\n"
    "in vec4 pos;\n"
    "in vec4 color;\n"
    "out vec4 ocolor;\n"
    "uniform mat4 viewmat;\n"
    "uniform mat4 projmat;\n"
    "void main(void) {\n"
    "    ocolor = color;\n"
    "    gl_Position = projmat * (viewmat * pos);\n"
    "}\n";

static char const* const WIREFRAME_FRAG =
    "#version 330 core\n"
    "in vec4 ocolor;\n"
    "layout(location=0) out vec4 frag_color;\n"
    "void main(void) {\n"
    "    gl_FragDepth = gl_FragCoord.z;\n"
    "    frag_color = ocolor;\n"
    "}\n";

void SceneRenderer::load_shaders(void) {
  if (!this->wireframe_shader) {
    this->wireframe_shader = new QOpenGLShaderProgram();
    this->wireframe_shader->addShaderFromSourceCode(
      QOpenGLShader::Vertex, WIREFRAME_VERT);
    this->wireframe_shader->addShaderFromSourceCode(
      QOpenGLShader::Fragment, WIREFRAME_FRAG);
    this->wireframe_shader->link();
  }
}

void SceneRenderer::send_uniform(sfm::SfmCamera const& cam) {
  this->wireframe_shader->bind();
  // transpose=true: our matrices are row-major, GL expects column-major
  GLint loc_view = this->wireframe_shader->uniformLocation("viewmat");
  if (loc_view >= 0)
    glFunctions()->glUniformMatrix4fv(loc_view, 1, GL_TRUE, *cam.view);
  GLint loc_proj = this->wireframe_shader->uniformLocation("projmat");
  if (loc_proj >= 0)
    glFunctions()->glUniformMatrix4fv(loc_proj, 1, GL_TRUE, *cam.proj);
}

SceneRenderer::SceneRenderer(GlWidget* gl_widget) {
  this->gl_widget = gl_widget;

  this->action_frusta = new QAction("Draw camera frusta");
  this->action_frusta->setCheckable(true);
  this->action_frusta->setChecked(true);

  this->action_viewdir = new QAction("Draw viewing direction");
  this->action_viewdir->setCheckable(true);
  this->action_viewdir->setChecked(true);

  this->action_ground = new QAction("Draw ground plane");
  this->action_ground->setCheckable(true);
  this->action_ground->setChecked(true);

  this->frusta_size_slider = new QSlider();
  this->frusta_size_slider->setMinimum(1);
  this->frusta_size_slider->setMaximum(100);
  this->frusta_size_slider->setValue(10);
  this->frusta_size_slider->setOrientation(Qt::Horizontal);

  this->connect(&SceneManager::get(), SIGNAL(scene_selected(sfm::Scene::Ptr)),
    this, SLOT(on_scene_changed()));
  this->connect(&SceneManager::get(), SIGNAL(view_selected(sfm::View::Ptr)),
    this, SLOT(reset_viewdir_renderer()));
  this->connect(this->frusta_size_slider, SIGNAL(valueChanged(int)),
    this, SLOT(reset_frusta_renderer()));
  this->connect(this->frusta_size_slider, SIGNAL(valueChanged(int)),
    this->gl_widget, SLOT(repaint()));
  this->connect(this->action_frusta, SIGNAL(toggled(bool)),
    this->gl_widget, SLOT(repaint()));
  this->connect(this->action_viewdir, SIGNAL(toggled(bool)),
    this->gl_widget, SLOT(repaint()));
  this->connect(this->action_ground, SIGNAL(toggled(bool)),
    this->gl_widget, SLOT(repaint()));
}

QAction* SceneRenderer::get_action_frusta(void) {
  return this->action_frusta;
}

QAction* SceneRenderer::get_action_viewdir(void) {
  return this->action_viewdir;
}

QAction* SceneRenderer::get_action_ground(void) {
  return this->action_ground;
}

QSlider* SceneRenderer::get_frusta_size_slider(void) {
  return this->frusta_size_slider;
}

void SceneRenderer::set_scene(sfm::Scene::Ptr scene) {
  this->scene = scene;
  this->gl_widget->repaint();
}

void SceneRenderer::set_view(sfm::View::Ptr view) {
  this->view = view;
  this->gl_widget->repaint();
}

void SceneRenderer::reset_scene(void) {
  this->scene = nullptr;
  this->view = nullptr;
  this->gl_widget->repaint();
}

void SceneRenderer::on_scene_changed(void) {
  this->orig_cam_centers.clear();
  this->orig_cam2world_vec.clear();
  this->frusta_renderer.reset();
  this->ground_renderer.reset();
}

void SceneRenderer::reset_frusta_renderer(void) {
  this->frusta_renderer.reset();
  this->ground_renderer.reset();
}

void SceneRenderer::reset_ground_renderer(void) {
  this->ground_renderer.reset();
}

void SceneRenderer::reset_viewdir_renderer(void) {
  this->viewdir_renderer.reset();
}

void SceneRenderer::init_impl(void) {
  this->load_shaders();
}

void SceneRenderer::resize_impl(int old_width, int old_height) {
  this->sfm::GlContext::resize_impl(old_width, old_height);
}

void SceneRenderer::paint_impl(void) {
  this->update_camera();
  glFunctions()->glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

  glFunctions()->glDepthFunc(GL_LESS);
  glFunctions()->glEnable(GL_DEPTH_TEST);
  glFunctions()->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glFunctions()->glClearDepth(1.0f);
  glFunctions()->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  this->send_uniform(this->camera);

  if (this->action_frusta->isChecked()) {
    if (this->frusta_renderer == nullptr)
      this->create_frusta_renderer();
    if (this->frusta_renderer != nullptr)
      this->frusta_renderer->draw();
  }

  if (this->action_ground->isChecked()) {
    if (this->ground_renderer == nullptr)
      this->create_ground_renderer();
    if (this->ground_renderer != nullptr)
      this->ground_renderer->draw();
  }

  if (this->action_viewdir->isChecked()) {
    if (this->viewdir_renderer == nullptr)
      this->create_viewdir_renderer();
    if (this->viewdir_renderer != nullptr)
      this->viewdir_renderer->draw();
  }
}

// Add a camera frustum wireframe to a mesh.
void add_camera_to_mesh(sfm::SfmCameraInfo const& camera,
  float size, sfm::TriangleMesh::Ptr mesh) {
  sfm::Vec4f const frustum_start_color(1.0f, 1.0f, 1.0f, 1.0f);
  sfm::Vec4f const frustum_end_color(1.0f, 1.0f, 1.0f, 1.0f);

  sfm::TriangleMesh::VertexList& verts = mesh->get_vertices();
  sfm::TriangleMesh::ColorList& colors = mesh->get_vertex_colors();
  sfm::TriangleMesh::FaceList& faces = mesh->get_faces();

  sfm::Matrix4f ctw;
  camera.fill_cam_to_world(*ctw);
  sfm::Vec3f cam_x = ctw.mult(sfm::Vec3f(1.0f, 0.0f, 0.0f), 0.0f);
  sfm::Vec3f cam_y = ctw.mult(sfm::Vec3f(0.0f, 1.0f, 0.0f), 0.0f);
  sfm::Vec3f cam_z = ctw.mult(sfm::Vec3f(0.0f, 0.0f, 1.0f), 0.0f);
  sfm::Vec3f campos = ctw.mult(sfm::Vec3f(0.0f), 1.0f);

  std::size_t idx = verts.size();
  verts.push_back(campos);
  colors.push_back(frustum_start_color);
  for (int j = 0; j < 4; ++j) {
    sfm::Vec3f corner = campos + size * cam_z
      + cam_x * size / (2.0f * camera.flen) * (j & 1 ? -1.0f : 1.0f)
      + cam_y * size / (2.0f * camera.flen) * (j & 2 ? -1.0f : 1.0f);
    verts.push_back(corner);
    colors.push_back(frustum_end_color);
    faces.push_back(idx + 0); faces.push_back(idx + 1 + j);
  }
  faces.push_back(idx + 1); faces.push_back(idx + 2);
  faces.push_back(idx + 2); faces.push_back(idx + 4);
  faces.push_back(idx + 4); faces.push_back(idx + 3);
  faces.push_back(idx + 3); faces.push_back(idx + 1);

  verts.push_back(campos);
  verts.push_back(campos + (size * 0.5f) * cam_x);
  verts.push_back(campos);
  verts.push_back(campos + (size * 0.5f) * cam_y);
  verts.push_back(campos);
  verts.push_back(campos + (size * 0.5f) * cam_z);
  colors.push_back(sfm::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
  colors.push_back(sfm::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
  colors.push_back(sfm::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
  colors.push_back(sfm::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
  colors.push_back(sfm::Vec4f(0.0f, 0.0f, 1.0f, 1.0f));
  colors.push_back(sfm::Vec4f(0.0f, 0.0f, 1.0f, 1.0f));
  faces.push_back(idx + 5); faces.push_back(idx + 6);
  faces.push_back(idx + 7); faces.push_back(idx + 8);
  faces.push_back(idx + 9); faces.push_back(idx + 10);
}

// Find shortest distance from camera center to the origin
double find_shortest_distance(std::vector<sfm::Vec3d> centers) {
  if (centers.size() == 0)
    return 1.0;

  double shortest = centers[0].norm();
  for (std::size_t i = 1; i < centers.size(); i++) {
    double dist = centers[i].norm();
    if (dist < shortest)
      shortest = dist;
  }
  return shortest;
}

// Function to find the mean camera position
sfm::Vec3d find_mean_camera_pos(std::vector<sfm::Vec3d> const& centers) {
  if (centers.size() == 0)
    return sfm::Vec3d(0,0,0);

  sfm::Vec3d mean(0,0,0);
  for (std::size_t i = 0; i < centers.size(); i++)
    mean += centers[i];
  mean /= centers.size();
  return mean;
}

// Collect the cam2world matrices and camera centers in vectors.
void extractSfmCameraPoses(sfm::Scene::ViewList const& views,
  // Outputs
  std::vector<sfm::Vec3d>& cam_centers,
  std::vector<sfm::Matrix3d>& cam2world_vec) {

  cam_centers.clear();
  cam2world_vec.clear();

  for (std::size_t i = 0; i < views.size(); i++) {

    if (views[i].get() == nullptr) {
      std::cerr << "Error: Empty camera.\n";
      continue;
    }

    sfm::SfmCameraInfo const& cam = views[i]->get_camera(); // alias

    // The cameras store trans = -inverse(camera2world) * camera_center
    // and rot = inverse(camera2world).
    // We need to invert the camera2world matrix to get the camera center.
    // So, need to find:
    // and camera2world = transpose(rot)
    // camera_center = -camera2world * trans

    sfm::Matrix3d world2cam(cam.rot);
    sfm::Vec3d t(cam.trans);
    sfm::Matrix3d cam2world = world2cam.transposed();
    sfm::Vec3d ctr = -cam2world.mult(t);

    cam_centers.push_back(ctr);
    cam2world_vec.push_back(cam2world);
  }

  return;
}

// Apply the camera2world transforms and camera centers to the cameras
void applySfmCameraPoses(std::vector<sfm::Vec3d> const& cam_centers,
                      std::vector<sfm::Matrix3d> const& cam2world_vec,
                      sfm::Scene::ViewList& views) {

  for (std::size_t i = 0; i < views.size(); i++) {
    if (views[i].get() == nullptr) {
      std::cerr << "Error: Empty camera.\n";
      continue;
    }

    sfm::SfmCameraInfo cam = views[i]->get_camera();

    // Compute T = -cam2world^T * C and copy to the camera
    sfm::Vec3d t = -cam2world_vec[i].transposed().mult(cam_centers[i]);
    for (int coord = 0; coord < 3; coord++)
      cam.trans[coord] = t[coord];

    // Compute R = cam2world^T and copy it to the camera
    sfm::Matrix3d R = cam2world_vec[i].transposed();
    for (int row = 0; row < 3; row++)
      for (int col = 0; col < 3; col++)
        cam.rot[row * 3 + col] = R(row, col);

    views[i]->set_camera(cam);
    views[i]->set_dirty(false);
  }
  return;
}

// Given a vector, find a rotation matrix that rotates the vector to the y-axis.
void completeVectorToRotation(sfm::Vec3d& y, sfm::Matrix3d& R) {

  int largest_comp = 0;
  for (int i = 1; i < 3; i++) {
    if (std::abs(y[i]) > std::abs(y[largest_comp]))
      largest_comp = i;
  }

  int j = largest_comp + 1;
  if (j == 3)
    j = 0;

  sfm::Vec3d x(0.0, 0.0, 0.0);
  x[j] = y[largest_comp];
  x[largest_comp] = -y[j];

  if (std::abs(y[largest_comp]) == 0.0) {
    // Handle degenerate case, will return the identity matrix
    x = sfm::Vec3d(1.0, 0.0, 0.0);
    y = sfm::Vec3d(0.0, 1.0, 0.0);
  }

  x = x / x.norm();
  y = y / y.norm();

  // Find y as the cross product of plane normal and x
  sfm::Vec3d z = x.cross(y);
  z = z / z.norm();

  // Find the matrix with x, y, z as columns
  for (int row = 0; row < 3; row++) {
    R(row, 0) = x[row];
    R(row, 1) = y[row];
    R(row, 2) = z[row];
  }

  return;
}

// Find the bounding box of the camera centers
void bdBox(sfm::Matrix3d const& EcefToGL,
           std::vector<sfm::Vec3d>& cam_centers,
           // Outputs
           double& x_min, double& x_max,
           double& y_min, double& y_max,
           double& z_min, double& z_max) {

  double big = std::numeric_limits<double>::max();
  x_min = big; y_min = big; z_min = big;
  x_max = -big; y_max = -big; z_max = -big;

  for (std::size_t i = 0; i < cam_centers.size(); i++) {
    sfm::Vec3d trans_center = EcefToGL.mult(cam_centers[i]);
    x_min = std::min(x_min, trans_center[0]);
    x_max = std::max(x_max, trans_center[0]);
    y_min = std::min(y_min, trans_center[1]);
    y_max = std::max(y_max, trans_center[1]);
    z_min = std::min(z_min, trans_center[2]);
    z_max = std::max(z_max, trans_center[2]);
  }
}

// Compute a ground plane, scale the orbital camera positions relative
// to the plane, then plot the cameras and the ground plane.
void SceneRenderer::create_frusta_renderer(void) {

  if (this->scene == nullptr)
    return;

  sfm::Scene::ViewList& views(this->scene->get_views());

  // Cache the original poses on first call. Cleared by on_scene_changed().
  if (this->orig_cam_centers.empty())
    extractSfmCameraPoses(views, this->orig_cam_centers,
                       this->orig_cam2world_vec);
  if (this->orig_cam_centers.empty()) {
    std::cerr << "Error: No cameras found.\n";
    return;
  }

  // Work on copies so the originals are never modified
  std::vector<sfm::Vec3d> cam_centers = this->orig_cam_centers;
  std::vector<sfm::Matrix3d> cam2world_vec = this->orig_cam2world_vec;

  // Find shortest distance from camera center to the origin
  double shortest_dist = find_shortest_distance(cam_centers);
  shortest_dist = std::max(shortest_dist, 0.0001); // avoid division by zero

  // Divide by the shortest distance to normalize the camera centers
  for (size_t cam = 0; cam < cam_centers.size(); cam++)
    cam_centers[cam] = cam_centers[cam] / shortest_dist;

  // Find the mean camera center, use it as normal to the ground plane
  sfm::Vec3d mean_cam_center = find_mean_camera_pos(cam_centers);

  // Will map the cameras so that the ground plane is the x-z plane
  // This is consistent with how OpenGL by default renders this plane
  // as horizontal (z axis pointing to user, y goes up, x goes right).
  sfm::Vec3d y = mean_cam_center; // will change below
  sfm::Matrix3d GLToEcef;
  completeVectorToRotation(y, GLToEcef);

  // Inverse matrix
  sfm::Matrix3d EcefToGL = GLToEcef.transposed();

  // Find the bounding box of the camera centers in the coordinate system
  // having the ground plane as x and z axes. The y axis will point up,
  // consistent with the OpenGL default.
  double x_min = 0, x_max = 0, y_min = 0, y_max = 0, z_min = 0, z_max = 0;
  bdBox(EcefToGL, cam_centers, x_min, x_max, y_min, y_max, z_min, z_max);

  // Max box size and mid point
  double len = std::max(x_max - x_min,
                        std::max(y_max - y_min, z_max - z_min));
  if (len == 0.0)
    len = 1.0; // careful not to divide by zero
  double x_mid = (x_min + x_max)/2.0;
  double y_mid = (y_min + y_max)/2.0;
  double z_mid = (z_min + z_max)/2.0;

  // Rotate the camera centers so that the ground plane is the x-z plane
  // then normalize them to be on the order of 1.0. Rotate
  // the camera orientations as well. Then shift them so that they
  // can be seen in the default OpenGL view. Then put them
  // back to ECEF, before we later transform them again to GL, but after
  // more fine tuning of the rotation transformation.
  double GL_cam_shift_y = 0.2; // cameras are above the ground
  double GL_ground_shift_y = -0.8;
  for (std::size_t i = 0; i < cam_centers.size(); i++) {
    sfm::Vec3d cam_center = cam_centers[i];
    sfm::Vec3d trans_center = EcefToGL.mult(cam_center);
    trans_center[0] = (trans_center[0] - x_mid)/len;
    trans_center[1] = (trans_center[1] - y_mid)/len + GL_cam_shift_y;
    trans_center[2] = (trans_center[2] - z_mid)/len;
    cam_centers[i] = GLToEcef.mult(trans_center);
  }

  if (z_max - z_min > x_max - x_min) {
    // Rotate around the y axis by 90 degrees to show the cameras
    // side-by-side as seen in the OpenGL default coordinate system.
    sfm::Matrix3d R90;
    R90[0] = 0; R90[1] = 0; R90[2] = -1;
    R90[3] = 0; R90[4] = 1; R90[5] = 0;
    R90[6] = 1; R90[7] = 0; R90[8] = 0;
    EcefToGL = R90.mult(EcefToGL);
  }

  // Rotate the camera positions and orientations
  double GL_cam_shift_z = -0.5; // Move the cameras a bit to the back
  for (std::size_t i = 0; i < cam_centers.size(); i++) {
    cam_centers[i] = EcefToGL.mult(cam_centers[i]);
    cam_centers[i][2] += GL_cam_shift_z;
    cam2world_vec[i] = EcefToGL.mult(cam2world_vec[i]);
  }

  // Apply the camera positions and orientations to the views
  applySfmCameraPoses(cam_centers, cam2world_vec, views);

  // Plot the cameras as meshes
  sfm::TriangleMesh::Ptr mesh = sfm::TriangleMesh::create();
  sfm::TriangleMesh::VertexList& verts(mesh->get_vertices());
  sfm::TriangleMesh::FaceList& faces(mesh->get_faces());
  sfm::TriangleMesh::ColorList& colors(mesh->get_vertex_colors());
  sfm::Vec4f color(0, 1, 0, 1); // green
  float size = this->frusta_size_slider->value() / 100.0f;
  for (std::size_t i = 0; i < views.size(); i++) {
    if (views[i].get() == nullptr) {
      std::cerr << "Error: Empty camera.\n";
      continue;
    }

    sfm::SfmCameraInfo const& cam = views[i]->get_camera();
    if (cam.flen == 0.0f) {
      std::cerr << "Error: SfmCamera focal length is 0.\n";
      continue;
    }
    add_camera_to_mesh(cam, size, mesh);
  }

  this->frusta_renderer = sfm::MeshRenderer::create(mesh);
  this->frusta_renderer->set_shader(this->wireframe_shader);
  this->frusta_renderer->set_primitive(GL_LINES);
}

void SceneRenderer::create_ground_renderer(void) {

  sfm::TriangleMesh::Ptr mesh = sfm::TriangleMesh::create();
  sfm::TriangleMesh::VertexList& verts(mesh->get_vertices());
  sfm::TriangleMesh::FaceList& faces(mesh->get_faces());
  sfm::TriangleMesh::ColorList& colors(mesh->get_vertex_colors());
  sfm::Vec4f color(0, 1, 0, 1); // green

  // Draw a ground plane as (x, z) in [-1, 1] x [-1, 1] at some height y.
  double GL_ground_shift_y = -0.8;
  double d = 10.0;
  for (size_t i = 1; i <= 22; i++) {
    double x[2], z[2];
    if (i <= 11) {
      z[0] = 2.0*(i - 6.0)/d; z[1] = z[0];
      x[0] = -1.0; x[1] = 1.0;
    } else {
      x[0] = 2.0*(i - 11 - 6.0)/d; x[1] = x[0];
      z[0] = -1.0; z[1] = 1.0;
    }

    sfm::Vec3d v1(x[0], GL_ground_shift_y, z[0]);
    sfm::Vec3d v2(x[1], GL_ground_shift_y, z[1]);

    verts.push_back(v1);
    verts.push_back(v2);
    faces.push_back(verts.size() - 2);
    faces.push_back(verts.size() - 1);
    colors.push_back(color);
    colors.push_back(color);
  }

  this->ground_renderer = sfm::MeshRenderer::create(mesh);
  this->ground_renderer->set_shader(this->wireframe_shader);
  this->ground_renderer->set_primitive(GL_LINES);
}

void SceneRenderer::create_viewdir_renderer(void) {
  if (this->view == nullptr)
    return;

  sfm::Vec3f campos, viewdir;
  sfm::SfmCameraInfo const& cam(this->view->get_camera());
  cam.fill_camera_pos(*campos);
  cam.fill_viewing_direction(*viewdir);

  sfm::TriangleMesh::Ptr mesh = sfm::TriangleMesh::create();
  sfm::TriangleMesh::VertexList& verts = mesh->get_vertices();
  sfm::TriangleMesh::ColorList& colors = mesh->get_vertex_colors();
  verts.push_back(campos);
  verts.push_back(campos + viewdir * 100.0f);
  colors.push_back(sfm::Vec4f(1.0f, 1.0f, 0.0f, 1.0f));
  colors.push_back(sfm::Vec4f(1.0f, 1.0f, 0.0f, 1.0f));

  this->viewdir_renderer = sfm::MeshRenderer::create(mesh);
  this->viewdir_renderer->set_shader(this->wireframe_shader);
  this->viewdir_renderer->set_primitive(GL_LINES);
}
