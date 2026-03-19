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

#include <asp/SfmView/MeshRenderer.h>

#include <stdexcept>

namespace sfm {

// VertexBuffer

VertexBuffer::Ptr VertexBuffer::create() {
  Ptr vb(new VertexBuffer);
  vb->buf.create();
  return vb;
}

void VertexBuffer::set_data(GLfloat const* data, GLsizei elems, GLint vpv) {
  this->datatype = GL_FLOAT;
  this->vpv = vpv;
  this->elems = elems;

  this->buf.setUsagePattern(QOpenGLBuffer::StaticDraw);
  this->buf.bind();
  this->buf.allocate(data, elems * vpv * sizeof(GLfloat));
}

void VertexBuffer::set_indices(GLuint const* data, GLsizei num_indices) {
  this->buf = QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  this->buf.create();
  this->datatype = GL_UNSIGNED_INT;
  this->vpv = 3;
  this->elems = num_indices;

  this->buf.setUsagePattern(QOpenGLBuffer::StaticDraw);
  this->buf.bind();
  this->buf.allocate(data, num_indices * sizeof(GLuint));
}

// VertexArray

VertexArray::VertexArray(void) {
  this->vao.create();
  this->primitive = GL_TRIANGLES;
}

VertexArray::~VertexArray(void) {
  this->vao.destroy();
}

void VertexArray::set_primitive(GLuint primitive) {
  this->primitive = primitive;
}

void VertexArray::set_vertex_vbo(VertexBuffer::Ptr vbo) {
  this->vert_vbo = vbo;
}

void VertexArray::set_index_vbo(VertexBuffer::Ptr vbo) {
  this->index_vbo = vbo;
}

void VertexArray::add_vbo(VertexBuffer::Ptr vbo,
                          std::string const& name) {
  this->vbo_list.push_back(std::make_pair(vbo, name));
}

void VertexArray::set_shader(QOpenGLShaderProgram* shader) {
  this->shader = shader;
}

void VertexArray::reset_vertex_array(void) {
  this->vert_vbo.reset();
  this->index_vbo.reset();
  this->vbo_list.clear();
  this->vao.destroy();
  this->vao.create();
}

void VertexArray::assign_attrib(BoundVBO const& bound_vbo) {
  VertexBuffer::Ptr vbo = bound_vbo.first;
  std::string const& name = bound_vbo.second;

  GLint location = this->shader->attributeLocation(name.c_str());
  if (location < 0)
    return;

  vbo->buf.bind();
  glFunctions()->glVertexAttribPointer(location, vbo->vpv,
    vbo->datatype, GL_TRUE, 0, nullptr);
  glFunctions()->glEnableVertexAttribArray(location);
}

void VertexArray::draw(void) {
  if (this->vert_vbo == nullptr)
    throw std::runtime_error("No vertex VBO set!");

  if (this->shader == nullptr)
    throw std::runtime_error("No shader program set!");

  auto f = glFunctions();
  this->vao.bind();

  this->shader->bind();

  this->assign_attrib(BoundVBO(this->vert_vbo, SFM_ATTRIB_POSITION));

  for (std::size_t i = 0; i < this->vbo_list.size(); ++i)
    this->assign_attrib(this->vbo_list[i]);

  if (this->index_vbo != nullptr) {
    this->index_vbo->buf.bind();
    f->glDrawElements(this->primitive, this->index_vbo->elems,
      GL_UNSIGNED_INT, nullptr);
  } else {
    f->glDrawArrays(this->primitive, 0, this->vert_vbo->elems);
  }

  this->shader->release();
  this->vao.release();
}

// MeshRenderer

void MeshRenderer::set_mesh(sfm::TriangleMesh::ConstPtr mesh) {
  if (mesh == nullptr)
    throw std::invalid_argument("Got null mesh");

  this->reset_vertex_array();

  sfm::TriangleMesh::VertexList const& verts(mesh->get_vertices());
  sfm::TriangleMesh::FaceList const& faces(mesh->get_faces());
  sfm::TriangleMesh::NormalList const& vnormals(mesh->get_vertex_normals());
  sfm::TriangleMesh::ColorList const& vcolors(mesh->get_vertex_colors());
  sfm::TriangleMesh::TexCoordList const& vtexuv(mesh->get_vertex_texcoords());

  {
    VertexBuffer::Ptr vbo = VertexBuffer::create();
    vbo->set_data(&verts[0][0], (GLsizei)verts.size(), 3);
    this->set_vertex_vbo(vbo);
  }

  if (!faces.empty()) {
    VertexBuffer::Ptr vbo = VertexBuffer::create();
    vbo->set_indices(&faces[0], (GLsizei)faces.size());
    this->set_index_vbo(vbo);
  }

  if (!vnormals.empty()) {
    VertexBuffer::Ptr vbo = VertexBuffer::create();
    vbo->set_data(&vnormals[0][0], (GLsizei)vnormals.size(), 3);
    this->add_vbo(vbo, SFM_ATTRIB_NORMAL);
  }

  if (!vcolors.empty()) {
    VertexBuffer::Ptr vbo = VertexBuffer::create();
    vbo->set_data(&vcolors[0][0], (GLsizei)vcolors.size(), 4);
    this->add_vbo(vbo, SFM_ATTRIB_COLOR);
  }

  if (!vtexuv.empty()) {
    VertexBuffer::Ptr vbo = VertexBuffer::create();
    vbo->set_data(&vtexuv[0][0], (GLsizei)vtexuv.size(), 2);
    this->add_vbo(vbo, SFM_ATTRIB_TEXCOORD);
  }
}

MeshRenderer::Ptr MeshRenderer::create(sfm::TriangleMesh::ConstPtr mesh) {
  return Ptr(new MeshRenderer(mesh));
}

MeshRenderer::MeshRenderer(sfm::TriangleMesh::ConstPtr mesh) {
  this->set_mesh(mesh);
}

}
