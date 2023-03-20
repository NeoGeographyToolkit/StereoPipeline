.. _cgal_tools:

CGAL tools
----------

ASP distributes in the ``bin`` directory the following CGAL tools for
manipulating meshes in .ply format, which are produced with the
``multi_stereo`` (:numref:`multi_stereo`) and ``voxblox_mesh``
(:numref:`voxblox_mesh`) tools::

- rm_connected_components
- smoothe_mesh
- fill_holes
- simplify_mesh

These tools can be built and used independently of ASP. See the
`source code and build instructions
<https://github.com/NeoGeographyToolkit/cgal_tools>`_.

Examples
~~~~~~~~
Remove small connected components from the mesh::

    num_min_faces_in_component=100
    num_components_to_keep=10
    rm_connected_components                  \
      $num_min_faces_in_component            \
      $num_components_to_keep                \
      <input_mesh.ply> <output_mesh.ply>

Mesh smoothing::

    num_iter=1; smoothing_time=5e-5; smoothe_boundary=1
    smoothe_mesh                                  \
      $num_iter $smoothing_time $smoothe_boundary \
      <input_mesh.ply> <output_mesh.ply>

Hole-filling::

    max_hole_diameter=0.8
    max_num_hole_edges=100
    fill_holes                               \
      $max_hole_diameter $max_num_hole_edges \
      <input_mesh.ply> <output_mesh.ply>

Mesh simplification::

    edge_keep_ratio=0.2
    simplify_mesh $edge_keep_ratio       \
      <input_mesh.ply> <output_mesh.ply>

It is very strongly recommended to first run these tools on small
meshes to get a feel for how they work. Meshlab can be used
to inspect the results.


