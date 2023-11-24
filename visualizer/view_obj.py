import open3d as o3d

def visualize(mesh):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(mesh)
    vis.run()
    vis.destroy_window()
    

mesh = o3d.io.read_triangle_mesh("./data/res.obj",True)
mesh.compute_vertex_normals()
#visualize(mesh)
o3d.visualization.draw([mesh])