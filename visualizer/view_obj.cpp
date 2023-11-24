#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char** argv)
{
	std::string objPath = "./data/res.obj";
	pcl::TextureMesh mesh;
	pcl::io::loadOBJFile(objPath, mesh);
	pcl::TextureMesh mesh2;
	pcl::io::loadPolygonFileOBJ(objPath, mesh2);
	mesh2.tex_materials = mesh.tex_materials;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
	viewer->addTextureMesh(mesh2, "mesh");
	while (!viewer->wasStopped())  // 在按下 "q" 键之前一直会显示窗口
	{
		viewer->spinOnce();
	}
	return 0;
}