#include <fstream>
#include <iostream>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/visualization/pcl_visualizer.h>


std::ifstream& go_to_line(std::ifstream& file, unsigned int num)
{
	file.seekg(std::ios::beg);
	for (int i = 0; i < num - 1; ++i) 
		file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	return file;
}


bool read_cam_pose_file(std::string filename, pcl::TextureMapping<pcl::PointXYZ>::Camera& cam) 
{
	std::ifstream myReadFile;
	myReadFile.open(filename.c_str(), ios::in);
	if (!myReadFile.is_open())
	{
		PCL_ERROR("Error opening file %d\n", filename.c_str());
		return false;
	}
	myReadFile.seekg(ios::beg);

	float val;

	go_to_line(myReadFile, 1);
	myReadFile >> val;
	cam.pose(0, 3) = val;  // TX
	myReadFile >> val;
	cam.pose(1, 3) = val;  // TY
	myReadFile >> val;
	cam.pose(2, 3) = val;  // TZ

	go_to_line(myReadFile, 2);
	myReadFile >> val;
	cam.pose(0, 0) = val;
	myReadFile >> val;
	cam.pose(0, 1) = val;
	myReadFile >> val;
	cam.pose(0, 2) = val;

	myReadFile >> val;
	cam.pose(1, 0) = val;
	myReadFile >> val;
	cam.pose(1, 1) = val;
	myReadFile >> val;
	cam.pose(1, 2) = val;

	myReadFile >> val;
	cam.pose(2, 0) = val;
	myReadFile >> val;
	cam.pose(2, 1) = val;
	myReadFile >> val;
	cam.pose(2, 2) = val;

	cam.pose(3, 0) = 0.0;
	cam.pose(3, 1) = 0.0;
	cam.pose(3, 2) = 0.0;
	cam.pose(3, 3) = 1.0;  // Scale

	go_to_line(myReadFile, 5);
	myReadFile >> val;
	cam.focal_length_w = val;
	myReadFile >> val;
	cam.focal_length_h = val;
	myReadFile >> val;
	cam.center_w = val;
	myReadFile >> val;
	cam.center_h = val;
	myReadFile >> val;
	cam.height = val;
	myReadFile >> val;
	cam.width = val;

	myReadFile.close();
	return true;
}


int main(int argc, char** argv) 
{
	pcl::PolygonMesh triangles;
	pcl::io::loadPolygonFilePLY("./data/obj.ply", triangles);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(triangles.cloud, *cloud);

	// Create the texturemesh object that will contain our UV-mapped mesh
	pcl::TextureMesh mesh;
	mesh.cloud = triangles.cloud;
	std::vector<pcl::Vertices> polygon_1;

	// push faces into the texturemesh object
	polygon_1.resize(triangles.polygons.size());
	for (size_t i = 0; i < triangles.polygons.size(); ++i) 
		polygon_1[i] = triangles.polygons[i];

	mesh.tex_polygons.push_back(polygon_1);

	// Load textures and cameras poses and intrinsics
	pcl::texture_mapping::CameraVector my_cams;

	const int nums = 3;
	std::string prefix = "./data/color";
	std::vector<std::string> filenames;
	for (size_t i = 0; i < nums; i++)
	{
		std::stringstream ss;
		ss << std::setw(3) << std::setfill('0') << i;
		std::string filename = prefix + ss.str() + ".txt";
		filenames.push_back(filename);
	}

	for (int i = 0; i < nums; ++i)
	{
		pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
		read_cam_pose_file(filenames[i], cam);
		cam.texture_file = filenames[i].substr(0, filenames[i].size() - 3) + "png";
		my_cams.push_back(cam);
	}

	// Create materials for each texture (and one extra for occluded faces)
	mesh.tex_materials.resize(my_cams.size() + 1);
	for (int i = 0; i <= my_cams.size(); ++i) 
	{
		pcl::TexMaterial mesh_material;
		mesh_material.tex_Ka.r = 0.2f;
		mesh_material.tex_Ka.g = 0.2f;
		mesh_material.tex_Ka.b = 0.2f;

		mesh_material.tex_Kd.r = 0.8f;
		mesh_material.tex_Kd.g = 0.8f;
		mesh_material.tex_Kd.b = 0.8f;

		mesh_material.tex_Ks.r = 1.0f;
		mesh_material.tex_Ks.g = 1.0f;
		mesh_material.tex_Ks.b = 1.0f;

		mesh_material.tex_d = 1.0f;
		mesh_material.tex_Ns = 75.0f;
		mesh_material.tex_illum = 2;

		std::stringstream tex_name;
		tex_name << "material_" << i;
		tex_name >> mesh_material.tex_name;

		if (i < my_cams.size())
			mesh_material.tex_file = my_cams[i].texture_file;
		else
			mesh_material.tex_file = "occluded.jpg";

		mesh.tex_materials[i] = mesh_material;
	}

	// Sort faces
	pcl::TextureMapping<pcl::PointXYZ> tm;  // TextureMapping object that will perform the sort
	tm.textureMeshwithMultipleCameras(mesh, my_cams);

	// compute normals for the mesh
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	// Concatenate XYZ and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	pcl::toPCLPointCloud2(*cloud_with_normals, mesh.cloud);
	pcl::io::saveOBJFile("./data/res.obj", mesh, 5);

	return 0;
}