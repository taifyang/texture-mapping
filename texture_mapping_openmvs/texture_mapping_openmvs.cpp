#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <opencv2/opencv.hpp>
#include <OpenMVS/MVS/Common.h>
#include <OpenMVS/MVS/Scene.h>
#include <OpenMVS/Common/Types.h>


int main()
{
	const int numViews = 3;
	std::string prefix = "./data/color";
	std::vector<cv::Mat> v_color;
	double t1, t2, t3;
	double r11, r12, r13, r21, r22, r23, r31, r32, r33;
	std::vector<Eigen::Matrix4d> poses;
	for (size_t i = 0; i < numViews; i++)
	{
		std::stringstream ss;
		ss << std::setw(3) << std::setfill('0') << i;
		cv::Mat img = cv::imread(prefix + ss.str() + ".png");
		std::cout << prefix + ss.str() + ".png" << std::endl;
		v_color.push_back(img);

		std::ifstream f(prefix + ss.str() + ".txt");
		f >> t1 >> t2 >> t3
			>> r11 >> r12 >> r13
			>> r21 >> r22 >> r23
			>> r31 >> r32 >> r33;

		Eigen::Matrix4d pose;
		pose << r11, r12, r13, t1,
			r21, r22, r23, t2,
			r31, r32, r33, t3,
			0, 0, 0, 1;
		std::cout << pose << std::endl;

		poses.push_back(pose);
	}

	MVS::Scene MVS_scene(8);
	MVS_scene.platforms.Reserve(numViews);
	MVS_scene.images.Reserve(numViews);

	for (size_t i = 0; i < numViews; i++)
	{
		MVS::Image& image = MVS_scene.images.AddEmpty();
		image.image = cv::MatExpr(v_color[i]);
		image.platformID = i;
		image.ID = i;
		image.height = 1080;
		image.width = 1920;
		image.cameraID = 0;
		image.poseID = 0;

		MVS::Platform& platform = MVS_scene.platforms.AddEmpty();
		MVS::Platform::Camera& camera = platform.cameras.AddEmpty();

		//内参
		camera.K(0, 0) = 1.045238890839152646e+03;
		camera.K(1, 1) = 1.046345842268217893e+03;
		camera.K(0, 2) = 9.518466822597985129e+02;
		camera.K(1, 2) = 5.069212131671924340e+02;
		camera.K = camera.GetScaledK(REAL(1) / MVS::Camera::GetNormalizationScale(image.height, image.width));
		camera.R = RMatrix::IDENTITY;
		camera.C = CMatrix::ZERO;

		//外参
		MVS::Platform::Pose & openMVS_pose = platform.poses.AddEmpty();
		openMVS_pose.R = poses[i].block<3, 3>(0, 0);
		openMVS_pose.R = openMVS_pose.R.inv();
		openMVS_pose.C = poses[i].block<3, 1>(0, 3);

		image.UpdateCamera(MVS_scene.platforms);
	}

	pcl::PolygonMesh triangles;
	pcl::io::loadPolygonFilePLY("./data/obj.ply", triangles);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(triangles.cloud, *cloud);

	MVS_scene.mesh.vertices.Resize(cloud->size());
	MVS_scene.mesh.faces.Resize(triangles.polygons.size());

	for (size_t i = 0; i < cloud->size(); i++)
	{
		MVS_scene.mesh.vertices[i].x = cloud->points[i].x;
		MVS_scene.mesh.vertices[i].y = cloud->points[i].y;
		MVS_scene.mesh.vertices[i].z = cloud->points[i].z;
	}
	for (size_t i = 0; i < triangles.polygons.size(); i++)
	{
		MVS_scene.mesh.faces[i].x = triangles.polygons[i].vertices[0];
		MVS_scene.mesh.faces[i].y = triangles.polygons[i].vertices[1];
		MVS_scene.mesh.faces[i].z = triangles.polygons[i].vertices[2];
	}

	MVS_scene.TextureMesh(0, 1920, 0.0, 0.0, 1, 1, 0, 0);
	MVS_scene.mesh.Save("./data/res.obj");

	return 0;
}