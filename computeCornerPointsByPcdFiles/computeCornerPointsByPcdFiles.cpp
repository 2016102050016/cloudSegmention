// computeCornerPointsByPcdFiles.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/console/print.h>
#include <fstream>
#include <string>

using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	if (argc<2)
	{
		pcl::console::print_error ("Syntax is: %s [input1.pcd] [input2.pcd] ... [inputN.pcd]\n", argv[0]);
		return -1;
	}
	//获得exe路径
	char szapipath[MAX_PATH];
	memset(szapipath, 0, MAX_PATH);
	GetModuleFileNameA(NULL, szapipath, MAX_PATH);
	std::string mEXEPath = std::string(szapipath);
	int pos = mEXEPath.rfind('\\');
	mEXEPath = mEXEPath.substr(0, pos);
	// Read in the cloud data
	std::vector<std::string> vecPcdFile;
	for (int i = 1;i<argc;++i)
	{
		vecPcdFile.push_back(argv[i]);
	}
	/*vecPcdFile.push_back("pcdFile\\table_scene_lms400_plane_0.pcd");
	vecPcdFile.push_back("pcdFile\\table_scene_lms400_plane_1.pcd");
	vecPcdFile.push_back("pcdFile\\table_scene_lms400_plane_2.pcd");
	vecPcdFile.push_back("pcdFile\\table_scene_lms400_plane_3.pcd");
	vecPcdFile.push_back("pcdFile\\table_scene_lms400_plane_4.pcd");
	vecPcdFile.push_back("pcdFile\\table_scene_lms400_plane_5.pcd");*/
	pcl::PCDReader reader;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vecCloud;

	for (int i = 0;i<vecPcdFile.size();++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		reader.read (vecPcdFile[i], *cloud);
		vecCloud.push_back(cloud);
	}

	ofstream out;
	char cfile[255];
	std::string cmd = mEXEPath+"\\cvFitPlane\\cvFitPlane.exe ";
	for (int i = 0;i<vecCloud.size();++i)
	{
		sprintf_s(cfile,"%s\\cvFitPlane\\fileCache\\c%d.txt",mEXEPath.c_str(),i);
		cmd += (std::string(cfile) + " ");
		out.open(cfile);
		for (size_t j = 0;j<vecCloud[i]->size();++j)
		{
			out<<vecCloud[i]->points[j].x<<" "<<vecCloud[i]->points[j].y<<" "<<vecCloud[i]->points[j].z<<std::endl;
		}
		out.close();
	}

	system(cmd.c_str());
	return 0;
}



