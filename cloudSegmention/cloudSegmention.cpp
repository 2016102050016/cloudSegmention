// cloudSegmention.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pcl/console/print.h"

int main(int argc, char** argv)
{
	//���exe·��
	char szapipath[MAX_PATH];
	memset(szapipath, 0, MAX_PATH);
	GetModuleFileNameA(NULL, szapipath, MAX_PATH);
	std::string mEXEPath = std::string(szapipath);
	int pos = mEXEPath.rfind('\\');
	mEXEPath = mEXEPath.substr(0, pos);

	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2), cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	
	// Fill in the cloud data
	pcl::PCDReader reader;
	if (argc == 2)
	{
		reader.read(argv[1], *cloud_blob);
	}
	else
	{
		pcl::console::print_error ("Syntax is: %s [input.pcd]\n", argv[0]);
		return -1;
	}
		

	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	//sor.filter(*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2(*cloud_blob, *cloud_filtered);//ת��Ϊģ��<Template>����

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	// Write the downsampled version to disk
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>(mEXEPath + "\\house_downsampled.pcd", *cloud_filtered, false);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true); //���öԹ��Ƶ�ģ�Ͳ��������Ż�����
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);//
	seg.setMethodType(pcl::SAC_RANSAC); // �������ĸ�����������Ʒ���
	seg.setMaxIterations(500);
	seg.setDistanceThreshold(0.01);////�����ж��Ƿ�Ϊģ���ڵ�ľ�����ֵ
	
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > vecCloud;

	std::vector<std::string> vecPcdFiles;//�洢�ָ�õ���pcd�ļ�
	
	// While 30% of the original cloud is still there
	while (cloud_filtered->points.size() > 0.2 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			//std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		//std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
		vecCloud.push_back(cloud_p);
		std::stringstream ss;
		ss << mEXEPath +"\\pcdFile\\building_plane_" << i << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);
		//vecPcdFiles.push_back(ss.str());
		// Create the filtering object
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);//����
		i++;
	}
	
	/*std::string cmd(mEXEPath +"\\computeCornerPointsByPcdFiles.exe ");
	for (int i = 0;i<vecPcdFiles.size();++i)
	{
		cmd += (vecPcdFiles[i]+" ");
	}
	system(cmd.c_str());*/

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
	return (0);
}

//void main()
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PCDReader reader;
//	reader.read("d:\\123d.pcd", *cloud_blob);
//	//����һ��ģ�Ͳ����������ڼ�¼���
//	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//	//inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����
//	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//	// ����һ���ָ���
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	// Optional
//	seg.setOptimizeCoefficients (true);
//	// Mandatory-����Ŀ�꼸����״
//	seg.setModelType (pcl::SACMODEL_LINE);
//	//�ָ�������������
//	seg.setMethodType (pcl::SAC_RANSAC);
//	//����������̷�Χ
//	seg.setDistanceThreshold (0.1);
//	//�������
//	seg.setInputCloud (cloud_blob);
//	//�ָ����
//	seg.segment (*inliers, *coefficients);
//
//	ofstream out;
//	out.open("d:\\line.txt");
//	//out<<coefficients->values[0]<<" "<<coefficients->values[1]<<" "<<coefficients->values[2];
//	out<<*coefficients;
//	out.close();
//
//	pcl::visualization::PCLVisualizer viewer("demo");
//	viewer.addPointCloud(cloud_blob);
//	viewer.addLine(*coefficients);
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//	}
//}