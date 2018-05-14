//// PCL_EXAMPLE.cpp : 定义控制台应用程序的入口点。
////
//
#include "stdafx.h"
//#include <libpq-fe.h>

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <fstream>
using namespace std;
using namespace pcl;


void createPCDfileByTXT(string txtFileIn,string pcdFileOut)
{
	ifstream in;
	char line[256];
	double d[3];
	
	std::vector<pcl::PointXYZ> vec;
	in.open(txtFileIn);
	while(in.good())
	{
		in.getline(line,256);
		istringstream iss(line);
		iss>>d[0]>>d[1]>>d[2];
		vec.push_back(pcl::PointXYZ(d[0],d[1],d[2]));
	}
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width    = vec.size();  
	cloud.height   = 1;  
	cloud.is_dense = false;  
	cloud.points.resize (cloud.width * cloud.height); 
	for (int i = 0;i<vec.size();++i)
	{
		cloud.points[i] = vec[i];  
	}
	pcl::io::savePCDFileASCII (pcdFileOut, cloud); 
}
//PGconn* connectToDB(char *pghost, char *pgport, char *dbname, char *user, char *pwd)
//{
//	char *pgoptions, *pgtty;
//	pgoptions = NULL;
//	pgtty = NULL;
//	PGconn *conn = PQsetdbLogin(pghost, pgport, pgoptions, pgtty, dbname, user, pwd);
//	if (PQstatus(conn) == CONNECTION_BAD)
//	{
//		return nullptr;
//	}
//	else
//	{
//		return conn;
//	}
//}
//
//
//void createPCDfile(string pcdFile,char *pghost, char *pgport, char *dbname, char *user, char *pwd)
//{
//	PGconn* conn = connectToDB(pghost,pgport,dbname,user,pwd);
//	if(!conn)
//	{
//		cout<<"connect to database fialed.";
//		exit(-1);
//	}
//	PGresult* res = PQexec(conn,"select * from tb_pcd");	
//	if(PQntuples(res) == 0)
//	{
//		cout<<"no pcd data!";
//		exit(-1);
//	}
//
//	pcl::PointCloud<pcl::PointXYZ> cloud;
//	cloud.width    = PQntuples(res);  
//    cloud.height   = 1;  
//	cloud.is_dense = false;  
//    cloud.points.resize (cloud.width * cloud.height);  
//  
//	for(int i = 0;i<PQntuples(res);++i)
//	{
//		double x = atof(PQgetvalue(res,i,0));
//		double y = atof(PQgetvalue(res,i,1));
//		double z = atof(PQgetvalue(res,i,2));
//		cloud.points[i].x = x;  
//		cloud.points[i].y = y;  
//		cloud.points[i].z = z;
//	}
//	pcl::io::savePCDFileASCII (pcdFile, cloud); 
//}

int main(int argc, char** argv)
{
	/*if (argc != 7 && argc != 2)
	{
	char errMsg[1024];
	sprintf_s(errMsg,"Syntax is: %s [output.pcd] <pghost> <pgport> <dbname> <user> <pwd>\n", argv[0]);
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_INTENSITY |
	FOREGROUND_RED);
	cerr<< errMsg;
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_GREEN |
	FOREGROUND_RED|FOREGROUND_BLUE);
	return -1;
	}
	char* outPutFile,*pghost,*pgport,*dbname,*user,*pwd;
	if (argc == 7)
	{
	outPutFile  = argv[1];
	pghost = argv[2];
	pgport = argv[3];
	dbname = argv[4];
	user = argv[5];
	pwd = argv[6];
	}
	else
	{
	outPutFile  = argv[1];
	pghost = "127.0.0.1";
	pgport = "5432";
	dbname = "myDB3";
	user = "postgres";
	pwd = "";
	}

	createPCDfile(outPutFile,pghost,pgport,dbname,user,pwd);*/
	
	if (argc != 3)
	{
		char errMsg[1024];
		sprintf_s(errMsg,"Syntax is: %s [input.txt] [output.pcd]\n", argv[0]);
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_INTENSITY |
			FOREGROUND_RED);
		cerr<< errMsg;
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE),FOREGROUND_GREEN |
			FOREGROUND_RED|FOREGROUND_BLUE);
		return -1;
	}
	createPCDfileByTXT(argv[1],argv[2]);
	return 0;
}    



