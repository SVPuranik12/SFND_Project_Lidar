/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	/** FOR LINE **/
    /*
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size() < 2)
			inliers.insert(rand()%(cloud->points.size()));

		float x1, x2, y1, y2;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float A, B, C;

		A = (x1 - x2);
		B = (y1 - y2);
		C = (x1*y2 - x2*y1);

		for (int index = 0; index < cloud->points.size(); index ++)
		{
			if (inliers.count(index) > 0)
				continue;
			
			pcl::PointXYZ point = cloud->points[index];

			float x3, y3, distance;
			x3 = point.x;
			y3 = point.y;

			distance =  fabs(A*x3 + B*y3 + C)/sqrt(A*A + B*B);

			if (distance < distanceTol)
				inliers.insert(index);

			if (inliers.size() > inliersResult.size())
			{
				inliersResult = inliers;
			}
		} // For loop ended here
	} // while loop ended here

    */

	/** FOR PLANE **/
		while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size() < 3) // selscting 3 random points
			inliers.insert(rand()%(cloud->points.size()));

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float A, B, C, D;

		A =   ((y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1));
		B = - ((x2 - x1)*(z3 - z1) - (z2 - z1)*(x3 - x1));
		C =   ((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1));
		D = - ( A*x1 + B*y1 + C*z1);

		for (int index = 0; index < cloud->points.size(); index ++)
		{
			if (inliers.count(index) > 0)
				continue;
			
			pcl::PointXYZ point = cloud->points[index];

			float x_ref, y_ref, z_ref, distance;
			x_ref = point.x;
			y_ref = point.y;
			z_ref = point.z;

			distance =  fabs(A*x_ref + B*y_ref + C*z_ref + D)/sqrt(A*A + B*B + C*C);

			if (distance < distanceTol)
				inliers.insert(index);

			if (inliers.size() > inliersResult.size())
			{
				inliersResult = inliers;
			}
		} // For loop ended here
	} // while loop ended here

	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	// Create 3d data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 500, 0.25);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
