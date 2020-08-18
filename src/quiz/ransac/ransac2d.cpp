/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
//#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
//#include "../../processPointClouds.cpp"

// custom includes
#include <cmath>

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

void getLine(float x1, float y1, float x2, float y2, float &a, float &b, float &c)
{
       // (x- p1X) / (p2X - p1X) = (y - p1Y) / (p2Y - p1Y) 
       a = y1 - y2; // Note: this was incorrectly "y2 - y1" in the original answer
       b = x2 - x1;
       c = x1 * y2 - x2 * y1;
}

float distance(float pct1X, float pct1Y, float pct2X, float pct2Y, float pct3X, float pct3Y)
{
     float a, b, c;
     getLine(pct2X, pct2Y, pct3X, pct3Y, a, b, c);
  
     return fabs(a * pct1X + b * pct1Y + c) / sqrt(a * a + b * b);
}

float * crossProduct(float v_A[], float v_B[]) {
  	// Cross Product (i, j, k)
  	static float c_P[3];
  	// i =  (y2-y1) * (z3-z1)-(z2-z1) * (y3-y1)
   	c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
   	c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
	c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
  
   	return c_P;
}

void getPlane(float x1, float y1, float z1,
              float x2, float y2, float z2,
              float x3, float y3, float z3,
              float &a, float &b, float &c, float &d)
{
   	float *cp;
  	
  	float v1[3];
  	v1[0] = x2 - x1;
  	v1[1] = y2 - y1;
  	v1[2] = z2 - z1;
  
  	float v2[3];
  	v2[0] = x3 - x1;
  	v2[1] = y3 - y1;
  	v2[2] = z3 - z1;
    
  	cp = crossProduct(v1, v2); 

    a = cp[0];
    b = cp[1];
    c = cp[2];
  	d = -1 * ( cp[0]*x1 + cp[1]*y1 + cp[2]*z1 );
}

float distance3D(float pct1X, float pct1Y, float pct1Z,
                 float pct2X, float pct2Y, float pct2Z,
                 float pct3X, float pct3Y, float pct3Z,
                 float pct4X, float pct4Y, float pct4Z)
{
     float a, b, c, d;
  
     getPlane(pct2X, pct2Y, pct2Z,
              pct3X, pct3Y, pct3Z,
              pct4X, pct4Y, pct4Z,
              a,b,c,d);
  
     return fabs(a * pct1X + b * pct1Y + c * pct1Z + d) / sqrt(a * a + b * b + c * c);
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
    std::unordered_set<int> inliers;
	
  	srand(time(NULL));
	
  	double dist;

  	for (int i = 0; i < maxIterations; i++){
      
      inliers.clear();
            
      // Create a plan with 3 random points
      while (inliers.size() < 3)
        inliers.insert(rand()%(cloud->points.size()));
      
      float p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z;
      
      // Analyse the first three points
      // Point 1
      auto itr = inliers.begin();      
      p1x = cloud->points[*itr].x;
      p1y = cloud->points[*itr].y;
      p1z = cloud->points[*itr].z;
      // Point 2
      itr++;
      p2x = cloud->points[*itr].x;
      p2y = cloud->points[*itr].y;
      p2z = cloud->points[*itr].z;
      // Point 3
      itr++;
      p3x = cloud->points[*itr].x;
      p3y = cloud->points[*itr].y;
      p3z = cloud->points[*itr].z;
      
      // Iterate through the all the points
      for( int index = 0; index < cloud->points.size(); index++){
        
        if (inliers.count(index)>0)
          continue;
        
        pcl::PointXYZ point = cloud->points[index];
        float px = point.x;
        float py = point.y;
        float pz = point.z;

        // Measure the distance
        dist = distance3D(px, py, pz,       // Point of interest
                        p1x, p1y, p1z,    // 1st point of plane
                        p2x, p2y, p2z,    // 2nd point of plane
                        p3x, p3y, p3z);   // 3th point of plane

        // Insert points within distanceTol
        if (dist < distanceTol)
          inliers.insert(index);     

      }

      // Replace inliersResult with the bigger one
      if (inliers.size() > inliersResult.size())
        inliersResult = inliers;
    
    }
  
  return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.2);

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
