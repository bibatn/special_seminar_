#include "mls_mpi.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <numeric>



//#include <pcl/type_traits.h>
#include <pcl/surface/mls.h>
#include <pcl/common/common.h> // for getMinMax3D
#include <pcl/common/copy_point.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/search/organized.h> // for OrganizedNeighbor
#include <pcl/conversions.h>

#include <Eigen/Geometry> // for cross
#include <Eigen/LU> // for inverse

#include <chrono>

#include "mls_mpi.h"
#include "mpi.h"

using namespace std::chrono_literals;
#define _OPENMP

int
main (int argc, char ** argv)
{
  int size, rank;
  MPI_Init(&argc, &argv);
  MPI_Comm_size(MPI_COMM_WORLD, &size);
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);

  mls_mpi cloud_part(rank, size);
  const std::string filename = "1.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCLPointCloud2 point_cloud2;
  cloud_part.loadPCDFile(filename, point_cloud2);
  pcl::fromPCLPointCloud2(point_cloud2, *cloud);

//  pcl::io::loadPCDFile ("1.pcd", *cloud);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

//  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.09);

  mls.setNumberOfThreads(1);


  // Reconstruct
  std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
  uint64_t T1 = now.count();
  mls.process (*mls_points);
  now = std::chrono::high_resolution_clock::now().time_since_epoch();
  uint64_t T2 = now.count();

  if(rank==0)
  {
    std::cout << "TIME: " << T2-T1 << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

  //  pcl::visualization::PCLVisualizer viewer1("Raw");
  //  viewer1.addPointCloud<pcl::PointXYZRGB>(out_colored, "filtered_green");

  //  while (!viewer1.wasStopped()) {
  //      viewer1.spinOnce();
  //  }

    std::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("test"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v1(mls_points,0,250,0);
    view->setBackgroundColor(0.0,0,0);
    view->addPointCloud<pcl::PointNormal>(mls_points,v1,"sample1");
  //  view->addPointCloudNormals<pcl::PointNormal>(mls_points,50,20,"normal1");
  //  view->addCoordinateSystem(1.0);
    view->spin();


  //  pcl::io::savePCDFile ("bun0-mls.pcd", *mls_points);

  }



  MPI_Finalize();
}
