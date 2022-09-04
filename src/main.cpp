#include "mls_mpi.h"
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/search/organized.h> // for OrganizedNeighbor
#include <pcl/conversions.h>


#include <chrono>
#include "mpi.h"

using namespace std::chrono_literals;

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
  cloud_part.read(filename, point_cloud2);
//  if(size > 1)
//  {
//    //_
//    pcl::PCLPointCloud2 left_bord;
//    pcl::PCLPointCloud2 right_bord;
//    left_bord.data.resize(point_cloud2.height);
//    right_bord.data.resize(point_cloud2.height);
//    left_bord.width = 1; right_bord.width = 1;
//    left_bord.height = point_cloud2.height; right_bord.height = point_cloud2.height;
//
//    MPI_Sendrecv(&point_cloud2.data[(point_cloud2.height-1)*point_cloud2.width], point_cloud2.height, MPI_UINT8_T, (rank + 1) % size, 777, &left_bord.data[0], point_cloud2.height, MPI_UINT8_T, (rank - 1 + size) % size, 777, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
//    MPI_Sendrecv(&point_cloud2.data[0], point_cloud2.height, MPI_UINT8_T, (rank - 1 + size) % size, 777, &right_bord.data[0], point_cloud2.height, MPI_UINT8_T, (rank + 1) % size, 777, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
//
//    point_cloud2 = left_bord + point_cloud2 + right_bord;
//    //
//
//  }
  pcl::fromPCLPointCloud2(point_cloud2, *cloud);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

  cloud_part.setInputCloud (cloud);
  cloud_part.setPolynomialOrder (2);
  cloud_part.setSearchMethod (tree);
  cloud_part.setSearchRadius (0.09);

  cloud_part.setNumberOfThreads(1);


  // Reconstruct
  std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
  uint64_t T1 = now.count();
  cloud_part.process (*mls_points);
  now = std::chrono::high_resolution_clock::now().time_since_epoch();
  uint64_t T2 = now.count();

  if(rank==0)
  {
    std::cout << "TIME: " << T2-T1 << std::endl;
  //  pcl::io::savePCDFile ("bun0-mls.pcd", *mls_points);

  }



  MPI_Finalize();
}
