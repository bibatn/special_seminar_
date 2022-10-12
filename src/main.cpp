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

void Build_derived_type(pcl::PointNormal * indata, MPI_Datatype* message_type_ptr)
{
    int block_lengths[3];
    MPI_Aint displacements[3];
    MPI_Aint addresses[4];

    MPI_Datatype typelist[3];

    typelist[0] = MPI_FLOAT;
    typelist[1] = MPI_FLOAT;
    typelist[2] = MPI_FLOAT;
/* Определить количество элементов каждого типа */

    block_lengths[0] = block_lengths[1] = block_lengths[2] = 4;

/* Вычислить смещения элементов
* относительно indata */
    MPI_Get_address(indata, &addresses[0]);

    MPI_Get_address(&(indata->data), &addresses[1]);
    MPI_Get_address(&(indata->data_n), &addresses[2]);
    MPI_Get_address(&(indata->data_c), &addresses[3]);

    displacements[0] = addresses[1] - addresses[0];
    displacements[1] = addresses[2] - addresses[0];
    displacements[2] = addresses[3] - addresses[0];

    MPI_Type_create_struct(3, block_lengths, displacements, typelist, message_type_ptr);

    MPI_Type_commit(message_type_ptr);
}

int
main (int argc, char ** argv)
{
  int size, rank;
  MPI_Init(&argc, &argv);
  MPI_Comm_size(MPI_COMM_WORLD, &size);
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);

  mls_mpi cloud_part(rank, size);
  const std::string filename = argv[1];
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCLPointCloud2 point_cloud2;
  cloud_part.read(filename, point_cloud2);
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

  std::vector<int> sizes;
  std::vector<int> displs;
  std::vector<int> recvcounts;
  sizes.resize(size);
  displs.resize(size);
  recvcounts.resize(size);
  int size_of_cloud_part = mls_points->size();
  MPI_Gather(&size_of_cloud_part, 1, MPI_INT, &sizes[0],1, MPI_INT, 0, MPI_COMM_WORLD);
  displs[0] = 0;
  for(int i = 1; i < sizes.size(); i++)
      displs[i] = displs[i-1] + sizes[i];
  for(int i = 0; i < sizes.size(); i++)
      recvcounts[i] = sizes[i];
  pcl::PointCloud<pcl::PointNormal>::Ptr union_cloud(new pcl::PointCloud<pcl::PointNormal>);
  union_cloud->points.resize(std::accumulate(sizes.begin(), sizes.end(), 0));
  union_cloud->height = 1;
  union_cloud->width = std::accumulate(sizes.begin(), sizes.end(), 0);
  union_cloud->is_dense = true;

  MPI_Datatype message_type;
  Build_derived_type(&union_cloud->points[0], &message_type);
  MPI_Gatherv(&mls_points->points[0], mls_points->size(), message_type, &union_cloud->points[0], &recvcounts[0], &displs[0], message_type, 0, MPI_COMM_WORLD);

  if(rank==0)
  {
    std::cout << "TIME: " << T2-T1 << std::endl;
//    for(int i = 0; i<sizes.size(); i++)
//    {
//        std::cout<< sizes[i] << "  " << std::endl;
//    }



    std::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("test"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v1(union_cloud,0,250,0);
    view->setBackgroundColor(0.0,0,0);
    view->addPointCloud<pcl::PointNormal>(union_cloud,v1,"sample1");
    view->spin();


  //  pcl::io::savePCDFile ("bun0-mls.pcd", *mls_points);

  }
  MPI_Finalize();
}
