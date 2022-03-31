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

void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color) {

    int N = pc.points.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int i = 0; i < N; ++i) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}



int
main (int argc, char ** argv)
{
  int size, rank;
//  MPI_Init(&argc, &argv);
//  MPI_Comm_size(MPI_COMM_WORLD, &size);
//  MPI_Comm_rank(MPI_COMM_WORLD, &rank);

  mls_mpi cloud_part(0, 4);
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
  std::cout << "Длина массива: " << mls.indices_->size() <<std::endl;
  now = std::chrono::high_resolution_clock::now().time_since_epoch();
  uint64_t T2 = now.count();
  std::cout << "TIME: " << T2-T1 << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

  colorize(*cloud_part.cloud, *out_colored, {0, 255, 0});

  pcl::visualization::PCLVisualizer viewer1("Raw");
  viewer1.addPointCloud<pcl::PointXYZRGB>(out_colored, "filtered_green");

  while (!viewer1.wasStopped()) {
      viewer1.spinOnce();
  }

  std::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("test"));
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> v1(mls_points,0,250,0);
  view->setBackgroundColor(0.0,0,0);
  view->addPointCloud<pcl::PointNormal>(mls_points,v1,"sample1");
//  view->addPointCloudNormals<pcl::PointNormal>(mls_points,50,20,"normal1");
//  view->addCoordinateSystem(1.0);
  view->spin();


  pcl::io::savePCDFile ("bun0-mls.pcd", *mls_points);
//  MPI_Finalize();
}



//template <typename PointInT, typename PointOutT> void
//pcl::MovingLeastSquares<PointInT, PointOutT>::process (PointCloudOut &output)
//{
//  // Reset or initialize the collection of indices
//  corresponding_input_indices_.reset (new PointIndices);

//  // Check if normals have to be computed/saved
//  if (compute_normals_)
//  {
//    normals_.reset (new NormalCloud);
//    // Copy the header
//    normals_->header = input_->header;
//    // Clear the fields in case the method exits before computation
//    normals_->width = normals_->height = 0;
//    normals_->points.clear ();
//  }

//  // Copy the header
//  output.header = input_->header;
//  output.width = output.height = 0;
//  output.clear ();

//  if (search_radius_ <= 0 || sqr_gauss_param_ <= 0)
//  {
//    PCL_ERROR ("[pcl::%s::process] Invalid search radius (%f) or Gaussian parameter (%f)!\n", getClassName ().c_str (), search_radius_, sqr_gauss_param_);
//    return;
//  }

//  // Check if distinct_cloud_ was set
//  if (upsample_method_ == DISTINCT_CLOUD && !distinct_cloud_)
//  {
//    PCL_ERROR ("[pcl::%s::process] Upsample method was set to DISTINCT_CLOUD, but no distinct cloud was specified.\n", getClassName ().c_str ());
//    return;
//  }

//  if (!initCompute ())
//    return;

//  // Initialize the spatial locator
//  if (!tree_)
//  {
//    KdTreePtr tree;
//    if (input_->isOrganized ())
//      tree.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
//    else
//      tree.reset (new pcl::search::KdTree<PointInT> (false));
//    setSearchMethod (tree);
//  }

//  // Send the surface dataset to the spatial locator
//  tree_->setInputCloud (input_);

//  switch (upsample_method_)
//  {
//    // Initialize random number generator if necessary
//    case (RANDOM_UNIFORM_DENSITY):
//    {
//      std::random_device rd;
//      rng_.seed (rd());
//      const double tmp = search_radius_ / 2.0;
//      rng_uniform_distribution_.reset (new std::uniform_real_distribution<> (-tmp, tmp));

//      break;
//    }
//    case (VOXEL_GRID_DILATION):
//    case (DISTINCT_CLOUD):
//    {
//      if (!cache_mls_results_)
//        PCL_WARN ("The cache mls results is forced when using upsampling method VOXEL_GRID_DILATION or DISTINCT_CLOUD.\n");

//      cache_mls_results_ = true;
//      break;
//    }
//    default:
//      break;
//  }

//  if (cache_mls_results_)
//  {
//    mls_results_.resize (input_->size ());
//  }
//  else
//  {
//    mls_results_.resize (1); // Need to have a reference to a single dummy result.
//  }

//  // Perform the actual surface reconstruction
//  performProcessing (output);

//  if (compute_normals_)
//  {
//    normals_->height = 1;
//    normals_->width = normals_->size ();

//    for (std::size_t i = 0; i < output.size (); ++i)
//    {
//      using FieldList = typename pcl::traits::fieldList<PointOutT>::type;
//      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output[i], "normal_x", (*normals_)[i].normal_x));
//      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output[i], "normal_y", (*normals_)[i].normal_y));
//      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output[i], "normal_z", (*normals_)[i].normal_z));
//      pcl::for_each_type<FieldList> (SetIfFieldExists<PointOutT, float> (output[i], "curvature", (*normals_)[i].curvature));
//    }

//  }

//  // Set proper widths and heights for the clouds
//  output.height = 1;
//  output.width = output.size ();

//  deinitCompute ();
//}


