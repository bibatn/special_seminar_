#ifndef MLS_MPI_H
#define MLS_MPI_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class mls_mpi
{
public:
  mls_mpi();
  void read(std::string & path, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

#endif // MLS_MPI_H
