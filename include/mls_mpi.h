#ifndef MLS_MPI_H
#define MLS_MPI_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <istream>
#include <pcl/io/pcd_io.h>

class mls_mpi: public pcl::PCDReader
{
public:
  mls_mpi(int rank, int size);
  void read(std::string & path, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  std::istream& go_to_line(std::istream& file, unsigned int num);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  int rank, size;
};

#endif // MLS_MPI_H

