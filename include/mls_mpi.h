#ifndef MLS_MPI_H
#define MLS_MPI_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <istream>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/common/io.h>
#include <pcl/types.h>
//#include <pcl/io/file_io.h>
#include "boost/filesystem.hpp"
#include <boost/algorithm/string.hpp>

class mls_mpi: public pcl::PCDReader
{
public:
  mls_mpi(int rank, int size);
  void read(std::string & path, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  std::istream& go_to_line(std::istream& file, unsigned int num);
  int readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset);
  int readHeader (std::istream &fs, pcl::PCLPointCloud2 &cloud,
                              Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                              int &pcd_version, int &data_type, unsigned int &data_idx);
  int readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                              Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                              int &pcd_version, int &data_type, unsigned int &data_idx, const int offset);

  int read (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset);

  int read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                        Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version,
                        const int offset);
  int readBodyASCII (std::istream &fs, pcl::PCLPointCloud2 &cloud, int /*pcd_version*/);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

private:
  int rank, size;
};

#endif // MLS_MPI_H

