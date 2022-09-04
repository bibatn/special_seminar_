#ifndef MLS_MPI_H
#define MLS_MPI_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <istream>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/common/io.h>
//#include <pcl/types.h>
#include <mls.h>
#include "boost/filesystem.hpp"
#include <boost/algorithm/string.hpp>

class mls_mpi: public pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>
{
public:
  mls_mpi(int rank, int size);
//  void read(std::string & path, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  inline int
  loadPCDFile (const std::string &file_name, pcl::PCLPointCloud2 &cloud)
  {
    return mls_mpi::read (file_name, cloud);
  }
  std::istream& go_to_line(std::istream& file, unsigned int num);
  int readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset = 0);
  int readHeader (std::istream &fs, pcl::PCLPointCloud2 &cloud,
                              Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                              int &pcd_version, int &data_type, unsigned int &data_idx);
  int readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                              Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                              int &pcd_version, int &data_type, unsigned int &data_idx, const int offset = 0);

  int read (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset = 0);

  int read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                        Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version,
                        const int offset = 0);
  int readBodyASCII (std::istream &fs, pcl::PCLPointCloud2 &cloud, int /*pcd_version*/);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  enum
  {
    PCD_V6 = 0,
    PCD_V7 = 1
  };

private:
  int rank_, size_;
};

#endif // MLS_MPI_H

