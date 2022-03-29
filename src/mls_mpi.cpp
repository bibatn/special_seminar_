#include "mls_mpi.h"

mls_mpi::mls_mpi():cloud (new pcl::PointCloud<pcl::PointXYZ> ())
{

}

std::istream& mls_mpi::go_to_line(std::istream& file, unsigned int num){
    file.seekg(std::ios::beg);
    for(int i=0; i < num - 1; ++i){
        file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
}
