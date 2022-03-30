#include "mls_mpi.h"

mls_mpi::mls_mpi(int rank, int size):cloud (new pcl::PointCloud<pcl::PointXYZ> ())
{
  rank = rank;
  size = size;
  PCDReader();
}

std::istream& mls_mpi::go_to_line(std::istream& file, unsigned int num){
    file.seekg(std::ios::beg);
    for(int i=0; i < num - 1; ++i){
        file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
}

int
mls_mpi::readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset)
{
  Eigen::Vector4f origin = Eigen::Vector4f::Zero ();
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity ();
  int pcd_version = 0;
  int data_type = 0;
  unsigned int data_idx = 0;

  return readHeader (file_name, cloud, origin, orientation, pcd_version, data_type, data_idx, offset);
}

int
mls_mpi::readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                            int &pcd_version, int &data_type, unsigned int &data_idx, const int offset)
{
  if (file_name.empty() || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }

  // Open file in binary mode to avoid problem of
  // std::getline() corrupting the result of ifstream::tellg()
  std::ifstream fs;
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror (errno));
    fs.close ();
    return (-1);
  }

  // Seek at the given offset
  fs.seekg (offset, std::ios::beg);

  // Delegate parsing to the istream overload.
  int result = readHeader (fs, cloud, origin, orientation, pcd_version, data_type, data_idx);

  // Close file
  fs.close ();

  return result;
}


int
mls_mpi::readHeader (std::istream &fs, pcl::PCLPointCloud2 &cloud,
                            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                            int &pcd_version, int &data_type, unsigned int &data_idx)
{
  // Default values
  data_idx = 0;
  data_type = 0;
  pcd_version = PCD_V6;
  origin      = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;

  std::size_t nr_points = 0;
  std::string line;

  // field_sizes represents the size of one element in a field (e.g., float = 4, char = 1)
  // field_counts represents the number of elements in a field (e.g., x = 1, normal_x = 1, fpfh = 33)
  std::vector<int> field_sizes, field_counts;
  // field_types represents the type of data in a field (e.g., F = float, U = unsigned)
  std::vector<char> field_types;
  std::vector<std::string> st;

  // Read the header and fill it in with wonderful values
  try
  {
    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line.empty())
        continue;

      // Tokenize the line
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());

      std::string line_type;
      sstream >> line_type;

      // Ignore comments
      if (line_type.substr (0, 1) == "#")
        continue;

      // Version numbers are not needed for now, but we are checking to see if they're there
      if (line_type.substr (0, 7) == "VERSION")
        continue;

      // Get the field indices (check for COLUMNS too for backwards compatibility)
      if ( (line_type.substr (0, 6) == "FIELDS") || (line_type.substr (0, 7) == "COLUMNS") )
      {
        int specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        cloud.fields.resize (specified_channel_count);
        for (int i = 0; i < specified_channel_count; ++i)
        {
          std::string col_type = st.at (i + 1);
          cloud.fields[i].name = col_type;
        }

        // Default the sizes and the types of each field to float32 to avoid crashes while using older PCD files
        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i, offset += 4)
        {
          cloud.fields[i].offset   = offset;
          cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
          cloud.fields[i].count    = 1;
        }
        cloud.point_step = offset;
        continue;
      }

      // Get the field sizes
      if (line_type.substr (0, 4) == "SIZE")
      {
        int specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <SIZE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_sizes.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          int col_type ;
          sstream >> col_type;
          cloud.fields[i].offset = offset;                // estimate and save the data offsets
          offset += col_type;
          field_sizes[i] = col_type;                      // save a temporary copy
        }
        cloud.point_step = offset;
        //if (cloud.width != 0)
          //cloud.row_step   = cloud.point_step * cloud.width;
        continue;
      }

      // Get the field types
      if (line_type.substr (0, 4) == "TYPE")
      {
        if (field_sizes.empty ())
          throw "TYPE of FIELDS specified before SIZE in header!";

        int specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <TYPE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_types.resize (specified_channel_count);

        for (int i = 0; i < specified_channel_count; ++i)
        {
          field_types[i] = st.at (i + 1).c_str ()[0];
          cloud.fields[i].datatype = static_cast<std::uint8_t> (pcl::getFieldType (field_sizes[i], field_types[i]));
        }
        continue;
      }

      // Get the field counts
      if (line_type.substr (0, 5) == "COUNT")
      {
        if (field_sizes.empty () || field_types.empty ())
          throw "COUNT of FIELDS specified before SIZE or TYPE in header!";

        int specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <COUNT> differs than the number of elements in <FIELDS>!";

        field_counts.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          cloud.fields[i].offset = offset;
          int col_count;
          sstream >> col_count;
          if (col_count < 1)
            throw "Invalid COUNT value specified.";
          cloud.fields[i].count = col_count;
          offset += col_count * field_sizes[i];
        }
        // Adjust the offset for count (number of elements)
        cloud.point_step = offset;
        continue;
      }

      // Get the width of the data (organized point cloud dataset)
      if (line_type.substr (0, 5) == "WIDTH")
      {
        sstream >> cloud.width;

        cloud.width = cloud.width/size; //m

        if (sstream.fail ())
          throw "Invalid WIDTH value specified.";
        if (cloud.point_step != 0)
          cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
        continue;
      }

      // Get the height of the data (organized point cloud dataset)
      if (line_type.substr (0, 6) == "HEIGHT")
      {
        sstream >> cloud.height;
        continue;
      }

      // Get the acquisition viewpoint
      if (line_type.substr (0, 9) == "VIEWPOINT")
      {
        pcd_version = PCD_V7;
        if (st.size () < 8)
          throw "Not enough number of elements in <VIEWPOINT>! Need 7 values (tx ty tz qw qx qy qz).";

        float x, y, z, w;
        sstream >> x >> y >> z ;
        origin      = Eigen::Vector4f (x, y, z, 0.0f);
        sstream >> w >> x >> y >> z;
        orientation = Eigen::Quaternionf (w, x, y, z);
        continue;
      }

      // Get the number of points
      if (line_type.substr (0, 6) == "POINTS")
      {
        if (!cloud.point_step)
          throw "Number of POINTS specified before COUNT in header!";
        sstream >> nr_points;

        nr_points = nr_points / size;

        // Need to allocate: N * point_step
        cloud.data.resize (nr_points * cloud.point_step);
        continue;
      }

      // Read the header + comments line by line until we get to <DATA>
      if (line_type.substr (0, 4) == "DATA")
      {
        data_idx = static_cast<int> (fs.tellg ());
        if (st.at (1).substr (0, 17) == "binary_compressed")
         data_type = 2;
        else
          if (st.at (1).substr (0, 6) == "binary")
            data_type = 1;
        continue;
      }
      break;
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] %s\n", exception);
    return (-1);
  }

  // Exit early: if no points have been given, there's no sense to read or check anything anymore
  if (nr_points == 0)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] No points to read\n");
    return (-1);
  }

  // Compatibility with older PCD file versions
  if (cloud.width == 0 && cloud.height == 0)
  {
    cloud.width  = nr_points;
    cloud.height = 1;
    cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
  }
  //assert (cloud.row_step != 0);       // If row_step = 0, either point_step was not set or width is 0

  // if both height/width are not given, assume an unorganized dataset
  if (cloud.height == 0)
  {
    cloud.height = 1;
    PCL_WARN ("[pcl::PCDReader::readHeader] no HEIGHT given, setting to 1 (unorganized).\n");
    if (cloud.width == 0)
      cloud.width  = nr_points;
  }
  else
  {
    if (cloud.width == 0 && nr_points != 0)
    {
      PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT given (%d) but no WIDTH!\n", cloud.height);
      return (-1);
    }
  }

  if (cloud.width * cloud.height != nr_points)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT (%d) x WIDTH (%d) != number of points (%d)\n", cloud.height, cloud.width, nr_points);
    return (-1);
  }

  return (0);
}

int
mls_mpi::read (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset)
{
  int pcd_version;
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  // Load the data
  int res = read (file_name, cloud, origin, orientation, pcd_version, offset);

  if (res < 0)
    return (res);

  return (0);
}

int
mls_mpi::read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &pcd_version,
                      const int offset)
{
  pcl::console::TicToc tt;
  tt.tic ();

  if (file_name.empty() || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[pcl::PCDReader::read] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }

  int data_type;
  unsigned int data_idx;

  int res = readHeader (file_name, cloud, origin, orientation, pcd_version, data_type, data_idx, offset);

  if (res < 0)
    return (res);

  // if ascii
  if (data_type == 0)
  {
    // Re-open the file (readHeader closes it)
    std::ifstream fs;
    fs.open (file_name.c_str ());
    if (!fs.is_open () || fs.fail ())
    {
      PCL_ERROR ("[pcl::PCDReader::read] Could not open file %s.\n", file_name.c_str ());
      return (-1);
    }

    fs.seekg (data_idx + offset);

    // Read the rest of the file
    res = readBodyASCII (fs, cloud, pcd_version);

    // Close file
    fs.close ();
  }
  else
  /// ---[ Binary mode only
  /// We must re-open the file and read with mmap () for binary
  {
    // Open for reading
    int fd = pcl::io::raw_open (file_name.c_str (), O_RDONLY);
    if (fd == -1)
    {
      PCL_ERROR ("[pcl::PCDReader::read] Failure to open file %s\n", file_name.c_str () );
      return (-1);
    }

    // Infer file size
    const std::size_t file_size = pcl::io::raw_lseek (fd, 0, SEEK_END);
    pcl::io::raw_lseek (fd, 0, SEEK_SET);

    std::size_t mmap_size = offset + data_idx;   // ...because we mmap from the start of the file.
    if (data_type == 2)
    {
      // Seek to real start of data.
      long result = pcl::io::raw_lseek (fd, offset + data_idx, SEEK_SET);
      if (result < 0)
      {
        pcl::io::raw_close (fd);
        PCL_ERROR ("[pcl::PCDReader::read] lseek errno: %d strerror: %s\n", errno, strerror (errno));
        PCL_ERROR ("[pcl::PCDReader::read] Error during lseek ()!\n");
        return (-1);
      }

      // Read compressed size to compute how much must be mapped
      unsigned int compressed_size = 0;
      ssize_t num_read = pcl::io::raw_read (fd, &compressed_size, 4);
      if (num_read < 0)
      {
        pcl::io::raw_close (fd);
        PCL_ERROR ("[pcl::PCDReader::read] read errno: %d strerror: %s\n", errno, strerror (errno));
        PCL_ERROR ("[pcl::PCDReader::read] Error during read()!\n");
        return (-1);
      }
      mmap_size += compressed_size;
      // Add the 8 bytes used to store the compressed and uncompressed size
      mmap_size += 8;

      // Reset position
      pcl::io::raw_lseek (fd, 0, SEEK_SET);
    }
    else
    {
      mmap_size += cloud.data.size ();
    }

    if (mmap_size > file_size)
    {
      pcl::io::raw_close (fd);
      PCL_ERROR ("[pcl::PCDReader::read] Corrupted PCD file. The file is smaller than expected!\n");
      return (-1);
    }

    // Prepare the map
#ifdef _WIN32
    // As we don't know the real size of data (compressed or not),
    // we set dwMaximumSizeHigh = dwMaximumSizeLow = 0 so as to map the whole file
    HANDLE fm = CreateFileMapping ((HANDLE) _get_osfhandle (fd), NULL, PAGE_READONLY, 0, 0, NULL);
    // As we don't know the real size of data (compressed or not),
    // we set dwNumberOfBytesToMap = 0 so as to map the whole file
    unsigned char *map = static_cast<unsigned char*> (MapViewOfFile (fm, FILE_MAP_READ, 0, 0, 0));
    if (map == NULL)
    {
      CloseHandle (fm);
      io::raw_close (fd);
      PCL_ERROR ("[pcl::PCDReader::read] Error mapping view of file, %s\n", file_name.c_str ());
      return (-1);
    }
#else
    unsigned char *map = static_cast<unsigned char*> (::mmap (nullptr, mmap_size, PROT_READ, MAP_SHARED, fd, 0));
    if (map == reinterpret_cast<unsigned char*> (-1))    // MAP_FAILED
    {
      pcl::io::raw_close (fd);
      PCL_ERROR ("[pcl::PCDReader::read] Error preparing mmap for binary PCD file.\n");
      return (-1);
    }
#endif

    res = readBodyBinary (map, cloud, pcd_version, data_type == 2, offset + data_idx);

    // Unmap the pages of memory
#ifdef _WIN32
    UnmapViewOfFile (map);
    CloseHandle (fm);
#else
    if (::munmap (map, mmap_size) == -1)
    {
      pcl::io::raw_close (fd);
      PCL_ERROR ("[pcl::PCDReader::read] Munmap failure\n");
      return (-1);
    }
#endif
    pcl::io::raw_close (fd);
  }
  double total_time = tt.toc ();
  PCL_DEBUG ("[pcl::PCDReader::read] Loaded %s as a %s cloud in %g ms with %d points. Available dimensions: %s.\n",
             file_name.c_str (), cloud.is_dense ? "dense" : "non-dense", total_time,
             cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());
  return res;
}

int
mls_mpi::readBodyASCII (std::istream &fs, pcl::PCLPointCloud2 &cloud, int /*pcd_version*/)
{
  go_to_line(fs, cloud.height*cloud.width*rank);
  // Get the number of points the cloud should have
  unsigned int nr_points = cloud.width * cloud.height;
  // The number of elements each line/point should have
  const unsigned int elems_per_line = std::accumulate (cloud.fields.cbegin (), cloud.fields.cend (), 0u,
                                                       [](const auto& i, const auto& field){ return (i + field.count); });
  PCL_DEBUG ("[pcl::PCDReader::readBodyASCII] Will check that each line in the PCD file has %u elements.\n", elems_per_line);

  // Setting the is_dense property to true by default
  cloud.is_dense = true;

  unsigned int idx = 0;
  std::string line;
  std::vector<std::string> st;
  std::istringstream is;
  is.imbue (std::locale::classic ());

  try
  {
    while (idx < nr_points && !fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line.empty())
        continue;

      // Tokenize the line
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      if (st.size () != elems_per_line) // If this is not checked, an exception might occur while accessing st
      {
        PCL_WARN ("[pcl::PCDReader::readBodyASCII] Possibly malformed PCD file: point number %u has %zu elements, but should have %u\n",
                  idx+1, st.size (), elems_per_line);
        ++idx; // Skip this line/point, but read all others
        continue;
      }

      if (idx >= nr_points)
      {
        PCL_WARN ("[pcl::PCDReader::read] input has more points (%d) than advertised (%d)!\n", idx, nr_points);
        break;
      }

      std::size_t total = 0;
      // Copy data
      for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
      {
        // Ignore invalid padded dimensions that are inherited from binary data
        if (cloud.fields[d].name == "_")
        {
          total += cloud.fields[d].count; // jump over this many elements in the string token
          continue;
        }
        for (pcl::uindex_t c = 0; c < cloud.fields[d].count; ++c)
        {
          switch (cloud.fields[d].datatype)
          {
            case pcl::PCLPointField::INT8:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT8>::type> (
                  st.at (total + c), cloud, idx, d, c, is);
              break;
            }
            case pcl::PCLPointField::UINT8:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT8>::type> (
                  st.at (total + c), cloud, idx, d, c, is);
              break;
            }
            case pcl::PCLPointField::INT16:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT16>::type> (
                  st.at (total + c), cloud, idx, d, c, is);
              break;
            }
            case pcl::PCLPointField::UINT16:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT16>::type> (
                  st.at (total + c), cloud, idx, d, c, is);
              break;
            }
            case pcl::PCLPointField::INT32:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT32>::type> (
                  st.at (total + c), cloud, idx, d, c, is);
              break;
            }
            case pcl::PCLPointField::UINT32:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT32>::type> (
                  st.at (total + c), cloud, idx, d, c, is);
              break;
            }
            case pcl::PCLPointField::FLOAT32:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type> (
                  st.at (total + c), cloud, idx, d, c, is);
              break;
            }
            case pcl::PCLPointField::FLOAT64:
            {
              pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type> (
                  st.at (total + c), cloud, idx, d, c, is);
              break;
            }
            default:
              PCL_WARN ("[pcl::PCDReader::read] Incorrect field data type specified (%d)!\n",cloud.fields[d].datatype);
              break;
          }
        }
        total += cloud.fields[d].count; // jump over this many elements in the string token
      }
      idx++;
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::PCDReader::read] %s\n", exception);
    return (-1);
  }

  if (idx != nr_points)
  {
    PCL_ERROR ("[pcl::PCDReader::read] Number of points read (%d) is different than expected (%d)\n", idx, nr_points);
    return (-1);
  }

  return (0);
}

