#pragma once
#include <boost/filesystem.hpp>
#include <cmath>

/**
 * \brief   Return the filenames of all files that have the specified extension
 *          in the specified directory and all subdirectories.
 */
inline std::vector<std::string> get_all_filenames(boost::filesystem::path const & root, std::string const & ext = "")
{
  std::vector<std::string> paths;

  if(boost::filesystem::exists(root) && boost::filesystem::is_directory(root))
  {
    for(auto const & entry : boost::filesystem::recursive_directory_iterator(root))
    {
      if(boost::filesystem::is_regular_file(entry) && (ext.empty() || entry.path().extension() == ext))
        paths.emplace_back(entry.path().filename().c_str());
    }
  }

  return paths;
}

inline double truncate(double value, double precision = 2)
{
  return (floor((value * pow(10, precision) + 0.5)) / pow(10, precision));
}

template<typename VectorT>
VectorT truncate(const VectorT & value, double precision = 2)
{
  VectorT r;
  for(unsigned i = 0; i < value.size(); ++i)
  {
    r[i] = truncate(value[i], precision);
  }
  return r;
}
