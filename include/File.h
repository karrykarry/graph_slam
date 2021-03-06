#ifndef FILE_H
#define FILE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <bits/stdc++.h>
#include <sys/stat.h>
#include <dirent.h>

#include <iostream>
#include <fstream>
#include <tf/tf.h>

struct ID{
    int id;
    tf::Transform transform;
};

namespace GRAPH_SLAM
{

class File{
    public:
        File();

        std::vector<std::string> split(std::string& input, char delimiter);
        int file_count_boost(const boost::filesystem::path& root);

        template<typename T_p>
        bool loadCloud(typename pcl::PointCloud<T_p>::Ptr&, std::string);
        
        template<typename T_p>
        bool saveCloud(typename pcl::PointCloud<T_p>::Ptr&, std::string);

        bool loadTF(std::vector< tf::Transform >&, std::string);
        bool loadTF(std::vector< ID >&, std::string);

        void search_dir(std::string, std::vector<std::string>&);

     
    private:
};

}

#endif
