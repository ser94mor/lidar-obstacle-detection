/**
 * Copyright (C) 2019  Sergey Morozov <sergey@morozov.ch>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "render/render.h"
#include "PointCloudProcessor.hpp"

#include <iostream>
#include <sstream>
#include <string>


#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace ser94mor::lidar_obstacle_detection;

using PointProcessorPCL = PointCloudProcessor<pcl::PointXYZI, true>;
using PointProcessorNOPCL = PointCloudProcessor<pcl::PointXYZI, false>;


void init_camera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

  viewer->setBackgroundColor (0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch(setAngle)
  {
    case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
    case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
    case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
    case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if(setAngle!=FPS)
    viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, PointCloudProcessor<pcl::PointXYZI, false> pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  auto filterCloud = pointProcessorI.FilterCloud(inputCloud, 0.5f, Eigen::Vector4f (-10, -6, -3, 1), Eigen::Vector4f ( 20, 6, 3, 1));
  //renderPointCloud(viewer,filterCloud,"filterCloud");

  auto segmentCloud = pointProcessorI.SegmentPlane(filterCloud, 100, 0.2);
  //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first, 0.55, 10, 1000);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};



  for(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : cloudClusters)
  {
    std::cout << "cluster size ";
    pointProcessorI.NumberOfPointsIn(cluster);
    renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);
    Box box = pointProcessorI.BoundingBox(cluster);
    renderBox(viewer,box,clusterId);
    ++clusterId;
  }


}

int main(int argc, char* argv[])
{
  // handle input arguments; notice that the validity of the arguments themselves is not checked, only their number
  if (argc != 1 and argc != 3)
  {
    std::cerr << "Wrong number of command line arguments provided.\n"
              << "Should be either none or 2, where the first argument\n"
              << "is a path to the directory containing *.pcd files and\n"
              << "the second argument is boolean (true/false) indicating\n"
              << "whether or not to use PLC implementations of KD-Tree,\n"
              << "RANSAC algorithm, and Euclidean clustering algorithm."
              << std::endl;
    return EXIT_FAILURE;
  }
  std::string pcd_data_dir = (argc == 3) ? argv[1] : "../data/pcd/data_1";
  bool use_pcl;
  std::istringstream((argc == 3) ? argv[2] : "false") >> std::boolalpha >> use_pcl;

  std::cout << "PCD data directory: " << pcd_data_dir << '\n'
            << "Use PCL algorithms: " << std::boolalpha << use_pcl
            << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  CameraAngle setAngle = FPS;
  init_camera(setAngle, viewer);
  auto pointProcessorI = PointCloudProcessor<pcl::PointXYZI, false>();
  std::vector<boost::filesystem::path> stream = pointProcessorI.StreamPcd(pcd_data_dir);
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
  //cityBlock(viewer, pointProcessorI, inputCloudI);

  while (!viewer->wasStopped ())
  {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI.LoadPcd(streamIterator->string());
    cityBlock(viewer, pointProcessorI, inputCloudI);
    //renderPointCloud(viewer,inputCloudI,"planeCloud");

    streamIterator++;
    if(streamIterator == stream.end())
      streamIterator = stream.begin();

    viewer->spinOnce ();
  }


  return EXIT_SUCCESS;
}
