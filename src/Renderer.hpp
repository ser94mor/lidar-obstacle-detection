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

//
// The original author of the rendering code is Aaron Brown (https://github.com/awbrown90).
// His code has been slightly modified to make it more structured.
//

#ifndef LIDAR_OBSTACLE_DETECTION_RENDERER_HPP
#define LIDAR_OBSTACLE_DETECTION_RENDERER_HPP


#include "Box.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>
#include <string>


namespace ser94mor::lidar_obstacle_detection
{

  struct Color
  {

    float r, g, b;

    Color(float setR, float setG, float setB)
        : r(setR), g(setG), b(setB)
    {}
  };

  enum class CameraAngle
  {
    XY, TopDown, Side, FPS
  };

  class Renderer
  {
  private:
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    unsigned long long rays_counter_;

  public:
    Renderer();

    void RenderHighway();

    void RenderRays(const Eigen::Vector3f& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    void ClearRays();

    void ClearViewer();

    void RenderPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          const std::string& name,
                          const Color& color = Color(1,1,1));

    void RenderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                          const std::string& name,
                          const Color& color = Color(-1,-1,-1));

    void RenderBox(const Box& box, int id, const Color& color = Color(1,0,0), float opacity = 1.0);

    void RenderBox(const BoxQ& box, int id, const Color& color = Color(1,0,0), float opacity = 1.0);

    void InitCamera(CameraAngle view_angle);

    bool WasViewerStopped() const;

    void SpinViewerOnce() const;

  };
}

#endif //LIDAR_OBSTACLE_DETECTION_RENDERER_HPP
