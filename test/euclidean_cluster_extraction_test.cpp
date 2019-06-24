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

#include "../src/EuclideanClusterExtraction.hpp"

#include <iostream>
#include <catch.hpp>

using namespace ser94mor::lidar_obstacle_detection;

TEST_CASE("EuclideanClusterExtraction<3>::Extract", "[EuclideanClusterExtraction]")
{
  using euclideanclusterextraction_type = EuclideanClusterExtraction<3, uint64_t>;
  using point_type = euclideanclusterextraction_type::point_type;
  using vector_type = point_type::vector_type;
  std::vector<point_type> points{
      {0, vector_type(1.0, 2.0, 3.0)},
      {1, vector_type(1.5, 1.5, 2.5)},
      {2, vector_type(2.0, 1.0, 2.0)},
      {3, vector_type(6, 8, 7)},
      {4, vector_type(7, 8, 6)},
      {5, vector_type(8, 7, 6)},
      {6, vector_type(10, 11, 10)},
      {7, vector_type(8, 15, 13)},
      {8, vector_type(9, 14, 12)},
      {9, vector_type(7.5, 16, 11)},
  };

  auto result{euclideanclusterextraction_type::Extract(points, 1.0)};

  for (auto& cluster : result)
  {
    for (auto id : cluster)
    {
      std::cout << id << ' ';
    }
    std::cout << std::endl;
  }
}
