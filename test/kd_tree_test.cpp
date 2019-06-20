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

#include "../src/kd_tree.hpp"

#include <iostream>
#include <catch.hpp>

using namespace ser94mor::lidar_obstacle_detection;


TEST_CASE("KDTree<2>::Insert", "[kd_tree]")
{
  using point_type = typename KDTree<2>::point_type;
  using vector_type = point_type::vector_type;

  KDTree<2> tree;
  tree.Insert({ 1, vector_type(1.0, 2.0)});
  tree.Insert({ 5, vector_type(2.0, 3.0)});
  tree.Insert({ 2, vector_type(0.5, 5.0)});
  tree.Insert({10, vector_type(2.0, 1.0)});
  tree.Insert({ 3, vector_type(3.0, 4.0)});
  tree.Insert({16, vector_type(2.5, 4.0)});

  REQUIRE(tree.Root()->Point() == point_type{1, vector_type(1.0, 2.0)});
  REQUIRE(tree.Root()->Right()->Point() == point_type{5, vector_type(2.0, 3.0)});
  REQUIRE(tree.Root()->Left()->Point() == point_type{2, vector_type(0.5, 5.0)});
  REQUIRE(tree.Root()->Right()->Left()->Point() == point_type{10, vector_type(2.0, 1.0)});
  REQUIRE(tree.Root()->Right()->Right()->Point() == point_type{3, vector_type(3.0, 4.0)});
  REQUIRE(tree.Root()->Right()->Right()->Left()->Point() == point_type{16, vector_type(2.5, 4.0)});

  REQUIRE(tree.Root()->Left()->Left() == nullptr);
  REQUIRE(tree.Root()->Left()->Right() == nullptr);
  REQUIRE(tree.Root()->Right()->Left()->Left() == nullptr);
  REQUIRE(tree.Root()->Right()->Left()->Right() == nullptr);
  REQUIRE(tree.Root()->Right()->Right()->Left()->Left() == nullptr);
  REQUIRE(tree.Root()->Right()->Right()->Left()->Right() == nullptr);
  REQUIRE(tree.Root()->Right()->Right()->Right() == nullptr);
}

TEST_CASE("KDTree<3>::Insert", "[kd_tree]")
{
  using point_type = typename KDTree<3>::point_type;
  using vector_type = point_type::vector_type;

  KDTree<3> tree;
  tree.Insert({ 1, vector_type(1.0, 2.0, 3.0)});
  tree.Insert({ 5, vector_type(2.0, 3.0, 1.0)});
  tree.Insert({ 2, vector_type(0.5, 5.0, 4.0)});
  tree.Insert({10, vector_type(2.0, 1.0, 5.0)});
  tree.Insert({ 3, vector_type(3.0, 4.0, 6.0)});
  tree.Insert({16, vector_type(2.5, 3.5, 7.0)});
  tree.Insert({ 8, vector_type(2.6, 3.5, 7.0)});

  REQUIRE(tree.Root()->Point() == point_type{1, vector_type(1.0, 2.0, 3.0)});
  REQUIRE(tree.Root()->Right()->Point() == point_type{5, vector_type(2.0, 3.0, 1.0)});
  REQUIRE(tree.Root()->Left()->Point() == point_type{2, vector_type(0.5, 5.0, 4.0)});
  REQUIRE(tree.Root()->Right()->Left()->Point() == point_type{10, vector_type(2.0, 1.0, 5.0)});
  REQUIRE(tree.Root()->Right()->Right()->Point() == point_type{3, vector_type(3.0, 4.0, 6.0)});
  REQUIRE(tree.Root()->Right()->Right()->Right()->Point() == point_type{16, vector_type(2.5, 3.5, 7.0)});
  REQUIRE(tree.Root()->Right()->Right()->Right()->Right()->Point() == point_type{8, vector_type(2.6, 3.5, 7.0)});

  REQUIRE(tree.Root()->Left()->Left() == nullptr);
  REQUIRE(tree.Root()->Left()->Right() == nullptr);
  REQUIRE(tree.Root()->Right()->Left()->Left() == nullptr);
  REQUIRE(tree.Root()->Right()->Left()->Right() == nullptr);
  REQUIRE(tree.Root()->Right()->Right()->Right()->Left() == nullptr);
  REQUIRE(tree.Root()->Right()->Right()->Right()->Right()->Left() == nullptr);
  REQUIRE(tree.Root()->Right()->Right()->Right()->Right()->Right() == nullptr);
}


TEST_CASE("KDTree<2>::Search", "[kd_tree]")
{
  using point_type = typename KDTree<2, uint64_t>::point_type;
  using vector_type = point_type::vector_type;

  KDTree<2, uint64_t> tree;
  tree.Insert({ 1, vector_type(1.0, 2.0)});
  tree.Insert({ 5, vector_type(2.0, 3.0)});
  tree.Insert({ 2, vector_type(0.5, 5.0)});
  tree.Insert({10, vector_type(2.0, 1.0)});
  tree.Insert({ 3, vector_type(3.0, 4.0)});
  tree.Insert({16, vector_type(2.5, 4.0)});

  REQUIRE(tree.Search(vector_type(2.2, 3.5), 1.5) == std::vector<uint64_t>{5, 3, 16});
  REQUIRE(tree.Search(vector_type(2.2, 3.5), 0.5).empty());
  REQUIRE(tree.Search(vector_type(2.2, 3.5), 5.0) == std::vector<uint64_t>{1, 2, 5, 10, 3, 16});
}


TEST_CASE("KDTree<3>::Search", "[kd_tree]")
{
  using point_type = typename KDTree<3, uint64_t>::point_type;
  using vector_type = point_type::vector_type;

  KDTree<3, uint64_t> tree;
  tree.Insert({ 1, vector_type(1.0, 2.0, 3.0)});
  tree.Insert({ 5, vector_type(2.0, 3.0, 1.0)});
  tree.Insert({ 2, vector_type(0.5, 5.0, 4.0)});
  tree.Insert({10, vector_type(2.0, 1.0, 5.0)});
  tree.Insert({ 3, vector_type(3.0, 4.0, 6.0)});
  tree.Insert({16, vector_type(2.5, 3.5, 7.0)});
  tree.Insert({ 8, vector_type(2.6, 3.5, 7.0)});

  REQUIRE(tree.Search(vector_type(2.0, 3.5, 5.8), 1.5) == std::vector<uint64_t>{3, 16, 8});
  REQUIRE(tree.Search(vector_type(2.0, 3.5, 5.8), 0.5).empty());
  REQUIRE(tree.Search(vector_type(2.0, 3.5, 5.8), 5.0) == std::vector<uint64_t>{1, 2, 5, 10, 3, 16, 8});
}
