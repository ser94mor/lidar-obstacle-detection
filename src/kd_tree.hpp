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

#ifndef LIDAR_OBSTACLE_DETECTION_KD_TREE_HPP
#define LIDAR_OBSTACLE_DETECTION_KD_TREE_HPP

#include <cstdlib>
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace ser94mor::lidar_obstacle_detection
{

  template<size_t dims, typename id_type = uint64_t>
  class KDTree
  {
  public:

    struct KDPoint
    {
      using vector_type = Eigen::Matrix<double_t, dims, 1>;

      id_type id;
      vector_type data;

      bool IsInHypersphere(const vector_type& center, const double_t radius) const
      {
        auto diff = data - center;
        double_t dist = std::sqrt(diff.transpose() * diff);

        return dist <= radius;
      }

      bool IsGreaterThanOrEqualTo(const vector_type& point, size_t depth, const double_t distance_tolerance = 0.0) const
      {
        auto ind = depth % dims;
        return data(ind) >= point(ind)-distance_tolerance;
      }

      bool IsLessThanOrEqualTo(const vector_type& point, size_t depth, const double_t distance_tolerance = 0.0) const
      {
        auto ind = depth % dims;
        return data(ind) <= point(ind)+distance_tolerance;
      }

      bool operator==(const KDPoint& rhs) const
      {
        return id == rhs.id && data.isApprox(rhs.data);
      }

      bool operator!=(const KDPoint& rhs) const
      {
        return !(rhs == *this);
      }

    };

    using point_type = KDPoint;
    using vector_type = typename point_type::vector_type;

  private:
    class KDNode
    {
    private:
      KDPoint kd_point_;
      std::unique_ptr<KDNode> children_[2];

    public:
      explicit KDNode(const KDPoint& kd_point)
          : kd_point_{kd_point}, children_{nullptr, nullptr}
      {

      }

      std::unique_ptr<KDNode>* SubtreeFor(const KDPoint& point, const size_t depth)
      {
        size_t dim_index = depth % dims;
        size_t child_index = not (point.data(dim_index) < kd_point_.data(dim_index));
        return &(children_[child_index]);
      }

      const std::unique_ptr<KDNode>& Left() const
      {
        return children_[0];
      }

      const std::unique_ptr<KDNode>& Right() const
      {
        return children_[1];
      }

      const KDPoint& Point() const
      {
        return kd_point_;
      }
    };


    using node_type = KDNode;

  public:

    KDTree() : root_{nullptr}
    {

    }

    void Insert(const point_type& kd_point)
    {

      size_t cur_depth = 0;

      for (auto* cur_node = &root_; ; ++cur_depth)
      {
        if (*cur_node == nullptr)
        {
          *cur_node = std::make_unique<node_type>(kd_point);
          break;
        }

        cur_node = (*cur_node)->SubtreeFor(kd_point, cur_depth);
      }
    }

    std::vector<id_type> Search(const vector_type& target_point, const double_t distance_tolerance) const
    {
      std::vector<id_type> ids;

      SearchInternal(ids, root_, 0, target_point, distance_tolerance);

      return std::move(ids);
    }

    const std::unique_ptr<node_type>& Root() const
    {
      return root_;
    }

  private:
    std::unique_ptr<node_type> root_;

    void SearchInternal(std::vector<id_type>& ids,
                        const std::unique_ptr<node_type>& node,
                        const size_t depth,
                        const vector_type& target_point,
                        const double_t distance_tolerance) const
    {
      if (node == nullptr)
        return;

      if (node->Point().IsInHypersphere(target_point, distance_tolerance))
        ids.push_back(node->Point().id);

      if (node->Point().IsGreaterThanOrEqualTo(target_point, depth, distance_tolerance))
        SearchInternal(ids, node->Left(), depth+1, target_point, distance_tolerance);

      if (node->Point().IsLessThanOrEqualTo(target_point, depth, distance_tolerance))
        SearchInternal(ids, node->Right(), depth+1, target_point, distance_tolerance);
    }
  };
}

#endif //LIDAR_OBSTACLE_DETECTION_KD_TREE_HPP
