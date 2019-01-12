
#ifndef GRAPHNAVIGATION_GRAPH_NAVIGRAPH_H
#define GRAPHNAVIGATION_GRAPH_NAVIGRAPH_H

#include <vector>
#include <set>
#include <cstdint>
#include <cassert>
#include "src/graph/base_graph.h"

#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>

namespace GraphNavigation
{
namespace Graph
{

template <typename PositionT, typename CostT>
class NaviGraph : public BaseGraph<PositionT, CostT>
{
  public:

    NaviGraph(std::function<size_t(PositionT)> hashT)
        : BaseGraph<PositionT, CostT>(hashT),
          kdtree_(flann::KDTreeSingleIndexParams()),
          kdtree_init_(false)
    {
    }

    virtual bool FindPath(uint start, uint goal, std::vector<uint> &path)
    {
        std::cout << "this is a virtual function!\n";
        return false;
    }

    virtual void AddNode(const PositionT p)
    {
        this->BaseGraph<PositionT, CostT>::AddNode(p);
    }

    void initKDTree()
    {
        auto nodes = this->positions();
        assert(nodes.size() > 0); //
        int dim = nodes[0].rows();
        double targetData[nodes.size() * dim];
        for (uint i = 0; i < nodes.size(); ++i)
        {
            for (int j = 0; j < dim; ++j)
                targetData[i * dim + j] = nodes[i](j, 0);
        }
        flann::Matrix<double> dataset(targetData, nodes.size(), dim);
        kdtree_ = flann::Index<flann::L2_Simple<double>>(
            dataset,
            flann::KDTreeSingleIndexParams());
        kdtree_.buildIndex();
        kdtree_init_ = true;
    }

    void FindNearest(const PositionT &point, double *distanceOut = nullptr)
    {

            // k-NN search (O(log(N)))
            flann::Matrix<double> query = convertEigen2Flann(point);
            
            std::vector<int> i(query.rows);
            flann::Matrix<int> indices(i.data(), query.rows, 1);
            std::vector<double> d(query.rows);
            flann::Matrix<double> dists(d.data(), query.rows, 1);

            kdtree_.knnSearch(query, indices, dists, 1, flann::SearchParams());

            PositionT nearest;
            nearest = (PositionT)kdtree_.getPoint(indices[0][0]);
            printf("indx %d\n",indices[0][0]);
            std::cout<<this->GetIndex(nearest)<<std::endl;
            if (distanceOut)
            {
                *distanceOut = (point - nearest).norm();
            }

    }

  private:
    flann::Matrix<double> convertEigen2Flann(const PositionT &mat)
    {
        flann::Matrix<double> out(new double[mat.rows() * mat.cols()],
                                  mat.cols(), mat.rows());
        for (int i = 0; i < mat.cols(); i++)
        {
            for (int j = 0; j < mat.rows(); j++)
            {
                out[i][j] = mat(j, i);
            }
        }
        return out;
    }

  private:
    flann::Index<flann::L2_Simple<double>> kdtree_;
    bool kdtree_init_;
};

} // namespace Graph
} // namespace GraphNavigation
#endif //GRAPHNAVIGATION_GRAPH_BASHGRAPH_H