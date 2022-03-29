#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <vector>
#include <iterator>
#include <chrono>
#include <pico_tree/eigen.hpp>
#include <pico_tree/kd_tree.hpp>

extern "C"{

  struct closest_alignment{
    Eigen::Matrix3Xf s_vals;
    Eigen::Matrix3Xf t_vals;
    std::vector<int> dists;
    std::vector<int> close_enough_inds;
    closest_alignment(Eigen::Matrix3Xf s, Eigen::Matrix3Xf t, std::vector<int> d, std::vector<int> c){
      s_vals=s;
      t_vals=t;
      dists=d;
      close_enough_inds=c;
    }
  };

  closest_alignment get_closest_alignment(Eigen::Matrix3Xf aligned_sources, Eigen::Matrix3Xf target, pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xf>>* target_tree, float max_distance_threshold){
    pico_tree::Neighbor<int, Eigen::Matrix3Xf::Scalar> nn;
    std::vector<int> indices;
    std::vector<int> dists; 
    for(int i=0; i< aligned_sources.cols(); i++){
      target_tree->SearchNn(aligned_sources.col(i), &nn);  
      if(nn.distance<max_distance_threshold){
        indices.push_back(nn.index);
        dists.push_back(nn.distance);
      };
    };
    return closest_alignment(aligned_sources(Eigen::placeholders::all,indices), target(Eigen::placeholders::all, indices), dists, indices);
  }

  void testSVD(Eigen::Matrix3Xf x){
    Eigen::JacobiSVD<Eigen::Matrix3Xf> svd(x, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << "SVD of 3x3 identity matrix: " << std::endl;
    std::cout << "U matrix: " << std::endl;
    std::cout << svd.matrixU() << std::endl;
    std::cout << "Sigma matrix: " << std::endl;
    std::cout << svd.singularValues() << std::endl;
    std::cout << "V matrix: " << std::endl;
    std::cout << svd.matrixV() << std::endl;
    std::cout << "-------------------------------------------" << std::endl;
  }

  void picoNN() {
    size_t const n = 600000, d = 3;
    int const kMaxLeafCount = 16;

    Eigen::Matrix3Xf x = Eigen::Matrix3Xf::Random(d,n);    
    
    auto start = std::chrono::high_resolution_clock::now();  

    pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xf>> tree(x, kMaxLeafCount);

    pico_tree::KdTree<pico_tree::EigenTraits<Eigen::Matrix3Xf>> *treePtr = &tree;

    auto stop = std::chrono::high_resolution_clock::now();  
    auto timespan = std::chrono::duration<double>(stop - start);

    std::cout << "creating the tree took: " << timespan.count() << " seconds." << std::endl;

    Eigen::Matrix3Xf testPoints = x(Eigen::placeholders::all, {0,1,2,3,4,5});

    std::cout << "Test Points: " << testPoints << std::endl;

    closest_alignment ca = get_closest_alignment(testPoints,x,treePtr,1.0);

    if(ca.s_vals!=testPoints){
      std::cout << "Expected: " << testPoints << std::endl;
      std::cout << "Received: " << ca.s_vals << std::endl;
    } else {
      std::cout << "S Vals: " << ca.s_vals << std::endl;
      std::cout << "Distances: " << std::endl;
      std::copy(ca.dists.begin(),
                ca.dists.end(),
                std::ostream_iterator<int>(std::cout," "));
      std::cout << std::endl;
      std::cout << "Indices: " << std::endl;
      std::copy(ca.close_enough_inds.begin(),
                ca.close_enough_inds.end(),
                std::ostream_iterator<int>(std::cout," "));
      std::cout << std::endl;
      std::cout << std::endl;

      Eigen::Matrix3Xf i = Eigen::Matrix3f::Identity();
      testSVD(i);
    }
  }
}
