/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Raphael Favier, Technical University Eindhoven, (r.mysurname <aT>
 * tue.nl)
 */

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/mls.h>


int main(int argc, char **argv) {
  std::string input_file, output_file;

  if (argc < 3) {
    printf("Two args are required. output_file, output_file\n");
    exit(0);
  }
  input_file = argv[1];
  output_file = argv[2];


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(input_file, *cloud);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);

  mls.setComputeNormals (true);
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
  mls.process (*cloud_with_normals);

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree_pointnormal(
      new pcl::search::KdTree<pcl::PointNormal>);
  tree_pointnormal->setInputCloud(cloud_with_normals);

  pcl::MarchingCubesHoppe<pcl::PointNormal> marching_cubes;
  pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

  marching_cubes.setIsoLevel(0.0);
  marching_cubes.setGridResolution(50, 50, 50);
  marching_cubes.setPercentageExtendGrid(0.f);

  marching_cubes.setInputCloud(cloud_with_normals);
  marching_cubes.setSearchMethod(tree_pointnormal);
  marching_cubes.reconstruct(*triangles);

  pcl::io::savePLYFile(output_file, *triangles);

  return (0);
}
