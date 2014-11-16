#ifndef ICP_REGISTRATION_H
#define ICP_REGISTRATION_H
#include "common.h"
#include "filter.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>

/**
 * @brief icp_Align
 * @param cloud_src
 * @param cloud_tgt
 * @param final_transform
 * @param fitnessscore
 */
void
icp_Align(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform, double *fitnessscore)
{   
    pcl::IterativeClosestPoint<PointT,PointT> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_tgt);
    icp.setMaximumIterations(100);
    //icp.setRANSACIterations(100);
    PointCloud Final;
    icp.align(Final);
    if(icp.hasConverged())
    {
        final_transform = icp.getFinalTransformation();
        *fitnessscore = icp.getFitnessScore();
        cout << *fitnessscore << endl;
    }
}

/**
 * @brief icpQuadrantAlign
 * @param cloud_src
 * @param srcQuadrant
 * @param cloud_tgt
 * @param tagQuadrant
 * @param final_transform
 * @param fitnessscore
 */
void icpQuadrantAlign(const PointCloud::Ptr cloud_src, Quadrant srcQuadrant, const PointCloud::Ptr cloud_tgt,
                      Quadrant tagQuadrant, Eigen::Matrix4f &final_transform, double *fitnessscore)
{
    //    int role = 0;
    //    Quadrant quadrant = ThirdQuadrant;
    //    cloudQuadrantFilter( limitCloud1, cloud_1_rotated_filtered, cloudInputShape, adjuestLength, role, quadrant );
    //    if( cloud_1_rotated_filtered->points.size() == 0 )
    //    {
    //        fprintf(stderr, "this quadrant has no data, of line %d!!\n", __LINE__);
    //        exit(-1);
    //    }
    //    role = 1;
    //    cloudQuadrantFilter( cloud_1_rotated_filtered, cloud_1_rotated_filtered, cloudInputShape, adjuestLength, role, quadrant );
    //    if( cloud_1_rotated_filtered->points.size() == 0 )
    //    {
    //        fprintf(stderr, "this quadrant has no data, of line %d!!\n", __LINE__);
    //        exit(-1);
    //    }
    //    quadrant = FourthQuadrant;
    //    role = 0;
    //    cloudQuadrantFilter( limitCloud2, cloud_2_rotated_filtered, cloudTagShape, adjuestLength, role, quadrant );
    //    if( cloud_2_rotated_filtered->points.size() == 0 )
    //    {
    //        fprintf(stderr, "this quadrant has no data, of line %d!!\n", __LINE__);
    //        exit(-1);
    //    }
    //    role = 1;
    //    cloudQuadrantFilter( cloud_2_rotated_filtered, cloud_2_rotated_filtered, cloudTagShape, adjuestLength, role, quadrant );
    //    if( cloud_2_rotated_filtered->points.size() == 0 )
    //    {
    //        fprintf(stderr, "this quadrant has no data, of line %d!!\n", __LINE__);
    //        exit(-1);
    //    }
    //    getPointCloudShape( cloud_1_rotated_filtered, cloudInputShape );
    //    getPointCloudShape( cloud_2_rotated_filtered, cloudTagShape );
    //    cout << "cloud  shape1: \n";
    //    printPointCloudShape(cloudInputShape);
    //    cout << "cloud  shape2: \n";
    //    printPointCloudShape(cloudTagShape);
    //    pcl::io::savePCDFile("cloud_1.pcd",*cloud_1_rotated_filtered,false);
    //    pcl::io::savePCDFile("cloud_2.pcd",*cloud_2_rotated_filtered,false);
    //    Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
    //    double fitnessscore = 0.0;
    //    icp_Align(cloud_2_rotated_filtered,cloud_1_rotated_filtered,Transform,&fitnessscore);
}

#endif // ICP_REGISTRATION_H
