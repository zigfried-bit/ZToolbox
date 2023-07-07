#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Core>

typedef pcl::PointXYZ PointType;

Eigen::Vector4f estimateGroundPlane(pcl::PointCloud<PointType>::Ptr cloud_in, const float in_distance_there);

int main(int argc, char** argv) {
    std::cout << "Height Measurement starting ..." << std::endl;
    // 读取pcd文件
    const std::string infile("../../../src/ZToolbox/lidar_height_mes/data/1422133388.503344128.pcd");
    pcl::PointCloud<PointType>::Ptr raw_data(new pcl::PointCloud<PointType>);
    if(pcl::io::loadPCDFile(infile, *raw_data) == -1) {
        PCL_ERROR("Can't load file!");
        return -1;
    }
    // 选取感兴趣区域
    pcl::PassThrough<PointType> passz;
    passz.setInputCloud(raw_data);
    passz.setFilterFieldName("z");
    passz.setFilterLimits(-5.0, -1.5);
    passz.filter(*raw_data);


    pcl::visualization::PCLVisualizer viewer("raw_data viewer");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color(raw_data, "z");// 按照z字段进行渲染
    // pcl::visualization::PointCloudColorHandlerCustom<PointType> pureColor(raw_data, 255, 0, 0);// 纯色
    viewer.addPointCloud(raw_data, color, "raw_data");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "raw_data"); // 设置点云大小
    viewer.addCoordinateSystem(5.0);// 添加坐标系

    //添加平面，四个参数分别代表(a, b, c, d with ax+by+cz+d=0) 
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (1.0);
    coeffs.values.push_back (0.0);
    viewer.addPlane (coeffs, "plane");
    


    // 提取地面法向量
    Eigen::Vector4f planeVec;
    planeVec = estimateGroundPlane(raw_data, 0.1);
    Eigen::Vector3f normalVec{planeVec.x(), planeVec.y(), planeVec.z()};
    printf("地面的法向量是：(%f, %f, %f)\n", planeVec.x(), planeVec.y(), planeVec.z());
    printf("平面方程：%fx + %fy + %fz + %f = 0\n", planeVec.x(), planeVec.y(), planeVec.z(), planeVec.w());

    pcl::ModelCoefficients ext_coeffs;
    ext_coeffs.values.push_back (planeVec.x());
    ext_coeffs.values.push_back (planeVec.y());
    ext_coeffs.values.push_back (planeVec.z());
    ext_coeffs.values.push_back (planeVec.w());
    viewer.addPlane (ext_coeffs, "extPlane");

    while (!viewer.wasStopped()) {
       viewer.spinOnce();
    }
    return 0;
}

// 提取地面法向量
Eigen::Vector4f estimateGroundPlane(pcl::PointCloud<PointType>::Ptr cloud_in, const float in_distance_there)
{
    Eigen::Vector4f normal_vector;
    pcl::SACSegmentation<PointType> plane_seg;
    pcl::PointIndices::Ptr Plane_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    plane_seg.setOptimizeCoefficients(true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(in_distance_there);//视情况定
    plane_seg.setInputCloud(cloud_in);
    plane_seg.segment(*Plane_inliers, *plane_coefficients);
    normal_vector = { plane_coefficients->values[0],plane_coefficients->values[1],plane_coefficients->values[2], plane_coefficients->values[3] };
    return normal_vector;
}