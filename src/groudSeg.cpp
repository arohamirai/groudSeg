#include<iostream>
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <opencv2/opencv.hpp>
#include <vtkGenericDataObjectReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace boost;
using namespace std;
using namespace cv;

int user_data = 0;
void
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}
int main(int argc, char** agrv)
{
    vtkSmartPointer<vtkPLYReader> plyReader =
            vtkSmartPointer<vtkPLYReader>::New();
    plyReader->SetFileName("/home/lf/MySoft/groudSeg/pointCloud/merge.ply");
    plyReader->Update();


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::vtkPolyDataToPointCloud(plyReader->GetOutput(), *cloud);

    //
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    seg.setDistanceThreshold (1);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
     {
       PCL_ERROR ("Could not estimate a planar model for the given dataset.");
       return (-1);
     }

     std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                         << coefficients->values[1] << " "
                                         << coefficients->values[2] << " "
                                         << coefficients->values[3] << std::endl;

     std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
//     for (size_t i = 0; i < inliers->indices.size (); ++i)
//       std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
//                                                  << cloud->points[inliers->indices[i]].y << " "
//                                                  << cloud->points[inliers->indices[i]].z << std::endl;






















    //    std::cout << "Loaded "
    //              << cloud->width * cloud->height
    //              << " data points from test_pcd.pcd with the following fields: "
    //              << std::endl;
    //    for (size_t i = 0; i < cloud->points.size (); ++i)
    //        std::cout << "    " << cloud->points[i].x
    //                  << " "    << cloud->points[i].y
    //                  << " "    << cloud->points[i].z << std::endl;



    //    // visualization
    //    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //    viewer.showCloud(cloud,"abc");
    //    viewer.runOnVisualizationThread (viewerPsycho);
    //        while (!viewer.wasStopped ())
    //        {
    //        //you can also do cool processing here
    //        //FIXME: Note that this is running in a separate thread from viewerPsycho
    //        //and you should guard against race conditions yourself...
    //        user_data++;
    //        }
    return 0;
}
