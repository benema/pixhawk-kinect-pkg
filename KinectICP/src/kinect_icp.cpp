#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pointcloud_registration4/pointcloud_registration_point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/SVD>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <vector>

//pcl include
#include "pcl_ros/io/bag_io.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <pcl_ros/point_cloud.h>
#include "pcl/registration/registration.h"
#include "pcl/features/feature.h"
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_registration.h"
#include "pcl/registration/icp_nl.h"
#include <pcl/common/transformation_from_correspondences.h>
#include <Eigen/Core>
#include <math.h>
#include "pcl/filters/statistical_outlier_removal.h" // to filter outliers

#include "pcl/filters/voxel_grid.h" //for downsampling the point cloud

#include "pcl/kdtree/kdtree_flann.h" //for the kdtree
//#include "pcl/octree/octree.h"	//for octree

#include "pcl/registration/transforms.h" //for the transformation function
#include <pcl/features/normal_3d_omp.h>

#include <pointcloud_registration4/icp/icp_correspondences_check.h> //for icp
#include <algorithm> //for the sort and unique functions

#include <ctime>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
using namespace std;

class PointCloudRegistration
{
  public:
    PointCloudRegistration();
    ~PointCloudRegistration();
    void pointcloudRegistrationCallBack(const sensor_msgs::PointCloud2& msg);
    Eigen::Matrix4f getOverlapTransformation();
    void publishPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pointcloud);
    pcl::PointCloud<pcl::PointXYZRGB> convertFromMsgToPointCloud(const sensor_msgs::PointCloud2& pointcloud_msg);
    pcl::PointCloud<pcl::PointXYZRGB> convertFromMsgToPointCloud_publish(const sensor_msgs::PointCloud2& pointcloud_msg);
    void run();

  private:
    ros::NodeHandle nh_;
    std::string merged_pointcloud_topic_, subscribe_pointcloud_topic_, frame_id_;
    int max_number_of_iterations_icp_, max_nn_icp_, max_nn_overlap_;
    double downsample_leafsize_, epsilon_z_, epsilon_curvature_, epsilon_transformation_, radius_icp_, radius_overlap_;
    bool downsample_pointcloud_before_, downsample_pointcloud_after_, filter_outliers_, curvature_check_;
    int scan_index_, counter_;
    clock_t start, end;
    Eigen::Matrix4f final_transformation_;
    ros::Subscriber pointcloud_subscriber_;
    ros::Publisher pointcloud_merged_publisher_;
    double ttImage;

    //pcl::IterativeClosestPointCorrespondencesCheck<pcl::PointXYZRGB, pcl::PointXYZRGB> icp_; // for icp
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_;  // for kdtree
    
    //pcl::octree::OctreePointCloud<PointXYZRGB> octree_(128.0f); //for octree	float resolution = 128.0f;
    
    bool firstCloudReceived_, secondCloudReceived_;
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud2_current_, pointcloud2_merged_, pointcloud2_transformed_;
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud2_current_publish_, pointcloud2_merged_publish_, pointcloud2_transformed_publish_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void estimateRigidTransformationSVD (const pcl::PointCloud<pcl::PointXYZRGB> &cloud_src, const pcl::PointCloud<pcl::PointXYZRGB> &cloud_tgt, Eigen::Matrix4f &transformation_matrix)
{
         ROS_ASSERT (cloud_src.points.size () == cloud_tgt.points.size ());
 
	if (cloud_src.points.size () != cloud_tgt.points.size ())
	{
	  ROS_ERROR ("[pcl::estimateRigidTransformationSVD] Number or points in source (%zu) differs than target (%zu)!", cloud_src.points.size (), cloud_tgt.points.size ());
	  return;
	}

         // <cloud_src,cloud_src> is the source dataset
 	 transformation_matrix.setIdentity ();

         Eigen::Vector4f centroid_src, centroid_tgt;
         // Estimate the centroids of source, target
         compute3DCentroid (cloud_src, centroid_src);
         compute3DCentroid (cloud_tgt, centroid_tgt);
 
         // Subtract the centroids from source, target
         Eigen::MatrixXf cloud_src_demean;
         demeanPointCloud (cloud_src, centroid_src, cloud_src_demean);
 
 
         Eigen::MatrixXf cloud_tgt_demean;
         demeanPointCloud (cloud_tgt, centroid_tgt, cloud_tgt_demean);
 
         // Assemble the correlation matrix H = source * target'
         Eigen::Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();
 
         // Compute the Singular Value Decomposition
         Eigen::JacobiSVD<Eigen::Matrix3f> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
         Eigen::Matrix3f u = svd.matrixU ();
         Eigen::Matrix3f v = svd.matrixV ();
 

         Eigen::Matrix3f R = v * u.transpose ();
 
         // Return the correct transformation
         transformation_matrix.topLeftCorner<3, 3> () = R;
         Eigen::Vector3f Rc = R * centroid_src.head<3> ();
         transformation_matrix.block <3, 1> (0, 3) = centroid_tgt.head<3> () - Rc;
	
	//return transofrmation_matrix;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f Ransac(const pcl::PointCloud<pcl::PointXYZRGB> & PointCloud1, const pcl::PointCloud<pcl::PointXYZRGB> & PointCloud2, int ransac_iterations)
{
	srand(time(0));
	pcl::TransformationFromCorrespondences tfc;
	Eigen::Matrix4f transformation;
	//transformation.setIdentity();

	float max_error = 0.075;
	int noInliers = 0;

	Eigen::Vector4f surfCloud1, surfCloud2; // used to transform PointCloud coordinates to homogenous coordinates
	pcl::PointCloud<pcl::PointXYZRGB> source, target;
	
	for (int i = 0; i < ransac_iterations; i++) 
	{
	
		
         	tfc.reset();
 		pcl::PointCloud<pcl::PointXYZRGB> source_temp, target_temp;

		int tempNoInliers = 0;
         	int sample_size = 3; // chose randomly 3 points from the correspondences
		int ealier_id = 0;
		
         	for (int k = 0; k < sample_size; k++) 
		{
			//cout<<"Test "<<PointCloud1.size()<<endl;
			
             		int id = rand() % (PointCloud1.size());
			
			if((id - ealier_id) == 0)	// checking that the same point is not drawn twice 
				break;
			
             		Eigen::Vector3f from(PointCloud1.points[id].x,PointCloud1.points[id].y,PointCloud1.points[id].z);
			Eigen::Vector3f   to(PointCloud2.points[id].x,PointCloud2.points[id].y,PointCloud2.points[id].z);
			
             		tfc.add(from, to);
			ealier_id = id;
         	}
		

		Eigen::Matrix4f tempTransformation(tfc.getTransformation().matrix());
		
		//calculating error and finding inliers
		for(size_t j = 0; j<PointCloud1.size();j++)
		{
			surfCloud1[0] = PointCloud1.points[j].x;
			surfCloud1[1] = PointCloud1.points[j].y;
			surfCloud1[2] = PointCloud1.points[j].z;
			surfCloud1[3] = 1;
			surfCloud2[0] = PointCloud2.points[j].x;
			surfCloud2[1] = PointCloud2.points[j].y;
			surfCloud2[2] = PointCloud2.points[j].z;
			surfCloud2[3] = 1;

			Eigen::Vector4f vec = tempTransformation * surfCloud1 - surfCloud2;
		        double error = vec.dot(vec);
		
			if(error<max_error)
			{
				tempNoInliers++;
				source_temp.push_back(PointCloud1.points[j]);
				target_temp.push_back(PointCloud2.points[j]);
			}
		}
		
		// only in case the next iteration brings us more inliers than all the previous ones we update the transformation
		if((tempNoInliers > noInliers))
		{
			noInliers = tempNoInliers;
			source = source_temp;
			target = target_temp;
		}
	}
	estimateRigidTransformationSVD(source, target, transformation);

	return transformation;
}

//--------------------------------------------------------------------------------------------------------------------------------------
Eigen::Matrix4f PointCloudRegistration::getOverlapTransformation()
{
  // In this function we extract the overlapped region of the two points cloud and compute
  // the transformation and return it.

  if(firstCloudReceived_ == false && secondCloudReceived_ == false )
  {
    ROS_ERROR("getOverlapTransformation called before receiving atleast 2 point clouds");
    exit(1);
  }
  else
  {
	int iter = 10;
	Eigen::Matrix4f transformation;
	if(counter_>2)
		transformation = final_transformation_;
	else
		transformation.setIdentity();
	int max_overlap = 1;
	double radius_overlap = 0.1;
	int prev =0, next = 0;
	int count=0;
	for(int i =0; i<=iter;i++)
	{
	
    		pcl::PointCloud<pcl::PointXYZRGB> transformed_PointCloud_current;

		pcl::transformPointCloud(pointcloud2_current_, transformed_PointCloud_current,transformation);	

		//kdtree
		pcl::PointCloud<pcl::PointXYZRGB> overlap_model, overlap_current;
		//cout<<"ICP "<<i<<endl;
	     	
		int count = 0;
     		for(size_t idx = 0 ; (idx < transformed_PointCloud_current.points.size()); idx++ )
	     	{
			if((!isnan(transformed_PointCloud_current.points[idx].x)) && (!isnan(transformed_PointCloud_current.points[idx].y)) && (!isnan(transformed_PointCloud_current.points[idx].z)))
			{
				std::vector<int> k_indices(max_overlap);
				std::vector<float> k_distances(max_overlap);
				kdtree_. nearestKSearch (transformed_PointCloud_current, idx, 1, k_indices, k_distances);
				//cout<< k_distances[0]<<" ";
				if(k_indices.size() > 0 )
      				{
					
					if((!isnan(kdtree_.getInputCloud()->points[k_indices[0]].x)) 
					&& (!isnan(kdtree_.getInputCloud()->points[k_indices[0]].y)) 
					&& (!isnan(kdtree_.getInputCloud()->points[k_indices[0]].z)))
					{
						overlap_current.points.push_back(transformed_PointCloud_current.points[idx]);
						overlap_model.points.push_back (kdtree_.getInputCloud()->points[k_indices[0]]);
						
					}
	     			}
			}
		}

    		if(i == iter)
		{
			//total_distance += k_distances[index];
			cout<<"overlap_current.size "<< overlap_current.size()<<endl;
							
		}
		

		Eigen::Matrix4f temp_transformation;
		/*
		if(counter_>2)
		{
			//start measure time
			//ttImage = 0.;
			//ttImage = (double)cvGetTickCount();

			temp_transformation = Ransac(overlap_current, overlap_model, 100);

			//stop measuring time
		    	//ttImage = (double)cvGetTickCount() - ttImage;
			//print in milliseconds
		  	//double s = 1./(cvGetTickFrequency()*1000.);
		  	//printf("Image:\t%6.1f ms\n", ttImage*s);
		  	//ROS_INFO("Ransac time:\t%6.1f ms\n", ttImage*s);
		}
		else
			estimateRigidTransformationSVD(overlap_current, overlap_model, temp_transformation);
		*/
		estimateRigidTransformationSVD(overlap_current, overlap_model, temp_transformation);

		transformation = transformation * temp_transformation;

	}
	cout<<transformation<<endl;
    return (transformation);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudRegistration::publishPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pointcloud)
{
  sensor_msgs::PointCloud2 mycloud;
  pcl::toROSMsg(pointcloud, mycloud);

  if( downsample_pointcloud_after_ == true)
  {
    // for downsampling of pointclouds
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor_;
    sensor_msgs::PointCloud2 mycloud_downsampled;

    //Now we will downsample the point cloud
    sor_.setInputCloud (boost::make_shared<sensor_msgs::PointCloud2> (mycloud));
    sor_.setLeafSize (downsample_leafsize_, downsample_leafsize_, downsample_leafsize_);
    sor_.filter (mycloud_downsampled);
    mycloud_downsampled.header.frame_id = frame_id_;
    mycloud_downsampled.header.stamp = ros::Time::now();

    pointcloud_merged_publisher_.publish(mycloud_downsampled);
  }
  else
  {
    mycloud.header.frame_id = frame_id_;
    mycloud.header.stamp = ros::Time::now();

    pointcloud_merged_publisher_.publish(mycloud);
  }
  ROS_INFO("Merged Point cloud published");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudRegistration::PointCloudRegistration(): nh_("~")
{
  nh_.param("publish_merged_pointcloud_topic", merged_pointcloud_topic_, std::string("/merged_pointcloud"));
  nh_.param("subscribe_pointcloud_topic", subscribe_pointcloud_topic_, std::string("/shoulder_cloud"));
  nh_.param("max_number_of_iterations_icp", max_number_of_iterations_icp_, 50);
  nh_.param("max_nn_icp", max_nn_icp_, 100);
  nh_.param("max_nn_overlap", max_nn_overlap_, 10);
  nh_.param("radius_icp", radius_icp_, 0.05);
  nh_.param("radius_overlap", radius_overlap_, 0.1);
  nh_.param("curvature_check", curvature_check_, true);
  nh_.param("downsample_pointcloud_before", downsample_pointcloud_before_, true);
  nh_.param("downsample_pointcloud_after", downsample_pointcloud_after_, false);
  nh_.param("filter_outliers", filter_outliers_, true);
  nh_.param("downsample_leafsize", downsample_leafsize_, 0.1);
  nh_.param("epsilon_z", epsilon_z_, 0.001);
  nh_.param("epsilon_curvature", epsilon_curvature_, 0.001);
  nh_.param("epsilon_transformation", epsilon_transformation_, 1e-6);
  firstCloudReceived_ = false;
  secondCloudReceived_ = false;
  scan_index_ = 0;
  counter_ = 0;
  //icp_.setMaximumIterations(max_number_of_iterations_icp_);
 // icp_.setTransformationEpsilon(epsilon_transformation_);
 // icp_.setParameters(radius_icp_, max_nn_icp_, epsilon_z_, epsilon_curvature_, curvature_check_ );
  ROS_INFO("pointcloud_registration node is up and running.");

  run();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void downSample(const pcl::PointCloud<pcl::PointXYZRGB>& src, pcl::PointCloud<pcl::PointXYZRGB>& to)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> down_sampler;
	down_sampler.setLeafSize (0.04, 0.04, 0.04);
	pcl::PCLBase<pcl::PointXYZRGB>::PointCloudConstPtr const_cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> > (src);
	down_sampler.setInputCloud (const_cloud_ptr);
	down_sampler.filter(to);
	ROS_INFO("gicp.cpp: Downsampling from %i to %i", (int) src.points.size(), (int) to.points.size());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZRGB> PointCloudRegistration::convertFromMsgToPointCloud(const sensor_msgs::PointCloud2& pointcloud_msg)
{
 
  //incrementing the scan index
  scan_index_++;
  // Declaring some variables required in this function
  sensor_msgs::PointCloud2 pointcloud2_msg = pointcloud_msg;
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl_step01, pointcloud_pcl_step02;
   
  pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree_ptr_;
  tree_ptr_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
  std::vector<int> indices;
 //cout<<"hahaha"<<endl;
  
  // STEP 01: Check if we should downsample the input cloud or not
 
    //cout<<"hahaha"<<endl;
    ROS_INFO ("PointCloud before downsampling: %d .", pointcloud2_msg.width * pointcloud2_msg.height);
    sensor_msgs::PointCloud2 pointcloud_downsampled_msg;
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor_; // for downsampling of pointclouds

    //Now we will downsample the point cloud
    sor_.setInputCloud (boost::make_shared<sensor_msgs::PointCloud2> (pointcloud2_msg));
    sor_.setLeafSize(0.25, 0.25, 0.25);
    sor_.filter (pointcloud_downsampled_msg);
    ROS_INFO ("PointCloud after downsampling: %d .", pointcloud_downsampled_msg.width * pointcloud_downsampled_msg.height);
   // cout<<"test"<<endl;
     //Converting from PointCloud2 msg format to pcl pointcloud format
    pcl::fromROSMsg(pointcloud_downsampled_msg, pointcloud_pcl_step01);
   // cout<<"end test"<<endl;
  


  // STEP 02: Check if we should filter the outliers or not


    // Removing outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(pointcloud_pcl_step01));
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (pointcloud_pcl_step02);
 
  tree_ptr_->setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> > (pointcloud_pcl_step02));
  indices.resize (pointcloud_pcl_step02.points.size ());
  for (size_t i = 0; i < indices.size (); ++i)
  {
    indices[i] = i;
  }
  
 // cout<<"end convert"<<endl;
  

  return (pointcloud_pcl_step02);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZRGB> PointCloudRegistration::convertFromMsgToPointCloud_publish(const sensor_msgs::PointCloud2& pointcloud_msg)
{
 
  //incrementing the scan index
  scan_index_++;
  // Declaring some variables required in this function
  sensor_msgs::PointCloud2 pointcloud2_msg = pointcloud_msg;
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl_step01, pointcloud_pcl_step02;
   
  pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree_ptr_;
  tree_ptr_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
  std::vector<int> indices;
 //cout<<"hahaha"<<endl;
  
  // STEP 01: Check if we should downsample the input cloud or not
 
    //cout<<"hahaha"<<endl;
    ROS_INFO ("PointCloud before downsampling: %d .", pointcloud2_msg.width * pointcloud2_msg.height);
    sensor_msgs::PointCloud2 pointcloud_downsampled_msg;
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor_; // for downsampling of pointclouds

    //Now we will downsample the point cloud
    sor_.setInputCloud (boost::make_shared<sensor_msgs::PointCloud2> (pointcloud2_msg));
    sor_.setLeafSize(0.01,0.01,0.01);					//(downsample_leafsize_, downsample_leafsize_, downsample_leafsize_);
    sor_.filter (pointcloud_downsampled_msg);
    ROS_INFO ("PointCloud after downsampling: %d .", pointcloud_downsampled_msg.width * pointcloud_downsampled_msg.height);
   // cout<<"test"<<endl;
    // Converting from PointCloud2 msg format to pcl pointcloud format
    pcl::fromROSMsg(pointcloud_downsampled_msg, pointcloud_pcl_step01);
   // cout<<"end test"<<endl;
  


  // STEP 02: Check if we should filter the outliers or not


    // Removing outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(pointcloud_pcl_step01));
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (pointcloud_pcl_step02);
 
  tree_ptr_->setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> > (pointcloud_pcl_step02));

  return (pointcloud_pcl_step02);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudRegistration::~PointCloudRegistration()
{
  ROS_INFO("Shutting down pointcloud_registration node!.");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudRegistration::run()
{
  pointcloud_subscriber_ = nh_.subscribe(subscribe_pointcloud_topic_, 100, &PointCloudRegistration::pointcloudRegistrationCallBack, this);
  pointcloud_merged_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(merged_pointcloud_topic_, 100);
  ros::spin();
}

void PointCloudRegistration::pointcloudRegistrationCallBack(const sensor_msgs::PointCloud2& pointcloud_msg)
{
	
	counter_++;
	frame_id_ = pointcloud_msg.header.frame_id;
	
	if( firstCloudReceived_ == false)
	{
		pointcloud2_current_ = convertFromMsgToPointCloud(pointcloud_msg);
		pointcloud2_current_publish_ = convertFromMsgToPointCloud_publish(pointcloud_msg);
 		ROS_INFO("Size of point cloud received = %d", (int) pointcloud2_current_.points.size());
		firstCloudReceived_ = true;
    		ROS_INFO("Received first point cloud.");
    		kdtree_.setInputCloud(boost::make_shared< pcl::PointCloud < pcl::PointXYZRGB> > (pointcloud2_current_));
    		//octree_.addPointsFromInputCloud ();
    		//pcl::fromROSMsg(pointcloud_msg, pointcloud2_current_publish_);
    
    		pointcloud2_merged_ = pointcloud2_current_;
    		pointcloud2_merged_publish_ = pointcloud2_current_publish_;
		//pointcloud2_transformed_publish_ = pointcloud2_current_publish_;
		//publishPointCloud(pointcloud2_merged_publish_);
	}
	else if( secondCloudReceived_ == false)
	{
	    	ROS_INFO("Received second point cloud.");
	    	secondCloudReceived_ = true;
	    	pointcloud2_current_ = convertFromMsgToPointCloud(pointcloud_msg);
	    	pointcloud2_current_publish_ = convertFromMsgToPointCloud_publish(pointcloud_msg);

	    	//Now we get the transformation from the overlapped regions of the 2 point clouds
	    	final_transformation_= getOverlapTransformation();
	   	// pcl::fromROSMsg(pointcloud_msg, pointcloud2_current_publish_);
	    	//end = clock();

	    	pcl::transformPointCloud(pointcloud2_current_, pointcloud2_transformed_, final_transformation_);
	    	pointcloud2_merged_ += pointcloud2_transformed_;
		

	    	pcl::transformPointCloud(pointcloud2_current_publish_, pointcloud2_transformed_publish_, final_transformation_);
	    	//pointcloud2_merged_publish_ += pointcloud2_transformed_publish_;

		//publishPointCloud(pointcloud2_merged_publish_);
		//publishPointCloud(pointcloud2_transformed_publish_);

	}
	else
	{
	  	//if((counter_==1)||(counter_%1 == 0))
		//{	
			//start measure time
			ttImage = 0.;
			ttImage = (double)cvGetTickCount();

			ROS_INFO("Received point cloud number: %d", counter_);
			pointcloud2_current_ = convertFromMsgToPointCloud(pointcloud_msg);
			    	
			kdtree_.setInputCloud(boost::make_shared< pcl::PointCloud < pcl::PointXYZRGB> > (pointcloud2_merged_));
			//odtree_.addPointsFromInputCloud ();

			//Now we get the transformation from the overlapped regions of the 2 point clouds
			final_transformation_= getOverlapTransformation();

			//stop measuring time
		    	ttImage = (double)cvGetTickCount() - ttImage;
			//print in milliseconds
		  	double s = 1./(cvGetTickFrequency()*1000.);
		  	//printf("Image:\t%6.1f ms\n", ttImage*s);
		  	ROS_INFO("Image:\t%6.1f ms\n", ttImage*s);

			// pcl::fromROSMsg(pointcloud_msg, pointcloud2_current_publish_);
			pcl::transformPointCloud(pointcloud2_current_, pointcloud2_transformed_, final_transformation_);
			pointcloud2_merged_ += pointcloud2_transformed_;

			//if((counter_==1)||(counter_%2 == 0))
			//{
				 
				pointcloud2_current_publish_ = convertFromMsgToPointCloud_publish(pointcloud_msg);
		    		pcl::transformPointCloud(pointcloud2_current_publish_, pointcloud2_transformed_publish_, final_transformation_);
		    		//pointcloud2_merged_publish_ += pointcloud2_transformed_publish_;
				//publishPointCloud(pointcloud2_merged_publish_);
				publishPointCloud(pointcloud2_transformed_publish_);
				
		  		cout<<"Cloud(pointcloud2_transformed_publish_size "<<pointcloud2_merged_publish_.size()<<endl;
			//}
			cout<<"pointcloud2_merged_.size() "<<pointcloud2_merged_.size()<<endl;
		//}

		//Downsample if pointcloud2_merged_ > 50,000 or pointcloud2_merged_publish_.size()>500000
		if(pointcloud2_merged_.size()>1000000)
		{
			pcl::PointCloud<pcl::PointXYZRGB>& pointcloud2_merged_1 = pointcloud2_merged_, pointcloud2_merged_2 = pointcloud2_merged_;
			downSample(pointcloud2_merged_1, pointcloud2_merged_2);
			pointcloud2_merged_ = pointcloud2_merged_2;
		}
		if(pointcloud2_merged_publish_.size()>100000000)
		{
			pcl::PointCloud<pcl::PointXYZRGB>& pointcloud2_merged_publish_1 = pointcloud2_merged_publish_, pointcloud2_merged_publish_2 = pointcloud2_merged_publish_;
			downSample(pointcloud2_merged_publish_1, pointcloud2_merged_publish_2);
			pointcloud2_merged_publish_ = pointcloud2_merged_publish_2;
		}
		
	}
  
  //publishPointCloud(pointcloud2_transformed_publish_);

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_registration");
    PointCloudRegistration pointcloud_registration;
    return(0);
}
