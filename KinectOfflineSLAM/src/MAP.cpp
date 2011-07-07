#include "MAP.h"

static CvScalar colors[] =
{
		{{0,0,255}},
		{{0,128,255}},
		{{0,255,255}},
		{{0,255,0}},
		{{255,128,0}},
		{{255,255,0}},
		{{255,0,0}},
		{{255,0,255}},
		{{255,255,255}}
};

MAP::MAP(float thresh, int iterations,int minimal_inliers, int keyframe_inliers,bool verbose, int near_keyframe_inliers,string filepath,int initialization)
{
	//Initialization of start parameters
	showDisplay=verbose;
	ransac_acc=thresh;
	ransac_it=iterations;
	min_inliers=minimal_inliers;
	min_keyframe_inlier=keyframe_inliers;
	min_keyframe_redection_inliers=near_keyframe_inliers;
	path=filepath;
	init=initialization;

	//other variables
	detector = new cv::GridAdaptedFeatureDetector(new cv::FastFeatureDetector,500);
	extractor=new cv::BriefDescriptorExtractor;
	called=false;
	counter=0;
	called_first_time=true;
	cv_image[0]=cvCreateImage(cvSize(640,480),8,3);
	cv_image[1]=cvCreateImage(cvSize(640,480),8,3);
	imgadd=cvCreateImage( cvSize(640, 960), IPL_DEPTH_8U, 3 );
	callback_image=cvCreateImage(cvSize(640,480),8,3);
	compute_transform_success=true;
	transformation_at_least_once_computed=false;
	stop_mapping=false;
	vicon_computed=false;
	imu_computed=false;
	next_keyframe=false;
	FeaturePointCloud[0].header.frame_id=std::string("/openni_depth_optical_frame");
	FeaturePointCloud[0].is_dense = false;
	FeaturePointCloud[1].header.frame_id=std::string("/openni_depth_optical_frame");
	FeaturePointCloud[1].is_dense = false;
	transformation_at_least_twice_computed=false;
	actual_keyframe=0;
	take_vicon=false;
	vicontransform=Eigen::Matrix4f::Identity();



	ros::NodeHandle n;


	//time the incoming rgb and cloud msgs
	message_filters::Subscriber<PointCloud> pointCloud_sub(n, "/camera/depth/points", 1);
	message_filters::Subscriber<sensor_msgs::Image> rgbImage_sub(n, "/camera/rgb/image_color", 1);
	typedef message_filters::sync_policies::ApproximateTime<PointCloud, sensor_msgs::Image> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pointCloud_sub, rgbImage_sub);



	//Callback functions for MAVLINK stuff
	imuSubscriber = n.subscribe ("/fromMAVLINK/Imu",  1, &MAP::imuCallback,  this);
	viconSubscriber= n.subscribe("/fromMAVLINK/Vicon",1,&MAP::viconCallback,this);
	commandSubscriber= n.subscribe("/fromMAVLINK/COMMAND",1,&MAP::commandCallback,this);

	KeyFramePoints=n.advertise<PointCloud>("KinectOfflineSLAM/keyframepoints",1);




	sync.registerCallback(&MAP::callback,this);
	//	ros::spin();

	ros::AsyncSpinner spinner(0);
	spinner.start();
	if(init==2)
		while(vicon_computed==0&&ros::ok)
		{
			std::cout<<"Waiting for Vicon data"<<std::endl;
			cvWaitKey(1000);
		}
	if(init==1)
		while(imu_computed==0&&ros::ok)
		{
			std::cout<<"Waiting for IMU data"<<std::endl;
			cvWaitKey(1000);
		}

	while(ros::ok())
	{
		if(called==1)
		{
			if(called_first_time)
				counter=0;
			else
				counter=1;

			cvCopy(callback_image,cv_image[counter]);
			kinectCloud[counter]=callbackCloud;
			kinectCloud[counter].header.frame_id="/pgraph";
			called=0;
			PointCloud tmpCloud;
			tmpCloud.header.frame_id="/pgraph";
			std::vector<cv::KeyPoint> kpts_tmp;
			Point tmpPoint;
			cv::Mat dtorstmp;
			detector->detect(cv_image[counter],kpts[counter]);
			extractor->compute(cv_image[counter],kpts[counter],dtors[counter]);
			dtorstmp.flags=dtors[counter].flags;
			dtorstmp.dims=dtors[counter].dims;
			dtorstmp.cols=dtors[counter].cols;
			dtorstmp.step[0]=dtors[counter].step[0];
			dtorstmp.step[1]=dtors[counter].step[1];


			for(uint i=0;i<kpts[counter].size();i++)
			{
				if(	pcl_isfinite (kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).x) &&
						pcl_isfinite (kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).y) &&
						pcl_isfinite (kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y).z))
				{
					tmpPoint=kinectCloud[counter].at(kpts[counter].at(i).pt.x,kpts[counter].at(i).pt.y);
					dtorstmp.push_back(dtors[counter].row(i));
					kpts_tmp.push_back(kpts[counter].at(i));
					tmpCloud.push_back(tmpPoint);

				}
			}
			;
			dtors[counter]=dtorstmp;
			kpts[counter]=kpts_tmp;

			FrameData[counter].Points=tmpCloud;
			FrameData[counter].Descriptor=dtors[counter];
			FrameData[counter].Keypoints=kpts[counter];
			FrameData[counter].KinectCloud=kinectCloud[counter];

			if(counter==0)
			{
				//				std::cout<<"init:"<<init<<std::endl;
				//				std::cout<<"imuRot"<<std::endl<<imuRot<<std::endl;;
				if(init==2)
				{
					systemRot=vicontransform;
//					FrameData[counter].Transformation=vicontransform;
				}
				if(init==1)
				{
					systemRot=imuRot;
//					FrameData[counter].Transformation=imuRot;
				}
				if(init==0)
				{
					systemRot=Eigen::Matrix4f::Identity();
//					FrameData[counter].Transformation=Eigen::Matrix4f::Identity();
				}

				FrameData[counter].Transformation=Eigen::Matrix4f::Identity();

				KeyframeDataVector.push_back(FrameData[counter]);

				std::cout<<"vicontransform:"<<std::endl<<vicontransform<<std::endl;
				std::cout<<"size of keyframedatavector"<<KeyframeDataVector.size()<<std::endl;
				std::cout<<"Keyframedatavector.at(0).transformation after vicon"<<std::endl<<KeyframeDataVector.at(0).Transformation<<std::endl;
			}

			called_first_time=false;

			if(counter==1)
			{
				if(showDisplay)
				{
					cvSetImageROI( imgadd,cvRect( 0, 0,cv_image[0]->width, cv_image[0]->height ) );
					cvCopy(cv_image[0],imgadd);
					cvResetImageROI(imgadd);
					cvSetImageROI( imgadd,cvRect( 0, cv_image[0]->height,cv_image[0]->width, cv_image[0]->height ) );
					cvCopy(cv_image[1],imgadd);
					cvResetImageROI(imgadd);
				}



				matchFeature(dtors[0],dtors[1],matches_popcount);

				if(showDisplay)
					std::cout<<"5"<<std::endl;
				if(showDisplay)
					std::cout<<"size of matches_popcount"<<matches_popcount.size()<<std::endl;


				//do RANSAC and compute the transforamtion
				RANSACandTransformation();



				if(next_keyframe)
				{
					swap();

				}
				if(showDisplay)
				{
					cvShowImage("RGB Image 1", imgadd);
					cvWaitKey(30);

				}


			}

		}
	}

	spinner.stop();
	cvReleaseImage(&imgadd);
	cvReleaseImage(&cv_image[0]);
	cvReleaseImage(&cv_image[1]);

}


void MAP::viconCallback (const geometry_msgs::PoseStamped& viconMsg)
{
	//	if(showDisplay)
	//		std::cout<<"in viconcallback"<<std::endl;
	quat_vicon.x()=viconMsg.pose.orientation.x;
	quat_vicon.y()=viconMsg.pose.orientation.y;
	quat_vicon.z()=viconMsg.pose.orientation.z;
	quat_vicon.w()=viconMsg.pose.orientation.w;

	pos_vicon[0]=viconMsg.pose.position.x;
	pos_vicon[1]=viconMsg.pose.position.y;
	pos_vicon[2]=viconMsg.pose.position.z;


	Eigen::Quaternion<float> quat_vicon_eigen;
	quat_vicon_eigen.x()=-quat_vicon.y();
	quat_vicon_eigen.y()=-quat_vicon.z();
	quat_vicon_eigen.z()=quat_vicon.x();
	quat_vicon_eigen.w()=quat_vicon.w();


	btQuaternion quat_tmp(quat_vicon.y(),quat_vicon.z(),quat_vicon.x(),quat_vicon.w());


	btMatrix3x3 m(quat_tmp);
	//	if(showDisplay)
	//		std::cout<<"m vicon:"<<m.getRow(0)[0]<<" "<<m.getRow(0)[1]<<" "<<m.getRow(0)[2]<<std::endl
	//		<<" "<<m.getRow(1)[0]<<" "<<m.getRow(1)[1]<<" "<<m.getRow(1)[2]<<std::endl
	//		<<" "<<m.getRow(2)[0]<<" "<<m.getRow(2)[1]<<" "<<m.getRow(2)[2]<<std::endl;




	vicontransform=Eigen::Matrix4f::Identity();

	Eigen::Vector3f tmp_vec;

	tmp_vec[0]=m.getColumn(0)[0];
	tmp_vec[1]=m.getColumn(0)[1];
	tmp_vec[2]=m.getColumn(0)[2];

	vicontransform.block<3,1>(0,0)=tmp_vec;

	tmp_vec[0]=m.getColumn(1)[0];
	tmp_vec[1]=m.getColumn(1)[1];
	tmp_vec[2]=m.getColumn(1)[2];

	vicontransform.block<3,1>(0,1)=tmp_vec;

	tmp_vec[0]=m.getColumn(2)[0];
	tmp_vec[1]=m.getColumn(2)[1];
	tmp_vec[2]=m.getColumn(2)[2];

	vicontransform.block<3,1>(0,2)=tmp_vec;

	tmp_vec[0]=pos_vicon[1];
	tmp_vec[1]=pos_vicon[2];
	tmp_vec[2]=pos_vicon[0];

	vicontransform.block<3,1>(0,3)=tmp_vec;

	vicon_computed=true;



}

void MAP::commandCallback (const lcm_mavlink_ros::COMMAND& commandMsg)
{
	if(commandMsg.command==200)
		take_vicon=true;
	if(commandMsg.command==201)
		stop_mapping=true;
}


void MAP::imuCallback (const sensor_msgs::Imu& imuMsg)
{
	quat_imu.x()=imuMsg.orientation.x;
	quat_imu.y()=imuMsg.orientation.y;
	quat_imu.z()=imuMsg.orientation.z;
	quat_imu.w()=imuMsg.orientation.w;

	btQuaternion q(-imuMsg.orientation.y, -imuMsg.orientation.z,imuMsg.orientation.x,  imuMsg.orientation.w);
	btMatrix3x3 m(q);
	double Roll, Pitch, Yaw;
	m.getRPY(Roll, Pitch, Yaw);
	//	m.setRPY(Roll,Pitch,0);
	//	if(showDisplay)
	//		std::cout<<"roll"<<Roll<<"pitch"<<Pitch<<"yaw"<<Yaw<<std::endl;



	Eigen::Vector3f tmp_vec;

	tmp_vec[0]=m.getColumn(0)[0];
	tmp_vec[1]=m.getColumn(0)[1];
	tmp_vec[2]=m.getColumn(0)[2];

	imuRot.block<3,1>(0,0)=tmp_vec;

	tmp_vec[0]=m.getColumn(1)[0];
	tmp_vec[1]=m.getColumn(1)[1];
	tmp_vec[2]=m.getColumn(1)[2];

	imuRot.block<3,1>(0,1)=tmp_vec;

	tmp_vec[0]=m.getColumn(2)[0];
	tmp_vec[1]=m.getColumn(2)[1];
	tmp_vec[2]=m.getColumn(2)[2];

	imuRot.block<3,1>(0,2)=tmp_vec;

	imu_computed=true;
}

void MAP::callback(const PointCloud::ConstPtr& pointCloud_ptr,const sensor_msgs::ImageConstPtr& image_ptr)
{
	if(showDisplay)
		std::cout<<"in kinectcallback"<<std::endl;
	static sensor_msgs::CvBridge bridgeImage1;
	try
	{
		callbackCloud=*pointCloud_ptr;
		callback_image = bridgeImage1.imgMsgToCv(image_ptr, "bgr8");
	}
	catch (sensor_msgs::CvBridgeException error)
	{

		ROS_ERROR("error");

	}
	called=1;

}
void MAP::matchFeature(cv::Mat &dtors0,cv::Mat&dtors1,	vector<cv::DMatch> &matches)
{
	matcher_popcount.match(dtors0, dtors1, matches);
}

void MAP::RANSACandTransformation()
{
	actual_keyframe=KeyframeDataVector.size()-1;

	createCorrespondingPointcloud(FrameData[0],FeaturePointCloud[0],FrameData[1],FeaturePointCloud[1],correspondences,matches_popcount,showDisplay,kpts[0],kpts[1]);

	RANSAC();

	if(showDisplay)
		std::cout<<"transformOld"<<std::endl<<transformOld<<std::endl;

	FrameData[counter].Transformation=transformOld;


	if(showDisplay)
	{

		{
			if(actual_keyframe==(KeyframeDataVector.size()-1))
			{
				if(showDisplay)
					ROS_WARN("in cvline target_good");

				for(uint i=0;i<correspondences_source_good.size();i++)
				{
					cvLine( imgadd, cvPoint(kpts[0][matches_popcount[correspondences_source_good[i]].queryIdx].pt.x, kpts[0][matches_popcount[correspondences_source_good[i]].queryIdx].pt.y),
							cvPoint(kpts[1][matches_popcount[correspondences_source_good[i]].trainIdx].pt.x, 480+kpts[1][matches_popcount[correspondences_source_good[i]].trainIdx].pt.y), colors[2] );
				}
			}
		}
	}


	if(showDisplay)
		std::cout<<"correspondences_inliers.size() RANSACINLIERS"<<correspondences_inliers.size()<<std::endl;
}





void MAP::createCorrespondingPointcloud(struct FrameData& Data0,PointCloud& Cloud0,struct FrameData& Data1,PointCloud& Cloud1,std::vector<int>& correspondvector,vector<cv::DMatch>& matches,bool show,std::vector<cv::KeyPoint>& kpts0,std::vector<cv::KeyPoint>& kpts1)

{

	std::vector<int> corr_tmp;
	std::vector<int> corr_matches_tmp;

	PointCloud FeatureTmp[2];
	FeatureTmp[0].header.frame_id="/pgraph";
	FeatureTmp[1].header.frame_id="/pgraph";



	Point push_back_point;

	{
		if(showDisplay)
			ROS_WARN("in cvline matches");


		for (size_t i = 0; i < matches_popcount.size(); i++)
		{
			FeatureTmp[0].push_back(Data0.Points.points[matches_popcount[i].queryIdx]);
			corr_tmp.push_back(i);
			if(show)
			{
				cvLine( imgadd, cvPoint(kpts0[matches_popcount[i].queryIdx].pt.x, kpts0[matches_popcount[i].queryIdx].pt.y),
						cvPoint(kpts1[matches_popcount[i].trainIdx].pt.x, 480+kpts1[matches_popcount[i].trainIdx].pt.y), colors[7] );

			}
			FeatureTmp[1].push_back(Data1.Points.points[matches_popcount[i].trainIdx]);


		}

	}

	Cloud0=FeatureTmp[0];
	Cloud1=FeatureTmp[1];
	correspondvector=corr_tmp;
}

//code partially copied from http://www.cs.unc.edu/~blloyd/comp290-089/fmatrix/
void MAP::RANSAC()
{
	if(showDisplay)
		std::cout<<"inransac"<<std::endl;


	Eigen::Matrix4f frametrans=Eigen::Matrix4f::Identity();
	correspondences_source_good.clear();
	correspondences_target_good.clear();
	Eigen::Vector3f translation;


	/***Defining all the RANSAC STUFF***/


	typedef pcl::SampleConsensusModelRegistration<Point>::Ptr SampleConsensusModelRegistrationPtr;

	SampleConsensusModelRegistrationPtr model;


	model.reset (new pcl::SampleConsensusModelRegistration<Point> (FeaturePointCloud[1].makeShared (), correspondences));
	// Pass the target_indices

	model->setInputTarget (FeaturePointCloud[0].makeShared(), correspondences);
	// Create a RANSAC model

	pcl::RandomSampleConsensus<Point> sac (model, ransac_acc);
	sac.setMaxIterations (ransac_it);


	// Compute the set of inliers

	if (!sac.computeModel (5))
	{
		correspondences_source_good = correspondences;
	}
	else
	{
		std::vector<int> inliers;
		// Get the inliers
		sac.getInliers (inliers);
		int c=0;
		for (size_t i = 0; i < correspondences.size (); ++i)
		{
			if(i==inliers[c])
			{
				correspondences_source_good.push_back(correspondences[inliers[c]]);
				c++;
			}
		}

	}

	if(showDisplay)
	{
		ROS_INFO("number of correspondences  after ransac %d",correspondences_source_good.size());
	}
	ransac_inliers=correspondences_source_good.size();

	//Check whether we need next keyframe
	if(ransac_inliers<min_keyframe_inlier)
		next_keyframe=true;
	else
		next_keyframe=false;

	// Check whether we have enough correspondences
	if (ransac_inliers < min_inliers)
	{
		if(showDisplay)
			ROS_ERROR ("[pcl::::computeTransformation] Not enough correspondences found. Relax your threshold parameters.");
		compute_transform_success=false;
	}
	else
		compute_transform_success=true;




	//Berechnung der Transformation
	if(compute_transform_success)
	{
		if(transformation_at_least_once_computed)
			transformation_at_least_twice_computed=true;
		transformation_at_least_once_computed=true;

		previous_transformation=transformation_;

		pcl::estimateRigidTransformationSVD(FeaturePointCloud[1],correspondences_source_good,FeaturePointCloud[0],correspondences_source_good,transformation_);
		if(showDisplay)
		{
			std::cout<<"transformation_"<<std::endl<<transformation_<<std::endl;
			std::cout<<"KeyframeDataVector.at(actual_keyframe).Transformation"<<std::endl<<KeyframeDataVector.at(actual_keyframe).Transformation<<std::endl;
			std::cout<<"actual keyframe"<<actual_keyframe<<std::endl;
		}

		transformOld=KeyframeDataVector.at(actual_keyframe).Transformation*transformation_;



	}
	else
	{

		if(transformation_at_least_twice_computed)
		{
			frametrans=KeyframeDataVector.at(actual_keyframe).Transformation*transformation_*previous_transformation.inverse()*transformation_;
			translation=frametrans.block<3,1>(0,3);
			if(showDisplay)
				std::cout<<"frametrans"<<frametrans<<std::endl;
			transformOld=frametrans;
		}
		else
			transformOld=KeyframeDataVector.at(0).Transformation;


	}
	//	if(take_vicon)
	//	{
	//		next_keyframe=true;
	//		transformOld=vicontransform;
	//		if(showDisplay)
	//			std::cout<<"vicontransform:"<<std::endl<<vicontransform<<std::endl;
	//
	//		take_vicon=false;
	//	}

	if(showDisplay)
		std::cout<<"transformold"<<std::endl<<transformOld<<std::endl;
	correspondences_inliers=correspondences_source_good;

	if(next_keyframe)
	{
		FrameData[counter].edges_to.push_back(actual_keyframe);
		FrameData[counter].Transformation_to.push_back(transformation_);
	}

}

void MAP::swap()
{

	if(showDisplay)
		std::cout<<"next_keyframe"<<std::endl;

	FrameData[0].Descriptor=FrameData[1].Descriptor;
	FrameData[0].Points=FrameData[1].Points;
	FrameData[1].matches_backward=matches_popcount;
	FrameData[1].correspondences_backward=correspondences_inliers;
	if(showDisplay)
		std::cout<<"keyframdatavector"<<std::endl;
	KeyframeDataVector.push_back(FrameData[1]);
	if(showDisplay)
		std::cout<<"after"<<std::endl;
	actual_keyframe=KeyframeDataVector.size()-1;

	kpts[0]=kpts[1];
	dtors[0]=dtors[1];

	kinectCloud[0]=kinectCloud[1];
	cvCopy(cv_image[1],cv_image[0]);

	FrameData[0].Transformation=FrameData[1].Transformation;
	FrameData[0].edges_to=FrameData[1].edges_to;
	FrameData[0].Transformation_to=FrameData[0].Transformation_to;
	FrameData[1].edges_to.clear();
	FrameData[1].Transformation_to.clear();



	PointCloud tmpsbapointcloud;
	tmpsbapointcloud.header.frame_id="/pgraph";
	cv::Mat dtorstmp;
	dtorstmp.flags=dtors[counter].flags;
	dtorstmp.dims=dtors[counter].dims;
	dtorstmp.cols=dtors[counter].cols;
	dtorstmp.step[0]=dtors[counter].step[0];
	dtorstmp.step[1]=dtors[counter].step[1];
	//		std::cout<<"before pushing back inliers size of keyframe"<<KeyframeDataVector.size()<<std::endl;

	struct mapPoint tmp;
	std::vector<int> tmp_camera (3);
	struct FrameData OldData=KeyframeDataVector.at(KeyframeDataVector.size()-2);
	struct FrameData NewData=KeyframeDataVector.at(KeyframeDataVector.size()-1);
	struct FrameData finalMAPtmp;


	if(KeyframeDataVector.size()==2)
		for(uint q=0;q<correspondences_inliers.size();q++)
		{
			tmp.cameras.clear();

			tmp.PointXYZ=OldData.Points.points.at(NewData.matches_backward.at(NewData.correspondences_backward.at(q)).queryIdx);
			tmp_camera[0]=0;
			tmp_camera[1]=OldData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(q)).queryIdx).pt.x;
			tmp_camera[2]=OldData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(q)).queryIdx).pt.y;
			dtorstmp.push_back(OldData.Descriptor.row(NewData.matches_backward.at(NewData.correspondences_backward.at(q)).queryIdx));

			circle.x=tmp_camera[1];
			circle.y=tmp_camera[2];
			cvCircle( imgadd, circle, 3, colors[5], 3, 8, 0 );

			tmp.cameras.push_back(tmp_camera);
			tmp_camera[0]=1;
			tmp_camera[1]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(q)).trainIdx).pt.x;
			tmp_camera[2]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(q)).trainIdx).pt.y;
			circle.x=tmp_camera[1];
			circle.y=tmp_camera[2]+480;
			cvCircle( imgadd, circle, 3, colors[5], 3, 8, 0 );
			tmp.cameras.push_back(tmp_camera);
			tmp.identifier=NewData.matches_backward.at(NewData.correspondences_backward.at(q)).trainIdx;
			SBAPoint.push_back(tmp);
			tmpsbapointcloud.push_back(tmp.PointXYZ);
		}


	int sizeofSBAPoint=SBAPoint.size();
	std::vector<int> updated_points(sizeofSBAPoint,0);

	if(KeyframeDataVector.size()>2)
	{

		for(uint w=0;w<NewData.correspondences_backward.size();w++)
		{
			bool new_inlier=true;
			for(uint q=0;q<sizeofSBAPoint;q++)
				if(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx==SBAPoint.at(q).identifier)
				{
					if(SBAPoint.at(q).cameras.at(SBAPoint.at(q).cameras.size()-1).at(0)!=KeyframeDataVector.size()-1)
					{
						tmp.cameras.clear();
						tmp_camera[0]=KeyframeDataVector.size()-1;
						tmp_camera[1]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx).pt.x;
						tmp_camera[2]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx).pt.y;
						circle.x=tmp_camera[1];
						circle.y=tmp_camera[2]+480;
						cvCircle( imgadd, circle, 3, colors[5], 3, 8, 0 );
						SBAPoint.at(q).cameras.push_back(tmp_camera);
						SBAPoint.at(q).identifier=NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx;
						updated_points.at(q)=1;
					}
					new_inlier=false;

				}

			if(new_inlier)
			{
				tmp.cameras.clear();
				tmp_camera[0]=KeyframeDataVector.size()-2;

				tmp_camera[1]=OldData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx).pt.x;
				tmp_camera[2]=OldData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx).pt.y;
				circle.x=tmp_camera[1];
				circle.y=tmp_camera[2];
				cvCircle( imgadd, circle, 3, colors[5], 3, 8, 0 );
				tmp.cameras.push_back(tmp_camera);
				tmp_camera[0]=KeyframeDataVector.size()-1;

				tmp_camera[1]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx).pt.x;
				tmp_camera[2]=NewData.Keypoints.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx).pt.y;
				circle.x=tmp_camera[1];
				circle.y=tmp_camera[2]+480;
				cvCircle( imgadd, circle, 3, colors[5], 3, 8, 0 );
				tmp.cameras.push_back(tmp_camera);
				tmp.PointXYZ=OldData.Points.points.at(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx);
				tmp.identifier=NewData.matches_backward.at(NewData.correspondences_backward.at(w)).trainIdx;

				SBAPoint.push_back(tmp);
				tmpsbapointcloud.push_back(tmp.PointXYZ);
				dtorstmp.push_back(OldData.Descriptor.row(NewData.matches_backward.at(NewData.correspondences_backward.at(w)).queryIdx));


			}



		}



		if(showDisplay)
			std::cout<<"size of sba"<<SBAPoint.size()<<std::endl;

		for(uint a=0;a<updated_points.size();a++)
			if(updated_points.at(a)==0)
			{
				SBAPoint.at(a).identifier=100000;
			}

	}
	finalMAPtmp.Descriptor=dtorstmp;
	finalMAPtmp.Points=tmpsbapointcloud;

	finalMAP.push_back(finalMAPtmp);


	if(stop_mapping)
	{
		refinedMap.Points.header.frame_id="/pgraph";
		cv::Mat dtorstmp;
		dtorstmp.flags=dtors[counter].flags;
		dtorstmp.dims=dtors[counter].dims;
		dtorstmp.cols=dtors[counter].cols;
		dtorstmp.step[0]=dtors[counter].step[0];
		dtorstmp.step[1]=dtors[counter].step[1];

		std::cout<<"Keyframedatatvector.at0.transfor"<<KeyframeDataVector.at(0).Transformation<<std::endl;

		refineMapWithTORO(&KeyframeDataVector);
		PointCloud tmp2;
		tmp2.header.frame_id="/pgraph";

		PointCloud tmp3;
		tmp3.header.frame_id="/pgraph";

		for(uint i=0;i<finalMAP.size();i++)
		{
			for(uint k=0;k<finalMAP.at(i).Points.size();k++)
				dtorstmp.push_back(finalMAP.at(i).Descriptor.row(k));
			PointCloud tmp;
			tmp.header.frame_id="/pgraph";

			if(showDisplay)
				std::cout<<"finalMAP.at(i).Transformation"<<std::endl<<finalMAP.at(i).Transformation<<std::endl;
			pcl::transformPointCloud(finalMAP.at(i).Points,tmp,finalMAP.at(i).Transformation);
			tmp2+=tmp;

		}
		refinedMap.Points=tmp2;
		refinedMap.Descriptor=dtorstmp;

		for(uint i=0;i<KeyframeDataVector.size();i++)
		{
			PointCloud tmp;
			tmp.header.frame_id="/pgraph";

			pcl::transformPointCloud(KeyframeDataVector.at(i).Points,tmp,KeyframeDataVector.at(i).Transformation);
			tmp3+=tmp;

		}

		//store the map in a file
		std::ofstream mapfile;
		mapfile.open(path.c_str(),ios::binary);
		size_t size=refinedMap.Points.size();
		mapfile.write((char *)(&size),sizeof(size));
		for(uint i=0;i<refinedMap.Points.size();i++)
		{
			mapfile.write((char *)(&refinedMap.Points.points.at(i).x),sizeof(refinedMap.Points.points.at(i).x));
			mapfile.write((char *)(&refinedMap.Points.points.at(i).y),sizeof(refinedMap.Points.points.at(i).y));
			mapfile.write((char *)(&refinedMap.Points.points.at(i).z),sizeof(refinedMap.Points.points.at(i).z));
		}
		std::cout<<"row"<<refinedMap.Descriptor.rows<<std::endl;
		std::cout<<"cols"<<refinedMap.Descriptor.cols<<std::endl;

		mapfile.write((char *)(&refinedMap.Descriptor.rows),sizeof(refinedMap.Descriptor.rows));
		mapfile.write((char *)(&refinedMap.Descriptor.cols),sizeof(refinedMap.Descriptor.cols));
		std::cout<<"type:"<<refinedMap.Descriptor.type()<<std::endl;
		for(int y=0;y<refinedMap.Descriptor.rows;y++)
			for(int x=0;x<refinedMap.Descriptor.cols;x++)
			{
				uchar tmp=(*refinedMap.Descriptor.row(y).col(x).data);
				mapfile.write((char *)(&tmp), sizeof(tmp));
			}
		mapfile.close();

		std::cout<<"!!!MAP CREATED!!!"<<std::endl;

		while(ros::ok())
		{
			if(showDisplay){
				std::cout<<"size of pointcloud:"<<refinedMap.Points.size()<<std::endl;
				std::cout<<"size of descriptor:"<<refinedMap.Descriptor.rows<<std::endl;
				if(showDisplay)
					std::cout<<"publishing old points"<<std::endl;
				KeyFramePoints.publish(tmp3);
				cvWaitKey(5000);
				if(showDisplay)
					std::cout<<"publishing new points before copy"<<std::endl;
				KeyFramePoints.publish(refinedMap.Points);
			}
			cvWaitKey(5000);
		}
		return;
	}

}


void MAP::refineMapWithTORO(std::vector<struct FrameData>* map)
{
	for(uint i=0;i<map->size();i++)
	{

		findConnectionsBetweenKeyframes(i,map->at(i), *map);

	}
	AISNavigation::TreeOptimizer3 pg;

	std::cout<<"first transformation before toro:"<<map->at(0).Transformation<<std::endl;
	for(uint i=0; i<map->size();i++)
	{
		Eigen::Quaternion<float> tmp_quat(map->at(i).Transformation.block<3,3>(0,0));
		btQuaternion tmp_btquat(tmp_quat.x(),tmp_quat.y(),tmp_quat.z(),tmp_quat.w());
		btMatrix3x3 tmp_mat(tmp_btquat);
		Eigen::Vector3f tmp_vec;
		tmp_vec=map->at(i).Transformation.block<3,1>(0,3);

		float x=tmp_vec[0];
		float y=tmp_vec[1];
		float z=tmp_vec[2];
		double R,P,Y;
		tmp_mat.getRPY(R,P,Y);
		AISNavigation::TreePoseGraph3::Vertex V;
		AISNavigation::TreePoseGraph3::Pose pose;

		pose.x()=x;
		pose.y()=y;
		pose.z()=z;
		pose.roll()=(float)R;
		pose.pitch()=(float)P;
		pose.yaw()=(float)Y;

		pg.addVertex(i,pose);

		if(showDisplay)
		{
			printf("added VERTEX3 %d %f %f %f %f %f %f\n",i,x,y,z,(float)R,(float)P,(float)Y);
		}
	}
	for(uint i=0;i<map->size();i++)
		for(uint w=0;w<map->at(i).edges_to.size();w++)
		{
			Eigen::Quaternion<float> tmp_quat(map->at(i).Transformation_to.at(w).block<3,3>(0,0));
			btQuaternion tmp_btquat(tmp_quat.x(),tmp_quat.y(),tmp_quat.z(),tmp_quat.w());
			btMatrix3x3 tmp_mat(tmp_btquat);
			Eigen::Vector3f tmp_vec;
			tmp_vec=map->at(i).Transformation_to.at(w).block<3,1>(0,3);

			float x=tmp_vec[0];
			float y=tmp_vec[1];
			float z=tmp_vec[2];
			double R,P,Y;
			tmp_mat.getRPY(R,P,Y);

			AISNavigation::TreePoseGraph3::Vertex* v1=pg.vertex(map->at(i).edges_to.at(w));
			AISNavigation::TreePoseGraph3::Vertex* v2=pg.vertex(i);
			AISNavigation::TreePoseGraph3::Pose pose;
			pose.x()=x;
			pose.y()=y;
			pose.z()=z;
			pose.roll()=(float)R;
			pose.pitch()=(float)P;
			pose.yaw()=(float)Y;

			AISNavigation::TreePoseGraph3::Transformation t(pose);
			AISNavigation::TreePoseGraph3::InformationMatrix m;
			m=DMatrix<double>::I(6);
			pg.addEdge(v1,v2,t,m);
			if(showDisplay)
			{
				printf ("ADDIng edgeEDGE3 %d %d %f %f %f %f %f %f\n",map->at(i).edges_to.at(w),i,x,y,z,(float)R,(float)P,(float)Y);
			}
		}


	int  iterations=100;

	bool adaptiveRestart=false;
	int  verboseLevel=0;
	bool ignorePreconditioner=false;

	AISNavigation::TreeOptimizer3::EdgeCompareMode compareMode=AISNavigation::EVComparator<AISNavigation::TreeOptimizer3::Edge*>::CompareLevel;

	pg.verboseLevel=verboseLevel;

	pg.restartOnDivergence=adaptiveRestart;


	pg.buildSimpleTree();
	pg.initializeOnTree();
	pg.initializeTreeParameters();
	pg.initializeOptimization(compareMode);


	for (int i=0; i<iterations; i++)
	{
		pg.iterate(0,ignorePreconditioner);
		double mte, mre, are, ate;
		double error=pg.error(&mre, &mte, &are, &ate);
		if(showDisplay)
			std::cout<<"error: "<<error<<std::endl;
	}

	for (AISNavigation::TreePoseGraph3::VertexMap::iterator it=pg.vertices.begin(); it!=pg.vertices.end(); it++){


		AISNavigation::TreePoseGraph3::Vertex* v=it->second;
		Eigen::Vector3f tmp_vec;
		v->pose=v->transformation.toPoseType();
		tmp_vec[0]=v->pose.x();
		tmp_vec[1]=v->pose.y();
		tmp_vec[2]=v->pose.z();
		if(v->id<finalMAP.size())
		{
			finalMAP.at(v->id).Transformation=Eigen::Matrix4f::Identity();
			finalMAP.at(v->id).Transformation.block<3,1>(0,3)=tmp_vec;
			btMatrix3x3 m;
			m.setRPY((double)v->pose.roll(),(double)v->pose.pitch(),(double)v->pose.yaw());
			tmp_vec[0]=m.getColumn(0)[0];
			tmp_vec[1]=m.getColumn(0)[1];
			tmp_vec[2]=m.getColumn(0)[2];
			finalMAP.at(v->id).Transformation.block<3,1>(0,0)=tmp_vec;
			tmp_vec[0]=m.getColumn(1)[0];
			tmp_vec[1]=m.getColumn(1)[1];
			tmp_vec[2]=m.getColumn(1)[2];
			finalMAP.at(v->id).Transformation.block<3,1>(0,1)=tmp_vec;
			tmp_vec[0]=m.getColumn(2)[0];
			tmp_vec[1]=m.getColumn(2)[1];
			tmp_vec[2]=m.getColumn(2)[2];
			finalMAP.at(v->id).Transformation.block<3,1>(0,2)=tmp_vec;
//			finalMAP.at(v->id).Transformation=finalMAP.at(v->id).Transformation;
		}
		if(showDisplay)
			printf("new VERTEX3 %d %f %f %f %f %f %f\n",v->id,v->pose.x(),v->pose.y(),v->pose.z(),(float)v->pose.roll(),(float)v->pose.pitch(),(float)v->pose.yaw());



	}
	Eigen::Matrix4f rot=systemRot*finalMAP.at(0).Transformation.inverse();
	std::cout<<"rot"<<std::endl<<rot<<std::endl;
	std::cout<<"first transformation after torobefore transform:"<<std::endl<<finalMAP.at(0).Transformation<<std::endl;
	std::cout<<"systemrot:"<<std::endl<<systemRot<<std::endl;
//

	for(int k=0;k<finalMAP.size();k++)
		finalMAP.at(k).Transformation=rot*finalMAP.at(k).Transformation;
	std::cout<<"first transformation after toroafter transfrom:"<<std::endl<<finalMAP.at(0).Transformation<<std::endl;


}

void MAP::findConnectionsBetweenKeyframes(int number,struct FrameData& from, std::vector<struct FrameData>& to)
{
	for(uint c=0;c<to.size();c++)
	{
		if(number!=c)
		{
			Eigen::Vector3f tmp_dist;
			vector<cv::DMatch> matches;
			std::vector<int> corr_Keyframe;
			std::vector<int> corr_Keyframe_inliers;
			std::vector<int> corr_Keyframe_inliers_tmp;
			std::vector<int> corr_Keyframe_outliers;
			PointCloud Feature_Keyframe[2];
			Feature_Keyframe[0].header.frame_id="/pgraph";
			Feature_Keyframe[1].header.frame_id="/pgraph";



			matchFeature(to.at(c).Descriptor,from.Descriptor,matches);

			for (size_t i = 0; i < matches.size(); i++)
			{
				Feature_Keyframe[0].push_back(to.at(c).Points.points[matches[i].queryIdx]);
				corr_Keyframe.push_back(i);
				Feature_Keyframe[1].push_back(from.Points.points[matches[i].trainIdx]);

			}

			//RANSAC


			typedef pcl::SampleConsensusModelRegistration<Point>::Ptr SampleConsensusModelRegistrationPtr;

			SampleConsensusModelRegistrationPtr model;


			model.reset (new pcl::SampleConsensusModelRegistration<Point> (Feature_Keyframe[0].makeShared (), corr_Keyframe));
			// Pass the target_indices


			model->setInputTarget (Feature_Keyframe[1].makeShared(), corr_Keyframe);
			// Create a RANSAC model

			pcl::RandomSampleConsensus<Point> sac (model, ransac_acc);
			sac.setMaxIterations (ransac_it);


			// Compute the set of inliers

			if (!sac.computeModel (5))
			{
				corr_Keyframe_inliers = corr_Keyframe;
			}
			else
			{
				std::vector<int> inliers;
				// Get the inliers
				sac.getInliers (inliers);
				corr_Keyframe_inliers.resize (inliers.size ());
				// Copy just the inliers
				for (size_t i = 0; i < inliers.size (); ++i)
				{
					corr_Keyframe_inliers[i] = corr_Keyframe[inliers[i]];
				}

			}

			if(showDisplay)
			{
				if(showDisplay)
					ROS_ERROR("number of correspondences  after ransac with nearest keyframe %d",corr_Keyframe_inliers.size());
			}
			ransac_inliers=corr_Keyframe_inliers.size();


			if(ransac_inliers>min_keyframe_redection_inliers)
			{
				pcl::estimateRigidTransformationSVD(Feature_Keyframe[1],corr_Keyframe_inliers,Feature_Keyframe[0],corr_Keyframe_inliers,transformation_);

				from.edges_to.push_back(c);
				from.Transformation_to.push_back(transformation_);

			}
		}
	}
}
