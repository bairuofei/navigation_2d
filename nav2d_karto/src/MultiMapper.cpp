#include <sstream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav2d_msgs/RobotPose.h>
#include <nav2d_karto/MultiMapper.h>
#include <geometry_msgs/PoseStamped.h>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))


MultiMapper::MultiMapper()
{
	// Get parameters from the ROS parameter server
	ros::NodeHandle robotNode;
	robotNode.param("robot_id", mRobotID, 1);
	robotNode.param("laser_frame", mLaserFrame, std::string("laser"));
	robotNode.param("robot_frame", mRobotFrame, std::string("robot"));
	robotNode.param("odometry_frame", mOdometryFrame, std::string("odometry_base"));
	robotNode.param("offset_frame", mOffsetFrame, std::string("odometry_offset"));
	robotNode.param("map_frame", mMapFrame, std::string("map"));

	ros::NodeHandle mapperNode("~/");
	mapperNode.param("grid_resolution", mMapResolution, 0.05);
	mapperNode.param("range_threshold", mRangeThreshold, 30.0);
	mapperNode.param("map_update_rate", mMapUpdateRate, 1);
	mapperNode.param("publish_pose_graph", mPublishPoseGraph, true);
	mapperNode.param("max_covariance", mMaxCovariance, 0.05);
	mapperNode.param("min_map_size", mMinMapSize, 50);

	// Apply tf_prefix to all used frame-id's
	mLaserFrame = mTransformListener.resolve(mLaserFrame);
	mRobotFrame = mTransformListener.resolve(mRobotFrame);
	mOdometryFrame = mTransformListener.resolve(mOdometryFrame);
	mOffsetFrame = mTransformListener.resolve(mOffsetFrame);
	mMapFrame = mTransformListener.resolve(mMapFrame);

	// Initialize Publisher/Subscribers
	mScanSubscriber = robotNode.subscribe("karto_in", 100, &MultiMapper::receiveLocalizedScan, this);
	mScanPublisher = robotNode.advertise<nav2d_msgs::LocalizedScan>("karto_out", 100, true);
	mMapServer = robotNode.advertiseService("get_map", &MultiMapper::getMap, this);
	mMapPublisher = robotNode.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	mLaserSubscriber = robotNode.subscribe("scan", 100, &MultiMapper::receiveLaserScan, this);
	mInitialPoseSubscriber = robotNode.subscribe("initialpose", 1, &MultiMapper::receiveInitialPose, this);
	mOtherRobotsPublisher = robotNode.advertise<nav2d_msgs::RobotPose>("others", 10, true);
	mPathPublisher = robotNode.advertise<nav_msgs::Path>("slam_path", 10, true);

	mVerticesPublisher = mapperNode.advertise<visualization_msgs::Marker>("vertices", 1, true);
	mClosureEdgesPublisher = mapperNode.advertise<visualization_msgs::Marker>("closure_edges", 1, true);
	mOdomEdgesPublisher = mapperNode.advertise<visualization_msgs::Marker>("odom_edges", 1, true);
	mColoredEdgePublisher = mapperNode.advertise<visualization_msgs::MarkerArray>("colored_edges", 1, true);
	mPosePublisher = robotNode.advertise<geometry_msgs::PoseStamped>("localization_result", 1, true);

	mPoseGraphIdxSub = robotNode.subscribe("pose_graph_idx", 5, &MultiMapper::handlePoseGraphIdx, this);

	// Pub covariance
	mCovPublisher = robotNode.advertise<cpp_solver::PoseGraph>("slam_pose_graph", 2);

	// Initialize KARTO-Mapper
	mMapper = new karto::OpenMapper(true);
	
	double param_d;
	bool param_b;
	int param_i;
	
	if(mapperNode.getParam("UseScanMatching", param_b))
		mMapper->SetParameters("UseScanMatching", param_b);
		
	if(mapperNode.getParam("UseScanBarycenter", param_b))
		mMapper->SetParameters("UseScanBarycenter", param_b);
		
	if(mapperNode.getParam("MinimumTravelDistance", param_d))
		mMapper->SetParameters("MinimumTravelDistance", param_d);
	
	if(mapperNode.getParam("MinimumTravelHeading", param_d))
		mMapper->SetParameters("MinimumTravelHeading", param_d);
		
	if(mapperNode.getParam("ScanBufferSize", param_i))
		mMapper->SetParameters("ScanBufferSize", param_i);
		
	if(mapperNode.getParam("ScanBufferMaximumScanDistance", param_d))
		mMapper->SetParameters("ScanBufferMaximumScanDistance", param_d);
		
	if(mapperNode.getParam("UseResponseExpansion", param_b))
		mMapper->SetParameters("UseResponseExpansion", param_b);
		
	if(mapperNode.getParam("DistanceVariancePenalty", param_d))
		mMapper->SetParameters("DistanceVariancePenalty", param_d);
		
	if(mapperNode.getParam("MinimumDistancePenalty", param_d))
		mMapper->SetParameters("MinimumDistancePenalty", param_d);
		
	if(mapperNode.getParam("AngleVariancePenalty", param_d))
		mMapper->SetParameters("AngleVariancePenalty", param_d);
		
	if(mapperNode.getParam("MinimumAnglePenalty", param_d))
		mMapper->SetParameters("MinimumAnglePenalty", param_d);
		
	if(mapperNode.getParam("LinkMatchMinimumResponseFine", param_d))
		mMapper->SetParameters("LinkMatchMinimumResponseFine", param_d);
		
	if(mapperNode.getParam("LinkScanMaximumDistance", param_d))
		mMapper->SetParameters("LinkScanMaximumDistance", param_d);
		
	if(mapperNode.getParam("CorrelationSearchSpaceDimension", param_d))
		mMapper->SetParameters("CorrelationSearchSpaceDimension", param_d);
		
	if(mapperNode.getParam("CorrelationSearchSpaceResolution", param_d))
		mMapper->SetParameters("CorrelationSearchSpaceResolution", param_d);
		
	if(mapperNode.getParam("CorrelationSearchSpaceSmearDeviation", param_d))
		mMapper->SetParameters("CorrelationSearchSpaceSmearDeviation", param_d);
		
	if(mapperNode.getParam("CoarseSearchAngleOffset", param_d))
		mMapper->SetParameters("CoarseSearchAngleOffset", param_d);
		
	if(mapperNode.getParam("FineSearchAngleOffset", param_d))
		mMapper->SetParameters("FineSearchAngleOffset", param_d);
		
	if(mapperNode.getParam("CoarseAngleResolution", param_d))
		mMapper->SetParameters("CoarseAngleResolution", param_d);
		
	if(mapperNode.getParam("LoopSearchSpaceDimension", param_d))
		mMapper->SetParameters("LoopSearchSpaceDimension", param_d);
		
	if(mapperNode.getParam("LoopSearchSpaceResolution", param_d))
		mMapper->SetParameters("LoopSearchSpaceResolution", param_d);
		
	if(mapperNode.getParam("LoopSearchSpaceSmearDeviation", param_d))
		mMapper->SetParameters("LoopSearchSpaceSmearDeviation", param_d);
		
	if(mapperNode.getParam("LoopSearchMaximumDistance", param_d))
		mMapper->SetParameters("LoopSearchMaximumDistance", param_d);
		
	if(mapperNode.getParam("LoopMatchMinimumChainSize", param_i))
		mMapper->SetParameters("LoopMatchMinimumChainSize", param_i);
		
	if(mapperNode.getParam("LoopMatchMaximumVarianceCoarse", param_d))
		mMapper->SetParameters("LoopMatchMaximumVarianceCoarse", param_d);
		
	if(mapperNode.getParam("LoopMatchMinimumResponseCoarse", param_d))
		mMapper->SetParameters("LoopMatchMinimumResponseCoarse", param_d);
		
	if(mapperNode.getParam("LoopMatchMinimumResponseFine", param_d))
		mMapper->SetParameters("LoopMatchMinimumResponseFine", param_d);
	
	mMapper->Message += karto::delegate(this, &MultiMapper::onMessage);
	
	mLaser = NULL;
	
	// Initialize Variables
	mMapToOdometry.setIdentity();
	mOdometryOffset.setIdentity();
	mNodesAdded = 0;
	mMapChanged = true;
	mLastMapUpdate = ros::WallTime(0);

	mVertexReceiveIdx = 0;
	mEdgeReceiveIdx = 0;
	
	if(mRobotID == 1)
	{
		// I am the number one, so start mapping right away.
		mState = ST_MAPPING;
		ROS_INFO("Inititialized robot 1, starting to map now.");
		mSelfLocalizer = NULL;
		
		geometry_msgs::PoseStamped locResult;
		locResult.header.stamp = ros::Time::now();
		locResult.header.frame_id = mMapFrame.c_str();
		locResult.pose.position.x = 0;
		locResult.pose.position.y = 0;
		locResult.pose.position.z = 0;
		locResult.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		mPosePublisher.publish(locResult);
	}else
	{
		// I am not number one, so wait to receive a map from number one.
		mState = ST_WAITING_FOR_MAP;
		ROS_INFO("Initialized robot %d, waiting for map from robot 1 now.", mRobotID);
		mSelfLocalizer = new SelfLocalizer();
	}
	std::cout << "Create multimapper!!" << std::endl;
}

MultiMapper::~MultiMapper()
{

}

/**
 * @brief Update pose graph publish index.
 * 
 * @param msg 
 */
void MultiMapper::handlePoseGraphIdx(const std_msgs::Int32MultiArray::ConstPtr& msg){
	mVertexReceiveIdx = msg->data[0];
	mEdgeReceiveIdx = msg->data[1];
	// ROS_INFO("pose graph index upadte: vertex %d, edge %d", mVertexReceiveIdx, mEdgeReceiveIdx);
	return;
}

void MultiMapper::setScanSolver(karto::ScanSolver* scanSolver)
{
	mMapper->SetScanSolver(scanSolver);
}

void MultiMapper::setRobotPose(double x, double y, double yaw)
{
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, 0));
	transform.setRotation(tf::createQuaternionFromYaw(yaw));
	transform = transform.inverse();
	
	tf::Stamped<tf::Pose> pose_in, pose_out;
	pose_in.setData(transform);
	pose_in.frame_id_ = mRobotFrame;
	pose_in.stamp_ = ros::Time(0);
	mTransformListener.transformPose(mOdometryFrame, pose_in, pose_out);
	
	transform = pose_out;
	mOdometryOffset = transform.inverse();

	if(mSelfLocalizer)
	{
		delete mSelfLocalizer;
		mSelfLocalizer = NULL;
	}
	
	// Publish the new pose (to inform other nodes, that we are localized now)
	geometry_msgs::PoseStamped locResult;
	locResult.header.stamp = ros::Time::now();
	locResult.header.frame_id = mMapFrame.c_str();
	locResult.pose.position.x = x;
	locResult.pose.position.y = y;
	locResult.pose.position.z = 0;
	locResult.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
	mPosePublisher.publish(locResult);
	
	// Publish via tf
	mState = ST_MAPPING;
	publishTransform();
}

karto::LocalizedRangeScan* MultiMapper::createFromRosMessage(const sensor_msgs::LaserScan& scan, const karto::Identifier& robot)
{
	// Implementing REP 117: Informational Distance Measurements
	// http://www.ros.org/reps/rep-0117.html
	karto::RangeReadingsList readings;
	std::vector<float>::const_iterator it;
	for(it = scan.ranges.begin(); it != scan.ranges.end(); it++)
	{
		if(*it >= scan.range_min && *it <= scan.range_max)
		{
			// This is a valid measurement.
			readings.Add(*it);
		}else if( !std::isfinite(*it) && *it < 0)
		{
			// Object too close to measure.
			readings.Add(scan.range_max);
		}else if( !std::isfinite(*it) && *it > 0)
		{
			// No objects detected in range.
			readings.Add(scan.range_max);
		}else if( std::isnan(*it) )
		{
			// This is an erroneous, invalid, or missing measurement.
			ROS_WARN_THROTTLE(1,"Laser scan contains nan-values!");
			readings.Add(scan.range_max);
		}else
		{
			// The sensor reported these measurements as valid, but they are
			// discarded per the limits defined by minimum_range and maximum_range.
			ROS_WARN_THROTTLE(1, "Laser reading not between range_min and range_max!");
			readings.Add(scan.range_max);
		}
	}
	return new karto::LocalizedRangeScan(robot, readings);
}

void MultiMapper::receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// Ignore own readings until map has been received
	if(mState == ST_WAITING_FOR_MAP)
	{
		return;
	}
	
	if(!mLaser)
	{
		// Create a laser range finder device and copy in data from the first scan
		char name[10];
		sprintf(name, "robot_%d", mRobotID);

		// Add the laser to the mapper
		try
		{
			mLaser = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name);		
			mLaser->SetMinimumRange(scan->range_min);
			mLaser->SetMaximumRange(scan->range_max);
			mLaser->SetMinimumAngle(scan->angle_min);
			mLaser->SetMaximumAngle(scan->angle_max);
			mLaser->SetAngularResolution(scan->angle_increment);
			mLaser->SetRangeThreshold(mRangeThreshold);
			mMapper->Process(mLaser);
		}
		catch(karto::Exception e)
		{
			ROS_ERROR("Could not add new Laser to Mapper: %s", e.GetErrorMessage().ToCString());
			return;
		}
	}
	
	if(mState == ST_LOCALIZING)  // Start localization
	{
		mSelfLocalizer->process(scan);
		if(mSelfLocalizer->getCovariance() < mMaxCovariance)
		{
			// Localization finished, kill the localizer and start mapping
			ROS_INFO("Localization finished on robot %d, now starting to map.", mRobotID);
			tf::Transform p = mSelfLocalizer->getBestPose();
			setRobotPose(p.getOrigin().getX(), p.getOrigin().getY(), tf::getYaw(p.getRotation()));
		}
	}else 
	if(mState == ST_MAPPING)  // Start mapping
	{
		// get the odometry pose from tf
		tf::StampedTransform tfPose;
		try
		{
			mTransformListener.lookupTransform(mOffsetFrame, mLaserFrame, scan->header.stamp, tfPose);  // Find tf at the same time stamp
		}
		catch(tf::TransformException e)
		{
			try
			{
				mTransformListener.lookupTransform(mOffsetFrame, mLaserFrame, ros::Time(0), tfPose);  // otherwise find the latest available transform
			}
			catch(tf::TransformException e)
			{
				ROS_WARN("Failed to compute odometry pose, skipping scan (%s)", e.what());
				return;
			}
		}
		karto::Pose2 kartoPose = karto::Pose2(tfPose.getOrigin().x(), tfPose.getOrigin().y(), tf::getYaw(tfPose.getRotation()));
		
		// create localized laser scan
		karto::LocalizedLaserScanPtr laserScan = createFromRosMessage(*scan, mLaser->GetIdentifier());  // * here take data from pointer "scan"
		laserScan->SetOdometricPose(kartoPose);
		laserScan->SetCorrectedPose(kartoPose);
		
		bool success;
		try
		{	
			// TODO: Publish newly added nodes and edges
			success = mMapper->Process(laserScan);  // Here the laserScan is registered into current map
		}
		catch(karto::Exception e)
		{
			ROS_ERROR("%s", e.GetErrorMessage().ToCString());
			success = false;
		}
		
		if(success)
		{	
			// Current laser scan has been added into pose graph, edges are also added.

			// Compute the map->odom transform
			karto::Pose2 corrected_pose = laserScan->GetCorrectedPose();
			tf::Pose map_in_robot(tf::createQuaternionFromYaw(corrected_pose.GetHeading()), 
								  tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0));
			map_in_robot = map_in_robot.inverse();
			tf::Stamped<tf::Pose> map_in_odom;
			bool ok = true;
			try
			{
				mTransformListener.transformPose(mOffsetFrame, tf::Stamped<tf::Pose>(map_in_robot, ros::Time(0) /*scan->header.stamp*/, mLaserFrame), map_in_odom);
			}
			catch(tf::TransformException e)
			{
				ROS_WARN("Transform from %s to %s failed! (%s)", mLaserFrame.c_str(), mOffsetFrame.c_str(), e.what());
				ok = false;
			}
			if(ok) 
			{
				mMapToOdometry = tf::Transform(tf::Quaternion( map_in_odom.getRotation() ), tf::Point(map_in_odom.getOrigin() ) ).inverse();
				tf::Vector3 v = mMapToOdometry.getOrigin();
				v.setZ(0);
				mMapToOdometry.setOrigin(v);
			}
			mNodesAdded++;
			mMapChanged = true;

			// TODO: Publish newly added vertex and edges
			/*  Both new vertex and edges are added sequently into a list.
				m_pGraph 
				karto::MapperGraph::VertexList vertices = mMapper->GetGraph()->GetVertices();
				karto::MapperGraph::EdgeList edges = mMapper->GetGraph()->GetEdges();

			*/

			ros::WallDuration d = ros::WallTime::now() - mLastMapUpdate;
			if(mMapUpdateRate > 0 && d.toSec() > mMapUpdateRate)
			{
				sendMap();   // Here will send slam pose path to /slam_path
				publishPoseGraph();  // Publish vertices and edges of pose graph.
			}

			// Send the scan to the other robots via com-layer (DDS)
			ROS_DEBUG("Robot %d: Sending scan (uniqueID: %d, Sensor: %s, stateID: %d)", mRobotID, laserScan->GetUniqueId(), laserScan->GetSensorIdentifier().ToString().ToCString(), laserScan->GetStateId());
			sendLocalizedScan(scan, laserScan->GetOdometricPose());
			
			// Publish via extra topic
			nav2d_msgs::RobotPose other;
			// other.header.stamp = ros::Time::now();
			other.header.stamp = scan->header.stamp;
			other.header.frame_id = mMapFrame;
			other.robot_id = mRobotID;
			other.pose.x = laserScan->GetCorrectedPose().GetX();
			other.pose.y = laserScan->GetCorrectedPose().GetY();
			other.pose.theta = laserScan->GetCorrectedPose().GetHeading();
			mOtherRobotsPublisher.publish(other);
		}
	}
}

void MultiMapper::publishPoseGraph(){

	// Store edges and covariance
	// node1_id, node2_id, upper triangle of the covariance matrix (6 elements)
	// 8 data for an edge

	cpp_solver::PoseGraph poseGraph;

	poseGraph.vertex_start_idx = this->mVertexReceiveIdx;
	karto::MapperGraph::VertexList vertices = mMapper->GetGraph()->GetVertices();
	for(int i = mVertexReceiveIdx; i < vertices.Size(); i++) {
		poseGraph.vertices.push_back(i);
		poseGraph.vertex_x.push_back(vertices[i]->GetVertexObject()->GetCorrectedPose().GetX());
		poseGraph.vertex_y.push_back(vertices[i]->GetVertexObject()->GetCorrectedPose().GetY());
		poseGraph.vertex_theta.push_back(vertices[i]->GetVertexObject()->GetCorrectedPose().GetHeading());
	}

	poseGraph.edge_start_idx = this->mEdgeReceiveIdx;
	karto::MapperGraph::EdgeList edges = mMapper->GetGraph()->GetEdges();
	for(int i = mEdgeReceiveIdx; i < edges.Size(); i++) {
		int id1 = -1, id2 = -1;
		for(int j = 0; j < vertices.Size(); j++){
			if(vertices[j] == edges[i]->GetSource())
				id1 = j;
			if(vertices[j] == edges[i]->GetTarget())
				id2 = j;
			if(id1 != -1 && id2 != -1)
				break;
		}
		poseGraph.edges_start.push_back(id1);
		poseGraph.edges_end.push_back(id2);

		std::vector<double> cov = edges[i]->GetCovarianceVector();
		std::stringstream ss;
		ss << cov[0] << " " << cov[1] << " " << cov[2] << " "
							<< cov[4] << " " << cov[5] << " "
											 << cov[8];						
		poseGraph.covariance.push_back(ss.str());
	}
	mCovPublisher.publish(poseGraph);
	return;
}


bool MultiMapper::getMap(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res)
{
	if(mState == ST_WAITING_FOR_MAP && mNodesAdded < mMinMapSize)
	{
		ROS_INFO("Still waiting for map from robot 1.");
		return false;
	}
	
	if(sendMap())
	{
		res.map = mGridMap;
		return true;
	}else
	{
		ROS_WARN("Serving map request failed!");
		return false;
	}
}

bool MultiMapper::sendMap()
{
	if(!updateMap()) return false;
	
	// Publish the map
	mMapPublisher.publish(mGridMap);
	// Update publish time to control sendMap frequency
	mLastMapUpdate = ros::WallTime::now();
	
	// Publish the pose-graph
	if(mPublishPoseGraph)
	{
		// Publish the vertices
		karto::MapperGraph::VertexList vertices = mMapper->GetGraph()->GetVertices();
		visualization_msgs::Marker marker;
		marker.header.frame_id = mMapFrame;
		marker.header.stamp = ros::Time();
		marker.ns = "mapper";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.points.resize(vertices.Size());
		
		// Clear previous path info
		pathAftPgo.header.frame_id = mMapFrame;
		pathAftPgo.header.stamp = ros::Time();
		pathAftPgo.poses.clear();

		for(int i = 0; i < vertices.Size(); i++)
		{
			marker.points[i].x = vertices[i]->GetVertexObject()->GetCorrectedPose().GetX();
			marker.points[i].y = vertices[i]->GetVertexObject()->GetCorrectedPose().GetY();
			marker.points[i].z = 0;
			
			// path after pgo
			geometry_msgs::PoseStamped poseAftPgo;
			poseAftPgo.header = pathAftPgo.header;
			poseAftPgo.pose.position.x = marker.points[i].x;
			poseAftPgo.pose.position.y = marker.points[i].y;
			poseAftPgo.pose.position.z = 0;
			poseAftPgo.pose.orientation = tf::createQuaternionMsgFromYaw(vertices[i]->GetVertexObject()->GetCorrectedPose().GetHeading());
			pathAftPgo.poses.push_back(poseAftPgo);
		}
		mVerticesPublisher.publish(marker);

        // Publish topic /slam_path
		mPathPublisher.publish(pathAftPgo);  // Up-to-date pose estimation of robot, to store the slam path
		
		// Publish the edges
		karto::MapperGraph::EdgeList edges = mMapper->GetGraph()->GetEdges();
		marker.header.frame_id = mMapFrame;
		marker.header.stamp = ros::Time();
		marker.ns = "mapper";
		marker.id = 1;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.scale.x = 0.05;  // 0.02
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		marker.points.resize((edges.Size() - vertices.Size() + 1) * 2);

		// Marker for odometry edge
		visualization_msgs::Marker connect_marker;
		connect_marker.header.frame_id = mMapFrame;
		connect_marker.header.stamp = ros::Time();
		connect_marker.ns = "mapper";
		connect_marker.id = 2;
		connect_marker.type = visualization_msgs::Marker::LINE_LIST;
		connect_marker.action = visualization_msgs::Marker::ADD;
		connect_marker.pose.position.x = 0;
		connect_marker.pose.position.y = 0;
		connect_marker.pose.position.z = 0;
		connect_marker.pose.orientation.x = 0.0;
		connect_marker.pose.orientation.y = 0.0;
		connect_marker.pose.orientation.z = 0.0;
		connect_marker.pose.orientation.w = 1.0;
		connect_marker.scale.x = 0.3;    // 0.15
 		connect_marker.color.a = 1.0;
		connect_marker.color.r = 1.0;
		connect_marker.color.g = 0.0;
		connect_marker.color.b = 0.0;
		connect_marker.points.resize((vertices.Size() - 1) * 2);

		// Marker for pose graph edges with D-opt weight
		std::vector<double> detCovariances;
		double maxDet = 0;
		double minDet = std::numeric_limits<double>::max();
		visualization_msgs::MarkerArray marker_array;



		
		int vertexIndex = 0;
		int p = 0, q = 0;
		for(int i = 0; i < edges.Size(); i++)
		{	
			if(vertexIndex + 1 < vertices.Size() && edges[i]->GetSource() == vertices[vertexIndex] && 
			   edges[i]->GetTarget() == vertices[vertexIndex+1]){
				connect_marker.points[2*p].x = edges[i]->GetSource()->GetVertexObject()->GetCorrectedPose().GetX();
				connect_marker.points[2*p].y = edges[i]->GetSource()->GetVertexObject()->GetCorrectedPose().GetY();
				connect_marker.points[2*p].z = 0;

				connect_marker.points[2*p+1].x = edges[i]->GetTarget()->GetVertexObject()->GetCorrectedPose().GetX();
				connect_marker.points[2*p+1].y = edges[i]->GetTarget()->GetVertexObject()->GetCorrectedPose().GetY();
				connect_marker.points[2*p+1].z = 0;	
				p++;
				vertexIndex++;

				// TODO: Publish colored marker according the covariance
				std::vector<double> cov = edges[i]->GetCovarianceVector();
				Eigen::Matrix3d matrix;
				matrix << cov[0], cov[1], cov[2],
						  cov[3], cov[4], cov[5],
						  cov[6], cov[7], cov[8];
				double determinant = matrix.inverse().determinant();
				maxDet = std::max(maxDet, determinant);
				minDet = std::min(minDet, determinant);
				detCovariances.push_back(determinant);

				// Add colored edge markers
				visualization_msgs::Marker marker2;
				marker2.header.frame_id = "map";
				marker2.header.stamp = ros::Time::now();
				marker2.ns = "colored_edge";
				marker2.type = visualization_msgs::Marker::LINE_STRIP;
				marker2.action = visualization_msgs::Marker::ADD;
				marker2.scale.x = 0.3; // 0.15
				marker2.lifetime = ros::Duration();
				marker2.pose.position.x = 0;
				marker2.pose.position.y = 0;
				marker2.pose.position.z = 0;
				marker2.pose.orientation.x = 0.0;
				marker2.pose.orientation.y = 0.0;
				marker2.pose.orientation.z = 0.0;
				marker2.pose.orientation.w = 1.0;
				// Line segment
				marker2.id = i;
				marker2.color.a = 1.0;
				geometry_msgs::Point p1_1, p2_1;
				p1_1.x = edges[i]->GetSource()->GetVertexObject()->GetCorrectedPose().GetX();
				p1_1.y = edges[i]->GetSource()->GetVertexObject()->GetCorrectedPose().GetY();
				p1_1.z = 0.0;
				p2_1.x = edges[i]->GetTarget()->GetVertexObject()->GetCorrectedPose().GetX();
				p2_1.y = edges[i]->GetTarget()->GetVertexObject()->GetCorrectedPose().GetY();
				p2_1.z = 0.0;
				marker2.points.push_back(p1_1);
				marker2.points.push_back(p2_1);
				marker_array.markers.push_back(marker2);
			}
			else{
				marker.points[2*q].x = edges[i]->GetSource()->GetVertexObject()->GetCorrectedPose().GetX();
				marker.points[2*q].y = edges[i]->GetSource()->GetVertexObject()->GetCorrectedPose().GetY();
				marker.points[2*q].z = 0;

				marker.points[2*q+1].x = edges[i]->GetTarget()->GetVertexObject()->GetCorrectedPose().GetX();
				marker.points[2*q+1].y = edges[i]->GetTarget()->GetVertexObject()->GetCorrectedPose().GetY();
				marker.points[2*q+1].z = 0;	
				q++;
			}
		}

		// Coloring the edges
		for(int j = 0; j < marker_array.markers.size(); j++){
			double determinant = detCovariances[j];
			double normalized_det;
			// FIXME: The value is donimated by the maxDet
			if (maxDet <= minDet)
				normalized_det = 1;
			else 
				normalized_det = (determinant - minDet) / (maxDet - minDet);
			marker_array.markers[j].color.b = normalized_det;
			marker_array.markers[j].color.r = 1 - normalized_det;
		}

		mColoredEdgePublisher.publish(marker_array);

		mClosureEdgesPublisher.publish(marker);
		mOdomEdgesPublisher.publish(connect_marker);  // Path do not need to be published repeatedly.
	}
	return true;
}

bool MultiMapper::updateMap()
{
	if(!mMapChanged) return true;
	
	const karto::LocalizedLaserScanList allScans = mMapper->GetAllProcessedScans();
	karto::OccupancyGridPtr kartoGrid = karto::OccupancyGrid::CreateFromScans(allScans, mMapResolution);

	if(!kartoGrid)
	{
		ROS_WARN("Failed to get occupancy map from Karto-Mapper.");
		return false; 
	}

	// Translate to ROS format
	unsigned int width = kartoGrid->GetWidth();
	unsigned int height = kartoGrid->GetHeight();
	karto::Vector2<kt_double> offset = kartoGrid->GetCoordinateConverter()->GetOffset();

	if(	mGridMap.info.width != width || 
		mGridMap.info.height != height || 
		mGridMap.info.origin.position.x != offset.GetX() || 
		mGridMap.info.origin.position.y != offset.GetY())
	{
		mGridMap.info.resolution = mMapResolution;
		mGridMap.info.origin.position.x = offset.GetX();
		mGridMap.info.origin.position.y = offset.GetY();
		mGridMap.info.width = width;
		mGridMap.info.height = height;
		mGridMap.data.resize(mGridMap.info.width * mGridMap.info.height);
	}
	
	for (unsigned int y = 0; y < height; y++)
	{
		for (unsigned int x = 0; x < width; x++) 
		{
			// Getting the value at position x,y
			kt_int8u value = kartoGrid->GetValue(karto::Vector2<kt_int32s>(x, y));

			switch (value)
			{
			case karto::GridStates_Unknown:
				mGridMap.data[MAP_IDX(mGridMap.info.width, x, y)] = -1;
				break;
			case karto::GridStates_Occupied:
				mGridMap.data[MAP_IDX(mGridMap.info.width, x, y)] = 100;
				break;
			case karto::GridStates_Free:
				mGridMap.data[MAP_IDX(mGridMap.info.width, x, y)] = 0;
				break;
			default:
				ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
				break;
			}
		}
	}

	// Set the header information on the map
	mGridMap.header.stamp = ros::Time::now();
	mGridMap.header.frame_id = mMapFrame.c_str();
	mMapChanged = false;
	return true;
}

void MultiMapper::receiveLocalizedScan(const nav2d_msgs::LocalizedScan::ConstPtr& scan)
{
	// Ignore my own scans
	if(scan->robot_id == mRobotID) return;
	
	// Get the robot id
	char robot[10];
	sprintf(robot, "robot_%d", scan->robot_id);
	
	// Get the scan pose
	karto::Pose2 scanPose(scan->x, scan->y, scan->yaw);
	
	// create localized laser scan
	karto::LocalizedLaserScanPtr localizedScan = createFromRosMessage(scan->scan, robot);
	localizedScan->SetOdometricPose(scanPose);
	localizedScan->SetCorrectedPose(scanPose);
	
	// feed the localized scan to the Karto-Mapper
	bool added = false;
	try
	{
		added = mMapper->Process(localizedScan);
	}
	catch(karto::Exception e1)
	{
		if(mOtherLasers.find(scan->robot_id) == mOtherLasers.end())
		{
			try
			{
				karto::LaserRangeFinderPtr laser = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, robot);
				laser->SetMinimumRange(scan->scan.range_min);
				laser->SetMaximumRange(scan->scan.range_max);
				laser->SetMinimumAngle(scan->scan.angle_min);
				laser->SetMaximumAngle(scan->scan.angle_max);
				laser->SetAngularResolution(scan->scan.angle_increment);
				laser->SetRangeThreshold(mRangeThreshold);
				mMapper->Process(laser);
				mOtherLasers.insert(std::pair<int,karto::LaserRangeFinderPtr>(scan->robot_id,laser));
			
				added = mMapper->Process(localizedScan);
			}
			catch(karto::Exception e2)
			{
				ROS_ERROR("%s", e2.GetErrorMessage().ToCString());
			}
		}else
		{
			ROS_ERROR("%s", e1.GetErrorMessage().ToCString());
		}
	}
	if(added)  // New laserscan is registered into current map
	{
		mNodesAdded++;
		mMapChanged = true;
		ROS_DEBUG("Robot %d: Received scan (uniqueID: %d, Sensor: %s, stateID: %d)", mRobotID, localizedScan->GetUniqueId(), localizedScan->GetSensorIdentifier().ToString().ToCString(), localizedScan->GetStateId());
		
		// Publish via extra topic
		nav2d_msgs::RobotPose other;
		other.header.stamp = ros::Time::now();
		other.header.frame_id = mMapFrame;
		other.robot_id = scan->robot_id;
		other.pose.x = localizedScan->GetCorrectedPose().GetX();
		other.pose.y = localizedScan->GetCorrectedPose().GetY();
		other.pose.theta = localizedScan->GetCorrectedPose().GetHeading();
		mOtherRobotsPublisher.publish(other);
		
		// Send the map via topic
		ros::WallDuration d = ros::WallTime::now() - mLastMapUpdate;
		if(mMapUpdateRate > 0 && d.toSec() > mMapUpdateRate)
		{
			sendMap();
			if(mState == ST_LOCALIZING)
			{
				mSelfLocalizer->convertMap(mGridMap);
			}
		}
	}else
	{
		ROS_DEBUG("Discarded Scan from Robot %d!", scan->robot_id);
	}

	if(mState == ST_WAITING_FOR_MAP && mNodesAdded >= mMinMapSize)
	{
		sendMap();
		mSelfLocalizer->convertMap(mGridMap);
		mSelfLocalizer->initialize();
		mState = ST_LOCALIZING;
		ROS_INFO("Received a map, now starting to localize.");
		mSelfLocalizer->publishParticleCloud();
	}
}

void MultiMapper::sendLocalizedScan(const sensor_msgs::LaserScan::ConstPtr& scan, const karto::Pose2& pose)
{	
	// Send localized scan out and change to ros format
	nav2d_msgs::LocalizedScan rosScan;
	rosScan.robot_id = mRobotID;
	rosScan.laser_type = 0;
	rosScan.x = pose.GetX();
	rosScan.y = pose.GetY();
	rosScan.yaw = pose.GetHeading();
	
	rosScan.scan.angle_min = scan->angle_min;
	rosScan.scan.angle_max = scan->angle_max;
	rosScan.scan.range_min = scan->range_min;
	rosScan.scan.range_max = scan->range_max;
	rosScan.scan.angle_increment = scan->angle_increment;
	rosScan.scan.time_increment = scan->time_increment;
	rosScan.scan.scan_time = scan->scan_time;
	
	unsigned int nReadings = scan->ranges.size();
	rosScan.scan.ranges.resize(nReadings);
	for(unsigned int i = 0; i < nReadings; i++)
	{
		rosScan.scan.ranges[i] = scan->ranges[i];
	}

//	rosScan.scan = *scan;
	mScanPublisher.publish(rosScan);
}

void MultiMapper::receiveInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
	double x = pose->pose.pose.position.x;
	double y = pose->pose.pose.position.y;
	double yaw = tf::getYaw(pose->pose.pose.orientation);
	ROS_INFO("Received initial pose (%.2f, %.2f, %.2f) on robot %d, now starting to map.",x,y,yaw,mRobotID);
	try
	{
		setRobotPose(x,y,yaw);
	}
	catch(tf::TransformException e)
	{
		ROS_ERROR("Failed to set pose on robot %d! (%s)", mRobotID, e.what());
		return;
	}
}

void MultiMapper::onMessage(const void* sender, karto::MapperEventArguments& args)
{
	ROS_DEBUG("OpenMapper: %s\n", args.GetEventMessage().ToCString());
}

void MultiMapper::publishTransform()
{
	if(mState == ST_MAPPING)
	{
		mTransformBroadcaster.sendTransform(tf::StampedTransform (mOdometryOffset, ros::Time::now() , mOffsetFrame, mOdometryFrame));
		mTransformBroadcaster.sendTransform(tf::StampedTransform (mMapToOdometry, ros::Time::now() , mMapFrame, mOffsetFrame));
	}
}
