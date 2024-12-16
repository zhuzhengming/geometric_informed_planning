
// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/SetBool.h>
#include <visualization_msgs/Marker.h>

// PCL includes (main doc: https://pcl.readthedocs.io/projects/tutorials/en/master/#)
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h> // remove Nan from pc
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/shot_lrf_omp.h>
#include <pcl/features/usc.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/registration/gicp.h> // Include GICP header
#include <pcl/registration/transformation_estimation_lm.h> // Include LM estimation header

#define VOXEL_SIZE          0.1 // resolution of the voxel grid in meters 
#define MIN_MOVE_DIST       0.0 // minimum displacement of the sensor in meters before adding a new point cloud
#define USE_LAST_MERGED     false // use the last merged point cloud as reference for the next point cloud (full merged point cloud otherwise)
#define FEATURE_RADIUS      1.5  // radius for feature estimation in meters (required by features' setRadiusSearch() method)
#define MAX_FEATURE_DIST    3.0 // maximum distance between features and pose (required by features' setMaxCorrespondenceDistance() method)
#define MAX_ANGLE_ALIGN     M_PI/8.0 // maximum allowable transformation angle as returned by the feature matching algorithm (in radians)
#define MAX_TRANS_ALIGN     1.0 // maximum allowable transformation translation norm as returned by the feature matching algorithm (in meters)
#define ICP_TYPE            'GICP' // 0 for ICP, 1 for GICP, 2 for LM

// Enum for the type of ICP to use
enum ICPType {
    ICP,
    GICP,
    DOF3
};

ICPType icp_type = GICP;


static ros::Publisher dwnsp_pc_pub; // publish the point cloud after downsampling
static ros::Publisher fused_pc_pub; // publish the fused and aligned point cloud


//static ros::Publisher kp_scene_pc_pub; // publish the keypoints of the scene 
//static ros::Publisher kp_object_pc_pub; // publish the keypoints of the object
//static ros::Publisher inliers_pc_pub; // publish the inliers after feature matching
//static ros::Publisher corresp_pub; // publish the correspondences after feature matching
//static ros::Publisher icp_pc_pub;   // publish the point cloud after ICP alignment
//static ros::Publisher noicp_pc_pub; // publish the fused point cloud before alignment
//static ros::Publisher voxel_grid_pub; // publish the voxel grid for visualization

// Visual odometery varaibles
static ros::Publisher position_adjusted_icp; 

static ros::ServiceClient rosbag_playback_client; // client for the rosbag pause service

/**
 * @brief The pose of a frame 
*/
struct Pose{
    Eigen::Vector3f p;
    Eigen::Quaternionf q;
} typedef Pose;

static Pose real_pose; // The real position of the drone (not known in real life)
static Pose noisy_pose; // Position of the drone that we recive
static Eigen::Matrix4f pose_error; // Error of the drone position and orientation (need to keep track of)


/**
 * @brief Save drone positions to a CSV file.
 * 
 * This function appends the positions of the drone (real position, adjusted position, and noisy position) 
 * to a CSV file specified by the filepath. The positions are written in the format:
 * 
 * @param filepath The path to the CSV file where positions will be saved.
 * @param drone_position The real position of the drone (x, y, z).
 * @param drone_adjusted The adjusted position of the drone (x, y, z) after alignment.
 * @param noisy_drone The noisy position of the drone (x, y, z).
 */
void savePositionsToFile(const std::string& filepath, Eigen::Vector3f drone_position,Eigen::Vector3f drone_adjusted,Eigen::Vector3f noisy_drone){                                        
    std::ofstream file(filepath, std::ios::app); // Open file in append mode
    if (!file.is_open()) {
        std::cout << "Failed to open file: " << filepath << std::endl;
        return;
    }
    // Write table header

    file << drone_position.x() << "," << drone_position.y() << "," << drone_position.z() << "," 
         << drone_adjusted.x() << "," << drone_adjusted.y() << "," << drone_adjusted.z() << ","
         << noisy_drone.x() << "," << noisy_drone.y() << "," << noisy_drone.z() << std::endl;
    file.close();
}


/**
 * @brief Pause or resume the rosbag playback.
 * @param pause If true, pause the rosbag playback. If false, resume the rosbag playback.
*/
void pause_bag(bool pause=true){
    std_srvs::SetBool srv;
    srv.request.data = pause;
    if(!rosbag_playback_client.call(srv) || !srv.response.success){
        ROS_ERROR("Failed to call the rosbag playback service");
    }
    else{
        ROS_INFO("%s rosbag playback", pause ? "Paused" : "Resumed");
    }
}

/**
 * @brief Get the transform between two frames.
 * @param frame_id The parent frame
 * @param child_frame_id The child frame
 * @param t The time stamp
 * @param[out] p_eigen The translation vector
 * @param[out] q_eigen The rotation quaternion
 * @param inverse If true, return the inverse transform
 * @returns 0 if successful, -1 otherwise
*/
int get_tf_transform(std::string frame_id, std::string child_frame_id, ros::Time t,
                      Eigen::Vector3f& p_eigen, Eigen::Quaternionf& q_eigen,
                      bool inverse = false){

    static tf::TransformListener listener(ros::Duration(100));

    tf::StampedTransform transform;
    
    try{
        listener.waitForTransform(frame_id, child_frame_id, t, ros::Duration(1.0));
        listener.lookupTransform(frame_id, child_frame_id, t, transform);
    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        return -1;
    }
    
    tf::Vector3    p = transform.getOrigin();
    tf::Quaternion q = transform.getRotation();

    // Convert to Eigen
    p_eigen << p.x(), p.y(), p.z();
    q_eigen.w() = q.w();
    q_eigen.x() = q.x();
    q_eigen.y() = q.y();
    q_eigen.z() = q.z();
    q_eigen.normalize();

    if(inverse){
        p_eigen = -p_eigen;
        q_eigen = q_eigen.conjugate();
    }

    return 0;
}

/**
 * @brief Publish a PCL point cloud with points of type T as ROS sensor_msgs::PointCloud2 message.
 * @tparam T The point type used in the point cloud (e.g. pcl::PointXYZ, pcl::PointNormal, pcl::PointXYZRGB, etc.)
 * @param cloud_ptr The point cloud to be published
 * @param header The header of the point cloud
 * @param pc_pub The publisher
 * @note This function is templated to allow publishing point clouds of different types.
*/
template<typename T>
void publish_pc(typename pcl::PointCloud<T>::Ptr cloud_ptr, std_msgs::Header header, ros::Publisher& pc_pub){
    sensor_msgs::PointCloud2 msg_filtered;
    pcl::toROSMsg<T>(*cloud_ptr, msg_filtered);
    msg_filtered.header = header;
    pc_pub.publish(msg_filtered);
}

/**
 * @brief Cleanup the point cloud by removing outliers.
 * @param pc The point cloud to clean up (will be modified)
*/
void cleanup_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc){
    // Apply a radius filter to remove outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter_radius;
    int k = 5;
    filter_radius.setRadiusSearch(k*VOXEL_SIZE);
    filter_radius.setMinNeighborsInRadius(k*2);
    filter_radius.setInputCloud(pc);
    filter_radius.filter(*pc);
}

/**
 * @brief Clean the point cloud by removing outliers.
 * @param pc The point cloud to clean up (will be modified)
*/
void clean_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc){
    if (pc->empty()) {
        ROS_WARN("Received an empty point cloud in cleanup_pc.");
        return;
    }

    // Parameters based on the voxel grid filter
    double voxel_size = VOXEL_SIZE;  // Assume VOXEL_SIZE is defined elsewhere
    int base_k = 2;  // Base multiplier for setting radius and neighbor threshold

    // Set the radius search based on voxel size, slightly larger to cover adjacent voxels
    double radius = base_k * voxel_size;

    // Estimate minimum neighbors based on expected density
    // Since data is voxelized, a point should theoretically have neighbors in all surrounding voxels under ideal conditions
    int min_neighbors = std::max(1, base_k * 1);  // Adjust the multiplier based on your data's characteristics
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
    radius_filter.setInputCloud(pc);
    radius_filter.setRadiusSearch(radius);
    radius_filter.setMinNeighborsInRadius(min_neighbors);
    radius_filter.filter(*pc);

    ROS_INFO("After radius filtering, %ld points remain", pc->size());
}

/**
 * @brief Verify the quality of the transformation returned by an alignment algorithm.
 * @param transformation The transformation matrix
 * @returns True if the transformation is acceptable, false otherwise
*/
bool verify_transform(Eigen::Matrix4f& transformation){
    // Assess quality of transformation
    Eigen::Vector3f t = transformation.block<3,1>(0,3);
    Eigen::Matrix3f R = transformation.block<3,3>(0,0);
    double angle = std::acos((R.trace() - 1.0)*0.5);
    double trans = t.norm();
    if(angle > MAX_ANGLE_ALIGN || trans > MAX_TRANS_ALIGN){
        ROS_ERROR("Feature alignment yielded an overcorrection (translation: %.3lf, rotation: %.3lf)",trans,angle);
        return false;
    }
    return true;
}

/**
 * @brief Apply the ICP algorithm to align pc_ptr relative to ref_ptr. This modifies pc_ptr.
 * @param ref_ptr The reference point cloud
 * @param pc_ptr The point cloud to be aligned (will be modified)
 * @returns True if the ICP converged, false otherwise, and the transformation matrix
*/
bool perform_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr ref_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr, Eigen::Matrix4f& transformation){

    // Make a copy of the point cloud to be aligned
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_copy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pc_ptr, *pc_copy);
    // Run the ICP algorithm
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setMaximumIterations(100);
    // icp.setTransformationEpsilon(1e-8);
    // icp.setMaxCorrespondenceDistance(0.1);
    // icp.setEuclideanFitnessEpsilon(1e-8);
    // icp.setRANSACOutlierRejectionThreshold(0.01);
    // icp.setInputTarget(ref_ptr);
    // icp.setInputSource(pc_ptr);
    // icp.align(*pc_copy);

    // Run the ICP algorithm
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set ICP parameters
    icp.setMaximumIterations(150);  // Set a low number of iterations
    icp.setTransformationEpsilon(0.001);  // Small transformation epsilon
    icp.setMaxCorrespondenceDistance(0.5);  // Increase max correspondence distance
    icp.setEuclideanFitnessEpsilon(0.001);  // Increase fitness epsilon
    icp.setRANSACOutlierRejectionThreshold(0.05);  // Set RANSAC outlier rejection threshold
    icp.setInputTarget(ref_ptr);
    icp.setInputSource(pc_ptr);
    icp.align(*pc_copy);

    // Get the transformation matrix from the registration
    bool converged = icp.hasConverged();
    if (converged){
        double fitness = icp.getFitnessScore();
        ROS_INFO("ICP has converged, score is %f", fitness);
        //Eigen::Matrix4f transformation = icp.getFinalTransformation();
        if(fitness > 0.1){
            ROS_WARN("High ICP fitness score");
            // converged = false; // do not consider the ICP converged if the fitness score is too high
        }
    }
    else{
        ROS_ERROR("ICP has not converged.");
    }
    
    // Assess quality of transformation
    transformation = icp.getFinalTransformation();
    if(!verify_transform(transformation)){
        converged = false;
        transformation = Eigen::Matrix4f::Zero();
    } 

    if(converged){
        // Update the input point cloud with the aligned version
        pcl::copyPointCloud(*pc_copy, *pc_ptr);
    
    }

    return converged;

}


/**
 * @brief Apply the modified ICP algorithm to align pc_ptr relative to ref_ptr. This modifies pc_ptr.
 * @param ref_ptr The reference point cloud
 * @param pc_ptr The point cloud to be aligned (will be modified)
 * @returns True if the ICP converged, false otherwise, and the transformation matrix
*/
bool perform_3dof_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr ref_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr, Eigen::Matrix4f& transformation){
    // Make a copy of the point cloud to be aligned
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_copy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pc_ptr, *pc_copy);
    // Run the ICP algorithm
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setMaximumIterations(100);
    // icp.setTransformationEpsilon(1e-8);
    // icp.setMaxCorrespondenceDistance(0.1);
    // icp.setEuclideanFitnessEpsilon(1e-8);
    // icp.setRANSACOutlierRejectionThreshold(0.01);
    // icp.setInputTarget(ref_ptr);
    // icp.setInputSource(pc_ptr);
    // icp.align(*pc_copy);
    icp.setMaximumIterations(200);  // Limit the number of iterations to prevent overfitting and reduce computation time
    icp.setTransformationEpsilon(5e-4);  // Set a small transformation epsilon to ensure convergence
    icp.setMaxCorrespondenceDistance(0.1);  // Use a moderate max correspondence distance to handle larger discrepancies
    icp.setEuclideanFitnessEpsilon(1e-6);  // Set a small fitness epsilon for precise alignment
    icp.setRANSACOutlierRejectionThreshold(0.1); // Set RANSAC outlier rejection threshold
    icp.setInputTarget(ref_ptr);
    icp.setInputSource(pc_ptr);
    icp.align(*pc_copy);
    // Get the transformation matrix from the registration
    bool converged = icp.hasConverged();
    if (converged){
        double fitness = icp.getFitnessScore();
        ROS_INFO("ICP has converged, score is %f", fitness);
        //Eigen::Matrix4f transformation = icp.getFinalTransformation();
        if(fitness > 0.1){
            ROS_WARN("High ICP fitness score");
            // converged = false; // do not consider the ICP converged if the fitness score is too high
        }
    }
    else{
        ROS_ERROR("ICP has not converged.");
    }
    // Modify the transformation matrix to only include translation
    Eigen::Matrix4f translation_transform = Eigen::Matrix4f::Identity();
    translation_transform.block<3,1>(0,3) = icp.getFinalTransformation().block<3,1>(0,3);
    transformation = translation_transform;
    // Assess quality of transformation
    if(!verify_transform(transformation)){
        converged = false;
        transformation = Eigen::Matrix4f::Zero();
    }
    if(converged){
        // Update the input point cloud with the aligned version
        pcl::copyPointCloud(*pc_copy, *pc_ptr);
    }

    return converged;
}


/**
 * @brief Apply the GICP algorithm to align pc_ptr relative to ref_ptr. This modifies pc_ptr.
 * @param ref_ptr The reference point cloud
 * @param pc_ptr The point cloud to be aligned (will be modified)
 * @returns True if the GICP converged, false otherwise, and the transformation matrix
*/
bool perform_gicp(pcl::PointCloud<pcl::PointXYZ>::Ptr ref_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr, Eigen::Matrix4f& transformation) {
    // Make a copy of the point cloud to be aligned
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_copy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pc_ptr, *pc_copy);

    // Run the GICP algorithm
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaximumIterations(150);  // Set a reasonable number of iterations
    gicp.setTransformationEpsilon(1e-5);  // Small transformation epsilon
    gicp.setMaxCorrespondenceDistance(0.3);  // Set max correspondence distance
    gicp.setEuclideanFitnessEpsilon(1e-6);  // Small fitness epsilon for precise alignment
    gicp.setInputTarget(ref_ptr);
    gicp.setInputSource(pc_ptr);
    gicp.align(*pc_copy);

    // Get the transformation matrix from the registration
    bool converged = gicp.hasConverged();
    if (converged) {
        double fitness = gicp.getFitnessScore();
        ROS_INFO("GICP has converged, score is %f", fitness);
        if (fitness > 0.1) {
            ROS_WARN("High GICP fitness score");
        }
    } else {
        ROS_ERROR("GICP has not converged.");
    }

    // Assess quality of transformation
    transformation = gicp.getFinalTransformation();
    if (!verify_transform(transformation)) {
        converged = false;
        transformation = Eigen::Matrix4f::Zero();
    }

    if (converged) {
        // Update the input point cloud with the aligned version
        pcl::copyPointCloud(*pc_copy, *pc_ptr);
    }

    return converged;
}



// Function to apply transformation and return a new pose
Pose applyTransformation(const Eigen::Matrix4f& transform, const Pose& pose) {
    Pose transformed_pose;
    // Extract rotation matrix and translation vector from the transformation matrix
    Eigen::Matrix3f rotationMatrix = transform.block<3,3>(0,0);
    Eigen::Vector3f translation = transform.block<3,1>(0,3);
    // Apply rotation and translation to position
    transformed_pose.p = rotationMatrix * pose.p + translation;
    // Convert rotation matrix to quaternion
    Eigen::Quaternionf rotationQuaternion(rotationMatrix);
    // Apply rotation to quaternion
    transformed_pose.q = rotationQuaternion * pose.q;
    return transformed_pose;
}


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Number of merged point clouds 
    static int count = 0;
    // Position error transform from noisy_pose to adjusted_pose
    static Eigen::Matrix4f position_error_transform = Eigen::Matrix4f::Identity();


    // Declare the point cloud that will contain the merged data
    static pcl::PointCloud<pcl::PointXYZ>::Ptr pc_merged_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // Declare the point cloud that will contain the last merged data (for feature matching)
    static pcl::PointCloud<pcl::PointXYZ>::Ptr last_merged_pc(new pcl::PointCloud<pcl::PointXYZ>);


    // Abort if the drone did not move enough since last point cloud
    // ROS_INFO("Message header for the frame: %s", msg->header.frame_id.c_str()); // Good frame without pose noise

    // msg->header.frame_id == 'world' (need to use world_noisy for testing)
    get_tf_transform("map", msg->header.frame_id, msg->header.stamp, real_pose.p, real_pose.q); // Get the ground truth position
    get_tf_transform("map", "world_noisy", msg->header.stamp, noisy_pose.p, noisy_pose.q); // Get the noisy position
    static Pose last_adjusted_pose = noisy_pose; // first time, last adj pose = adjusted_pose
    static Pose adjusted_pose = noisy_pose;

    // Estimate the postiion based on the recieved postion (noisy_pose) and the position_error_transform
    adjusted_pose = applyTransformation(position_error_transform, noisy_pose);

    static int init_count = 0;

    if((adjusted_pose.p - last_adjusted_pose.p).norm() < MIN_MOVE_DIST && init_count > 10){
        ROS_WARN("Sensor did not move enough, skipping point cloud (moved %.3lf < %.3lf m)", (adjusted_pose.p - last_adjusted_pose.p).norm(), MIN_MOVE_DIST);
        return;
    }
    else{
        last_adjusted_pose = adjusted_pose;
        init_count++;
    }

    count++; // increment the number of merged point clouds


    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromROSMsg(*msg, pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr(pc.makeShared());
    std_msgs::Header header = msg->header;
    std::cout << std::endl; // clear the previous line
    ROS_INFO("[PC %d] Received %d data points in frame %s", count, (int)pc.points.size(), msg->header.frame_id.c_str());



    ///////////////////////////////
    // Operations in local frame //
    ///////////////////////////////
    // remove NaNs
    pcl::Indices indices;
    // Remove NaN points from pc_ptr and store the result in pc_filtered
    pcl::removeNaNFromPointCloud(*pc_ptr, *pc_ptr, indices); // Modify pc_ptr in place
    ROS_INFO("%ld points remaining after remove inf nan", pc_ptr->size());

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    // Apply a VoxelGrid filter to downsample the data
    // voxel_grid.setDownsampleAllData(true);
    // voxel_grid.setFilterLimits(-100, 100); // -100,100 base
    // voxel_grid.setMinimumPointsNumberPerVoxel(1); // 1 base
    // voxel_grid.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    // voxel_grid.setSaveLeafLayout(false);
    // voxel_grid.setInputCloud(pc_ptr);
    // voxel_grid.filter(*pc_ptr);
    // ROS_INFO("%ld points remaining after voxelgrid", pc_ptr->size());

    // Discard points that are too close or too far away
    pcl::PassThrough<pcl::PointXYZ> filter_passthrough;
    // X-axis filtering
    filter_passthrough.setFilterFieldName("x");
    filter_passthrough.setFilterLimits(-0.1, 4.0);
    filter_passthrough.setInputCloud(pc_ptr);
    filter_passthrough.filter(*pc_ptr);
    // Y-axis filtering
    filter_passthrough.setFilterFieldName("y");
    filter_passthrough.setFilterLimits(-3.0, 3.0);
    filter_passthrough.setInputCloud(pc_ptr);
    filter_passthrough.filter(*pc_ptr);
    // Z-axis filtering
    filter_passthrough.setFilterFieldName("z");
    filter_passthrough.setFilterLimits(-3.0, 3.0);
    filter_passthrough.setInputCloud(pc_ptr);
    filter_passthrough.filter(*pc_ptr);
    ROS_INFO("%ld points remaining after filter passthrough" , pc_ptr->size());


    // Remove outliers using the Statistical Outlier Removal filter
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(pc_ptr);
    // sor.setMeanK(50); // Number of neighboring points to analyze for each point
    // sor.setStddevMulThresh(1.0); // Standard deviation multiplier threshold
    // sor.filter(*pc_ptr);
    // ROS_INFO("%ld points remaining after statistical outlier removal", pc_ptr->size());

    // Remove outliers in the point cloud
    // clean_pc(pc_ptr);

    // Publish the downsampled point cloud (borrow header from original point cloud)
    publish_pc<pcl::PointXYZ>(pc_ptr, header, dwnsp_pc_pub);
    ROS_INFO("dwnsp_pc_pub published");

    ////////////////////////////////
    // Operations in global frame //
    ////////////////////////////////

    // Rotate the point cloud in the global frame of reference
    // Eigen::Vector3f p;
    // Eigen::Quaternionf q;
    // if(get_tf_transform("map", msg->header.frame_id, msg->header.stamp, p, q)){
        // ROS_ERROR(" Could not lookup transform from %s to %s", "map", msg->header.frame_id.c_str());
        // return;
    // }
    // pcl::transformPointCloud(*pc_ptr, *pc_ptr, p, q);

    // Transform the pointcloud into the world frame (with estimate position)
    pcl::transformPointCloud(*pc_ptr, *pc_ptr, adjusted_pose.p, adjusted_pose.q);
    header.frame_id = "map"; // change the frame id to the global frame after the transform

    ROS_INFO("Check 1");
    ROS_INFO("%ld points remaining", pc_ptr->size());


    // Publish the to be merged point cloud prior any alignment 
    // publish_pc<pcl::PointXYZ>(pc_ptr, header, noicp_pc_pub);

    // If no point cloud is available yet, add the first one
    if(pc_merged_ptr->points.size() == 0){
        *pc_merged_ptr += *pc_ptr;
        pcl::copyPointCloud(*pc_ptr, *last_merged_pc);
        ROS_WARN("Init, returning, %ld points", pc_merged_ptr->size());
        return;
    }

    // Select the reference point cloud for the alignment 
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref_pc(new pcl::PointCloud<pcl::PointXYZ>);
    if(USE_LAST_MERGED){
        pcl::copyPointCloud(*last_merged_pc, *ref_pc);
    }
    else{
        pcl::copyPointCloud(*pc_merged_ptr, *ref_pc);
    }


    ROS_INFO("Pre-icp");
    // Run the ICP algorithm
    Eigen::Matrix4f iter_pos_error_transform = Eigen::Matrix4f::Zero();
    bool icp_converged = false;
    switch (icp_type) {
    case ICP:
        // Call the standard ICP function
        icp_converged = perform_icp(ref_pc, pc_ptr, iter_pos_error_transform);
        break;
    case GICP:
        // Call the GICP function
        icp_converged = perform_gicp(ref_pc, pc_ptr, iter_pos_error_transform);
        break;
    case DOF3:
        // Call the 3 DoF ICP function
        icp_converged = perform_3dof_icp(ref_pc, pc_ptr, iter_pos_error_transform);
        break;
    default:
        ROS_INFO("Unknown ICP type, default to 3 DoF ICP");
        icp_converged = perform_3dof_icp(ref_pc, pc_ptr, iter_pos_error_transform);
        break;
    }


    if(icp_converged){
        ROS_INFO("ICP converge, position published at ....");
        // Update the position error transform
        // position_error_transform = iter_pos_error_transform * position_error_transform;
        position_error_transform = position_error_transform * iter_pos_error_transform;
        // Compute the adjusted position
        adjusted_pose = applyTransformation(position_error_transform, noisy_pose);

        geometry_msgs::PoseStamped adjusted_pose_msg;
        adjusted_pose_msg.header.stamp = ros::Time::now();
        adjusted_pose_msg.header.frame_id = "mocap";  // or the appropriate frame
        adjusted_pose_msg.pose.position.x = adjusted_pose.p.x();
        adjusted_pose_msg.pose.position.y = adjusted_pose.p.y();
        adjusted_pose_msg.pose.position.z = adjusted_pose.p.z();
        adjusted_pose_msg.pose.orientation.x = adjusted_pose.q.x();
        adjusted_pose_msg.pose.orientation.y = adjusted_pose.q.y();
        adjusted_pose_msg.pose.orientation.z = adjusted_pose.q.z();
        adjusted_pose_msg.pose.orientation.w = adjusted_pose.q.w();
        position_adjusted_icp.publish(adjusted_pose_msg);

    }else{
        ROS_ERROR("ICP did not converge");
        return;
    }

    
    // Publish the aligned point cloud
    // publish_pc<pcl::PointXYZ>(pc_ptr, header, icp_pc_pub);

    // Store the last aligned point cloud for the next iteration
    pcl::copyPointCloud(*pc_ptr, *last_merged_pc);

    // merge pc_merged_ptr and cloud_ptr
    *pc_merged_ptr += *pc_ptr;

    // Downsample the full point cloud
    voxel_grid.setDownsampleAllData(true);
    voxel_grid.setFilterLimits(-100, 100);
    voxel_grid.setMinimumPointsNumberPerVoxel(1);
    voxel_grid.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    voxel_grid.setSaveLeafLayout(false);
    voxel_grid.setInputCloud(pc_merged_ptr);
    voxel_grid.filter(*pc_merged_ptr);

    // Publish the filtered point cloud for visualization
    publish_pc<pcl::PointXYZ>(pc_merged_ptr, header, fused_pc_pub);
    ROS_INFO("Merged pc");

    //visualize_voxel_grid(voxel_grid, pc_merged_ptr, header);


    // Store the point clouds (PCD and PLY formats)
    // {
    //     // Store the merged point cloud to a file
    //     std::string filename = "merged_pc";
    //     pcl::io::savePCDFileASCII(filename+".pcd", *pc_merged_ptr);
    //     pcl::io::savePLYFileASCII(filename+".ply", *pc_merged_ptr);
    //     ROS_INFO("Saved merged point cloud to %s", filename.c_str());

    //     // Store the last merged point cloud to a file
    //     filename = "pc_"+ std::to_string(count);
    //     pcl::io::savePCDFileASCII(filename+".pcd", *pc_ptr);
    //     pcl::io::savePLYFileASCII(filename+".ply", *pc_ptr);
    //     ROS_INFO("Saved point cloud to %s", filename.c_str());
    // }

    //Save the drone positions
    savePositionsToFile(std::string(getenv("HOME")) + "/Documents/POSITION_trajV_mPc_icp3dof.csv", real_pose.p, adjusted_pose.p, noisy_pose.p);
}



int main(int argc, char** argv)
{

    ROS_INFO("Pointcloud alignment node starting");
    ros::init(argc, argv, "pointcloud_alignment_node");
    ros::NodeHandle nh;

    // Create a subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/tof_pc", 1, pointCloudCallback);

    // Listen to the real position for data collection purpuses
    // // Service client 
    // rosbag_playback_client = nh.serviceClient<std_srvs::SetBool>("/rosbag_node/pause_playback");
    // ROS_INFO("Waiting for rosbag pause service...");
    // rosbag_playback_client.waitForExistence();

    // Create a publisher for the output point cloud
    dwnsp_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/downsampled_pc", 100);
    fused_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/fused_pc", 100);

    // Publish the adjusted drone pose
    position_adjusted_icp = nh.advertise<geometry_msgs::PoseStamped>("/drone_pose/adjusted_icp", 1);

    //kp_scene_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/keypoints_scene_pc", 100);
    //kp_object_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/keypoints_object_pc", 100);
    //inliers_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/inliers_pc", 100);
    //corresp_pub = nh.advertise<visualization_msgs::Marker>( "/correspondences", 100);
    //icp_pc_pub   = nh.advertise<sensor_msgs::PointCloud2>("/icp_pc", 100);
    //noicp_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/noicp_pc", 100);
    //voxel_grid_pub = nh.advertise<visualization_msgs::Marker>( "/voxel_grid", 100);

    // Initialize the transform listener
    Eigen::Vector3f v; Eigen::Quaternionf q;
    get_tf_transform("map", "world", ros::Time::now(), v, q);

    ros::spin();

    return 0;
}
