#pragma once 

/**
 * This file was adapted from the RRT* implementation in the following repository:
 * https://github.com/nikhilchandak/Rapidly-Exploring-Random-Trees
*/

#include <math.h>
#include <cstdio>
#include <iostream>
#include <random>
#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

#define INF 1e18f
#define REFINEMENT 100 // Number of iterations for path refinement

using namespace std ; 


// Type of data type to be used for all calculations (Ex: long double)
#define ftype float  

/*  NOTE: Most of the calculations are done using EPS as a factor of difference
    since double/long double doesn't store floating point values precisely (limited precision) */
const ftype EPS = 1e-6;
const ftype r = 0.001 ; // resolution

/**
 * @brief A point in 3D space.
*/
struct Point {
    ftype x, y, z;
    Point() {}
    Point(ftype x, ftype y, ftype z): x(x), y(y), z(z) {}
    Point& operator+=(const Point &t) {
        x += t.x;
        y += t.y;
        z += t.z;
        return *this;
    }
    Point& operator-=(const Point &t) {
        x -= t.x;
        y -= t.y;
        z -= t.z;
        return *this;
    }
    Point& operator*=(ftype t) {
        x *= t;
        y *= t;
        z *= t;
        return *this;
    }
    Point& operator/=(ftype t) {
        x /= t;
        y /= t;
        z /= t;
        return *this;
    }
    Point operator+(const Point &t) const {
        return Point(*this) += t;
    }
    Point operator-(const Point &t) const {
        return Point(*this) -= t;
    }
    Point operator*(ftype t) const {
        return Point(*this) *= t;
    }
    Point operator/(ftype t) const {
        return Point(*this) /= t;
    }
    ftype dot(const Point &t) const {
    	return (x*t.x + y*t.y + z*t.z); 
    }
    Point cross(const Point& t) const { 
        return Point(y * t.z - z * t.y, z * t.x - x * t.z, x * t.y - y * t.x);
    }
    Point cross(const Point& a, const Point& b) const {  /// pas sur juste
        return (a - *this).cross(b - *this); 
    }
    ftype distance(const Point &t) const {
    	const double x_diff = x - t.x, y_diff = y - t.y, z_diff = z - t.z;
    	return sqrt(x_diff * x_diff + y_diff * y_diff +  z_diff * z_diff);
    }
    Point steer(const Point& t, ftype DELTA) {
	    if(this->distance(t) < DELTA) {
	        return t ; 
	    }
	    else {
	        double theta = atan2(t.y - y, t.x - x);
            double theta_z = atan2(t.z - z, distance(t));
	        return Point(x + DELTA * cos(theta), y + DELTA * sin(theta),z + DELTA*sin(theta_z));  // pas sur z a la fin
	    }
	}
    bool operator==(const Point& rhs) const
    {
        return fabs(x - rhs.x) < EPS and fabs(y - rhs.y) < EPS  and fabs(z - rhs.z) < EPS; // or another approach as above
    }
    
};

Point operator*(ftype a, Point b) {
    return b * a;
}

ftype distance(Point& a, Point &b) {
	const ftype x_diff = a.x - b.x, y_diff = a.y - b.y, z_diff = a.z - b.z; ; 
	return sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

ftype dot(Point a, Point b) {
	return (a.x*b.x + a.y*b.y + a.z*b.z);
}

Point cross(Point a, Point b) {
    return Point(a.y * b.z - b.y * a.z, a.z * b.x - b.z * a.x, a.x * b.y - b.x * a.y);
}

ftype norm(Point a){
    return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

/*  Returns a point in the direction of (p2 - p1) vector such that 
    the new point is within a DELTA distance of point1  */
Point stepNear(Point& p1, Point& p2, ftype DELTA) {
    if( (distance(p1, p2) - DELTA) <= EPS) 
        return p2 ; 
    else {
        ftype theta = atan2(p2.y - p1.y, p2.x - p1.x);
        ftype theta_z = atan2(p2.z - p1.z, distance(p1, p2));
        return Point(p1.x + DELTA * cos(theta), p1.y + DELTA * sin(theta),p1.z + DELTA * sin(theta_z));
    }
}



class RRTStarBase {

public: 

    // Status codes
    enum {IDLE=-3, FAILURE, RUNNING, SUCCESS, TIMEOUT, START_IN_OBS, STOP_IN_OBS};

    RRTStarBase(
            ros::NodeHandle& nh,
            float goal_radius,
            float goal_sampling_prob,
            float jump_size,
            float disk_size,
            float threshold_distance,
            float limit_x_low,
            float limit_x_high,
            float limit_y_low,
            float limit_y_high,
            float limit_z_low,
            float limit_z_high,
            float timeout
        ):marker_pub_(nh.advertise<visualization_msgs::Marker>("visualization_marker", 10)){

        // Parameters 
        goal_radius_ = goal_radius;
        goal_sampling_prob_ = goal_sampling_prob;
        jump_size_ = jump_size ;
        disk_size_ = disk_size;
        threshold_distance_ = threshold_distance;

        // Search limits
        limit_x_low_  = limit_x_low;
        limit_x_high_ = limit_x_high;
        limit_y_low_  = limit_y_low;
        limit_y_high_ = limit_y_high;
        limit_z_low_  = limit_z_low;
        limit_z_high_ = limit_z_high;

        // Search timeout
        timeout_ = timeout;

        nh.getParam("/model", model_);


        reset();
    }

    /**
     * @brief Clear the tree and reset all variables.
    */
    void reset() {
        pathCoordinates_.clear();
        nodes_.clear();
        parent_.clear();
        nearby_.clear();
        cost_.clear();
        jumps_.clear();
        nodeCnt_    =  0;
        goalIndex_  = -1;
        closestIndex_ = -1;
        pathFound_  =  false;
    }

    void init_tree(Point start) {
        nodes_.push_back(start); 
        parent_.push_back(0); 
        cost_.push_back(0); 
        jumps_.push_back(0);
        nodeCnt_++;
    }

    void set_timeout(float timeout) {
        timeout_ = timeout;
    }

    std::vector<Point> get_nodes() {
        return nodes_;
    }

    /**
     * @brief Run an iteration of the RRT* algorithm. One node is added per iteration. 
     * @param start The start point of the path.
     * @param stop The goal point of the path.
     * @param startTime The time at which the algorithm started (in seconds).
     * @param timeout A boolean that is set to true if the algorithm times out while finding a path.
     */
    void RRT(Point& start, std::vector<Point> stops, double startTime, bool& timeout,
            const Eigen::Vector3d& cur_pos) {
        
        Point newPoint, nearestPoint, nextPoint ; bool updated = false ; int cnt = 0 ; 
        int nearestIndex = 0 ; double minCost = INF; nearby_.clear(); jumps_.resize(nodeCnt_); 

        // Check timeout (needed if there exists no path from start to stop)
        double ellapsed = getCurrentTime()-startTime; 
        if(ellapsed > timeout_){
            timeout = true;
            return;
        }

        while(!updated) {
            newPoint = pickRandomPoint(cur_pos); // custom (not robust)
            
            // Find nearest point to the newPoint such that the next node 
            // be added in graph in the (nearestPoint, newPoint) while being obstacle free
            nearestPoint = *nodes_.begin(); nearestIndex = 0;
            for(int i = 0; i < nodeCnt_; i++) {
                if(pathFound_ and randomCoordinate(0.0, 1.0) < 0.25) // Recalculate cost once in a while 
                    cost_[i] = cost_[parent_[i]] + distance(nodes_[parent_[i]], nodes_[i]);  

                // Make smaller jumps sometimes to facilitate passing through narrow passages 
                jumps_[i] = randomCoordinate(0.3, 1.0) * jump_size_ ; 
                auto pnt = nodes_[i] ; 
                if((pnt.distance(newPoint) - nearestPoint.distance(newPoint)) <= EPS and isEdgeObstacleFree(pnt, pnt.steer(newPoint, jumps_[i])))
                    nearestPoint = pnt, nearestIndex = i ; 
            }
            
            nextPoint = stepNear(nearestPoint, newPoint, jumps_[nearestIndex]);
            
            if(!isEdgeObstacleFree(nearestPoint, nextPoint)){
                // Do not run forever 
                double stopTime = getCurrentTime();
                double duration = stopTime - startTime; 
                // printf("Search timeout: %lf, current duration: %lf\n", timeout_, duration); 
                if (duration > timeout_){ //if algo gets stuck, stop it
                    timeout = true;
                    return;
                }
                continue; 
            }

            // Find nearby nodes to the new node as center in ball of radius DISK_SIZE 
            for(int i = 0; i < nodeCnt_; i++)
                if((nodes_[i].distance(nextPoint) - disk_size_) <= EPS and isEdgeObstacleFree(nodes_[i], nextPoint))
                    nearby_.push_back(i);

            // Find minimum cost path to the new node 
            int par = nearestIndex; minCost = cost_[par] + distance(nodes_[par], nextPoint);
            for(auto nodeIndex: nearby_) {
                if( ( (cost_[nodeIndex] + distance(nodes_[nodeIndex], nextPoint)) - minCost) <= EPS)
                    minCost = cost_[nodeIndex] + distance(nodes_[nodeIndex], nextPoint), par = nodeIndex;
            }

            parent_.push_back(par); 
            cost_.push_back(minCost);
            nodes_.push_back(nextPoint); 
            visualizeTree();
            nodeCnt_++; 
            updated = true ; 
            if(!pathFound_) checkDestinationReached(start, stops); 
            rewire();

        }
    }

void visualizeTree()
{
 
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "tree";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.005;  
    marker.color.r = 1.0;  
    marker.color.g = 1.0; 
    marker.color.a = 0.5;  
    marker.pose.orientation.w = 1.0;

    std::vector<Point> nodes = nodes_;
    std::vector<int> parent = parent_;

    for (size_t i = 0; i < nodes_.size(); ++i)
    {
        if (parent_[i] != -1) 
        {
            geometry_msgs::Point p1, p2;
            p1.x = nodes[i].x;
            p1.y = nodes[i].y;
            p1.z = nodes[i].z;

            p2.x = nodes[parent[i]].x;
            p2.y = nodes[parent[i]].y;
            p2.z = nodes[parent[i]].z;

            marker.points.push_back(p1);
            marker.points.push_back(p2);
        }
    }

    marker_pub_.publish(marker);
}


    /**
     * @brief Find a path from start to stop.
     * @param start The start point of the path.
     * @param stops The goal points of the path.
     * @param pathCoordinates The vector in which the path will be stored.
     * @return An integer that indicates the status of the algorithm 
     *          (SUCCESS=0, TIMEOUT=1, START_IN_OBS=2, STOP_IN_OBS=3).
    */
    int find_path(Point start, std::vector<Point> stops, vector<Point>& pathCoordinates) {

        pathCoordinates.clear();

        /////////////////////
        // Pre RRT* checks //
        /////////////////////
     

        // Check if start and is obstacle free
        if(!isObstacleFree(start)){
            return START_IN_OBS;
        }

        // Check if any of the stops are not obstacle free (and remove them if so)
        std::vector<int> stopInObs;
        for(int i=0; i<stops.size(); i++){
            if(!isObstacleFree(stops[i])){
                stopInObs.push_back(i);
            }
        }
        for(int i=stopInObs.size()-1; i>=0; i--){
            stops.erase(stops.begin() + stopInObs[i]);
        }
        if(stops.empty()){
            return STOP_IN_OBS;
        }

        // Check if start and a stop are the same
        for(auto stop : stops){
            if(start.distance(stop) < r){
                pathCoordinates.push_back(start);
                return SUCCESS;
            }
        }

        // Check if there is a line of sight between start and a stop
        for(auto stop : stops){
            if(isEdgeObstacleFree(start, stop)){
                pathCoordinates.push_back(start);
                pathCoordinates.push_back(stop);
                return SUCCESS;
            }
        }

     
        //////////////////////
        // Need to run RRT* //
        //////////////////////

        reset(); 
        init_tree(start);

        // Search for a path 
        bool timeout = false;
        double startTime = getCurrentTime();
        while(!pathFound_ && !timeout) {
            Eigen::Vector3d cur_pos = getCurPos();
            RRT(start, stops, startTime, timeout, cur_pos);
        }
        if(timeout){
            // Return path to node closest to goal
            backtrack_from_closest(start);
        }

        pathCoordinates = pathCoordinates_;

        // Prune the path
        if(pathCoordinates.size() > 0) prune_path(pathCoordinates);

        return timeout ? TIMEOUT : SUCCESS;
    }

    /**
     * @brief Get the current time in seconds.
     * @return The current time in seconds.
    */
    virtual double getCurrentTime() = 0;

    /**
     * @brief Check if a point is obstacle free.
     * @param p The point to be checked.
     * @return True if the point is obstacle free, false otherwise.
    */
    virtual bool isObstacleFree(Point p) = 0;
    
    /**
     * @brief Check if an edge is obstacle free.
     * @param a The first point of the edge.
     * @param b The second point of the edge.
     * @return True if the edge is obstacle free, false otherwise.
    */
    virtual bool isEdgeObstacleFree(Point a, Point b) = 0;

    /**
     * @brief Get the robot 3d position
     * @return the current pos of robot
    */
    virtual Eigen::Vector3d getCurPos() = 0;

protected: 

    /**
     * @brief Prune the path by removing unnecessary waypoints.
     * @param pathCoordinates The path to be pruned.
    */
    void prune_path(vector<Point>& pathCoordinates){

        // Remove consecutive points that are too close 
        vector<Point> simpPath;
        for(int i=0; i<pathCoordinates.size(); i++){
            if(i == 0 or i == pathCoordinates.size()-1 or distance(pathCoordinates[i], pathCoordinates[i-1]) > r){
                simpPath.push_back(pathCoordinates[i]);
            }
        }
        pathCoordinates = simpPath;

        // No further refinement for small paths
        if(pathCoordinates.size() < 3) return ; 

        // Prune the path where line of sight is clear
        vector<Point> prunedPath ; 
        prunedPath.push_back(pathCoordinates[0]); 
        int i = 0, j = 1, k = 2 ; 
        while(k < pathCoordinates.size()) {
            if(isEdgeObstacleFree(pathCoordinates[i], pathCoordinates[k])) {
                j = k ; 
            }
            else {
                prunedPath.push_back(pathCoordinates[j]); 
                i = j ; 
            }
            k++ ; 
        }
        prunedPath.push_back(pathCoordinates.back()); 
        pathCoordinates = prunedPath ;
    }

    template <typename T> 
    double randomCoordinate(T low, T high){
        static random_device random_device;
        static mt19937 engine{random_device()};
        uniform_real_distribution<double> dist(low, high);
        return dist(engine);
    }

    // custom implementation
    Point pickRandomPoint(const Eigen::Vector3d& cur_pos){
        
        static random_device random_device;
        static mt19937 engine{random_device()};

        Point  minCoord(limit_x_low_,limit_y_low_,limit_z_low_), 
               maxCoord(limit_x_high_,limit_y_high_,limit_z_high_);


        ftype offset = 0.0f; 
        std::uniform_real_distribution<float> dist_x(minCoord.x - offset, maxCoord.x + offset);
        std::uniform_real_distribution<float> dist_y(minCoord.y - offset, maxCoord.y + offset);
        ftype x = dist_x(engine);
        ftype y = dist_y(engine);
        ftype z;

        if(model_ == 0){
            // uniform distribution
            std::uniform_real_distribution<float> dist_z(minCoord.z - offset, maxCoord.z + offset);
            z = dist_z(engine);
        }else{
            // normal_distribution for z scale
            float current_z = cur_pos.z();
            float std_dev_z = 5.0;
            std::normal_distribution<float> dist_z(current_z, std_dev_z);
            z = dist_z(engine);
        }

        x > limit_x_high_ ? x = limit_x_high_ : x < limit_x_low_ ? x = limit_x_low_ : x = x;
        y > limit_y_high_ ? y = limit_y_high_ : y < limit_y_low_ ? y = limit_y_low_ : y = y;
        z > limit_z_high_ ? z = limit_z_high_ : z < limit_z_low_ ? z = limit_z_low_ : z = z;

        return Point(x,y,z);
    }

    /**
     * @brief Update the path coordinates by backtracking from the node at index idx.
     *        This can be used to generate a path even if the goal was not reached.
     * @param start The start point of the path.
     * @return Nothing, but the path coordinates are updated (pathCoordinates_).
    */
    void backtrack_from_closest(Point start){

        int idx = closestIndex_;

        while(parent_[idx] != idx){
            pathCoordinates_.push_back(nodes_[idx]);
            idx = parent_[idx];
        }
        pathCoordinates_.push_back(start); 
        std::reverse(pathCoordinates_.begin(), pathCoordinates_.end());
    } 

    bool checkDestinationReached(Point start, std::vector<Point> stops) {

        // Initialize the closest index and distance to goal
        static ftype minDist = INF;
        if(closestIndex_ == -1){
            minDist = INF;
            closestIndex_ = 0;
        }

        // Check if the new node is closest to the goal and store it if so 
        for(Point stop : stops){
            if(norm(nodes_.back() - stop) < minDist){
                minDist = norm(nodes_.back() - stop);
                closestIndex_ = nodeCnt_ - 1;
            }
        }


        bool lineOfSight = false; 
        Point stop; 
        for(auto pnt: stops) {
            if(isEdgeObstacleFree(nodes_.back(), pnt)) {
                lineOfSight = true; 
                stop = pnt; 
                break; 
            }
        }
        
        // In case a line of sight is found, backtrack to find the path
        if(lineOfSight) {
            pathFound_ = true;
            goalIndex_ = nodeCnt_ - 1;
            int node = goalIndex_;
            while (parent_[node] != node) {
                pathCoordinates_.push_back(nodes_[node]);
                node = parent_[node];
            }
            pathCoordinates_.push_back(start); 
            std::reverse(pathCoordinates_.begin(), pathCoordinates_.end()); // Reverse the vector to get the correct order
            pathCoordinates_.push_back(stop);
            return true;
        } else {
            return false;
        }
    }

    void rewire() {
        int lastInserted = nodeCnt_ - 1 ; 
        for(auto nodeIndex: nearby_) {
            int par = lastInserted, cur = nodeIndex;

            // Rewire parents as much as possible (greedily)
            while( ((cost_[par] + distance(nodes_[par], nodes_[cur])) - cost_[cur]) <= EPS) { 
                int oldParent = parent_[cur] ;
                parent_[cur] = par; cost_[cur] = cost_[par] + distance(nodes_[par], nodes_[cur]);
                par = cur, cur = oldParent; 
            }
        }
    }

    // Parameters
    float goal_radius_;         // radius around the goal to consider it reached 
    float goal_sampling_prob_;  // probability of sampling the goal point
    float jump_size_;           // Jump distance max for RRTstar
    float disk_size_;           // Ball radius around which nearby points are found 
    float threshold_distance_;  // Distance threshold for collision checking
    float limit_x_low_; 
    float limit_x_high_;
    float limit_y_low_; 
    float limit_y_high_;
    float limit_z_low_; 
    float limit_z_high_;
    float timeout_;             // Timeout for RRTstar
    int model_;

    ros::Publisher marker_pub_;

    // Variables
    std::vector<Point>  pathCoordinates_;   // Stores waypoints
    vector < Point >    nodes_ ;            // Stores nodes of the tree
    vector < int >      parent_,            // Stores parent of each node
                        nearby_ ;           // Stores nearby nodes of each node
    vector < double >   cost_,              // Stores cost of each node
                        jumps_ ;            // Stores jump distance of each node
    int                 nodeCnt_,           // Node count in the tree
                        goalIndex_,         // Stores number of nodes and index of goal node
                        closestIndex_;      // Stores index of closest node to goal
    bool                pathFound_;         // Stores whether goal has been reached or not
};