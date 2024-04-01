/* Developer: Seyi R. Afolayan
 * Work: Bidirectional EST (BiEST) --> Implementing the skeleton functions in the header file
 * Course: Algorithm for Sensor-Based Robotics
*/
/************************************************BEGIN CODE**************************************/
// Libraries and header files
#include "assignment3_context.h"
#include <iostream>
#include <moveit/planning_scene/planning_scene.h>
#include <random>
#include <cstdint>
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <limits>

/*************************************************BEGIN CODE**********************************************************/
// Random Number Generation
std::random_device ASBRContext::randomDevice;
std::mt19937 ASBRContext::generator(ASBRContext::randomDevice());

ASBRContext::ASBRContext( const moveit::core::RobotModelConstPtr& robotmodel,
                          const std::string& name,
                          const std::string& group ):
        planning_interface::PlanningContext( name, group ),
        robotmodel( robotmodel ){}

ASBRContext::~ASBRContext(){}

/*******************************************CONFIGURATION STATE COLLISION**********************************************/
bool ASBRContext::state_collides( const vertex& q ) const {

    // create a robot state
    moveit::core::RobotState robotstate( robotmodel );
    robotstate.setJointGroupPositions( "manipulator", q );

    if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
    else
    { return false; }

}

/****************************************************INTERPOLATION*****************************************************/
ASBRContext::vertex ASBRContext::interpolate( const ASBRContext::vertex& qA,
                                              const ASBRContext::vertex& qB,
                                              double t ){

    ASBRContext::vertex qt( qA.size(), 0.0 );
    for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
    return qt;
}

/**********************************************RANDOM CONFIGURATION SELECTION******************************************/
/**
 * The essence of this method is to try to generate configuration of lesser explored areas. I am going to use the
 * weights to return the index of a random configuration in the tree.
 * @param w
 * @return
 */
ASBRContext::index ASBRContext::select_config_from_tree(const std::vector <weight> &w) {
     // We check if the weight vector is empty
     if (w.empty())
         throw std::runtime_error("The weight vector is empty...\nNo configuration selected from tree");
     // Creating a bucket to store the weights uniformly
     static std::uniform_real_distribution<> uniform_distro(0.0, std::accumulate(w.begin(), w.end(), 0.0));
     double random_point{uniform_distro(generator)}; // This generates a random point (draws a random weight from the bucket)
     // Using roulette wheel sampling logic
     double cum_weight{0.0};
     ASBRContext::index i{0};
     for (; i < w.size(); ++i){
         cum_weight += w[i]; // increments the weight of the bucket
         if (random_point <= cum_weight)
             break;
     }
     // Bounds check --> if the iterator is kinda outside the range due to floating-point issues, return the last i
     if ( i >= w.size())
         i = w.size() - 1;
     // return the index
     return i;
}

/*********************************************SAMPLE NEARBY************************************************************/
/***
 * In this method, the logic here is to determine what nearby configuration is collision-free and close to the current
 * configuration state. I do this by sampling the configuration space using an assumption that the ur5 is from -pi to pi
 * and also using some sort of goal biasing techniques to force it to the connection point in both trees 1 and 2.
 * @param q
 * @param q_goal
 * @return
 */
ASBRContext::vertex ASBRContext::sample_nearby(const ASBRContext::vertex &q, const ASBRContext::vertex &q_goal) {
    // Creating a normal distribution for the generator using a mean of zero and standard deviation of pi. This is
    // chosen because the ur5 workspace is arguably -pi to pi. I think this allows me to explore the workspace entirely.
    static std::normal_distribution<> gaussian_generator(0, M_PI);
    static std::uniform_real_distribution<double> goal_random(0, 1); // generating doubles between 0 and 1.
    const double GOAL_BIASING{0.185}; // Setting a reasonable cheat code to optimize my BiEST path planner (TUNED).

    // Goal Biasing check --> I implement a goal-biasing feature where there is an 18% chance that the algorithm will
    // directly attempt to use the goal configuration if it's valid, potentially speeding the pathfinding process.
    if (goal_random(generator) < GOAL_BIASING && !state_collides(q_goal))
        return q_goal;

    // Main logic --> I am trying to find sample nearby the input parameter q that is not in any sort of collision.
    // Initializations for q_rand and the limit
    ASBRContext::vertex q_rand(q.size());
    std::uint16_t tries{0};
    const uint16_t MAX_TRIES{1000};

    do {
        for (std::size_t i{0}; i < q.size(); ++i){
            /**
             * In this for loop, I generate a deviation from the initial configuration q in hopes of finding a 6 by 1
             * vector that is collision free. However, the result of this deviation might be outside the ur5 workspace
             * range. So I apply a modulus operation to ensure that the result is within 0 to 2pi. Once I am sure of
             * this, I added a -pi offset to return it back to the ur5 workspace limit which is -pi to pi.
           **/
            double noise{gaussian_generator(generator)};
            q_rand[i] = std::fmod(q[i] + noise + M_PI, 2 * M_PI) - M_PI;
        }
        if (!state_collides(q_rand))
            return q_rand;
        ++tries;
    } while (tries < MAX_TRIES);

    // Else, if my algorithm does not find a good q_rand within max attempts, I return current configuration.
    return q;
}

/*********************************************LOCAL PATH COLLISION CHECK***********************************************/
/***
 * In this method, I need to check if the path(linear or curve) between a q and a q_rand is collision free. I do this
 * by using spherical interpolation to check if there collision within the local path.
 * @param q
 * @param q_rand
 * @return
 */
bool ASBRContext::is_local_path_collision_free( const ASBRContext::vertex& q,
                                                const ASBRContext::vertex& q_rand ){

    // TODO find if the straight line path between q_near and q_rand is collision free

    // Quick sanity check --> if either q_near (which is q at this time) or q_rand or both are in collision, return false.
    if (state_collides(q) || state_collides(q_rand))
        return false;
    // Else, we divide the path into segments
    double interpolation_resolution{0.005}, t{interpolation_resolution}; // I changed the resolution from 0.01 --> 0.005
    for (; t < 1; t += interpolation_resolution ){
        ASBRContext::vertex qt = interpolate(q, q_rand, t);
        // I check if there is any collision along this local path discretely
        if (state_collides(qt))
            return false;
    }
    return true;
}

/**************************************SEARCH PATH ---> ITERATIVE BACKTRACKING ALGORITHM*******************************/
/***
 * This logic here uses deterministic Iterative Backtracking algorithm. This can also be done recursively as well using
 * the Recursive Backtracking algorithm. I say deterministic as we already know the goal and have the connected path.
 * We just need to backtrack from the goal vertex to the parent vertex.
 * @param V
 * @param parent
 * @param idx_init
 * @param idx_goal
 * @return P ---> The connected path.
 */
ASBRContext::path ASBRContext::search_path(const std::vector<vertex>& V, // Vertices in the tree
                                           const std::vector<index>& parent, // Index of the parent vertex in the tree
                                           const index& idx_init, // Index of the initial vertex in the tree
                                           const index& idx_goal) // Index of the final vertex in the tree.
{
    // Declaration of the Path
    ASBRContext::path P; // This will  be used to return the path from initial index to the goal index in the tree

    // Sanity check ---> checking the bounds of the indices and vertices
    if (idx_init >= V.size() || idx_goal >= V.size() || idx_goal >= parent.size())
        return P;

    // So I have to check the nodes visited to prevent a cyclic search of my tree
    std::vector<bool> visited(V.size(), false); // Dynamically allocating memory to a vector of boolean values.
    ASBRContext::index current_idx{idx_goal}; // The current index will be the goal index for my backtracking

    // Logic ---> while I am not at the vertex node, I want to backtrack from the parent node of the goal recursively.
    while (current_idx != idx_init) {
        // Sanity check --> I check either the current vertex has already been visited or out of bounds or both.
        if (visited[current_idx]||current_idx >= V.size())
            return {}; // returns a null or empty path.
        // Else, lets append the vertex of the current index to the path.
        P.push_back(V[current_idx]); // Since, we are iteratively backtracking, this will initially be the goal vertex.
        visited[current_idx] = true; // I set the visiting index to visited at this point.
        // This is the BACKTRACK!!!!
        current_idx = parent[current_idx]; // Move the parent node to current node.
    }
    // When the current index is the same as the initial index, append it to the tree.
    P.push_back(V[idx_init]); // Add the initial configuration
    std::reverse(P.begin(), P.end()); //  Reverse the backtracked path.

    return P;
}

/*************************************************NEAREST CONFIGURATION************************************************/
/**
 * The logic in this method is to find the nearest configuration to the current tree (first or second). I do this by
 * using the Euclidean Norm and getting the the configuration with the smallest distance to the current tree. But, the
 * Euclidean Norm is indeed an educated approximation for measuring distances in the configuration space (C-space).
 * @param V
 * @param q
 * @return the minimum distance (the index of it).
 */
ASBRContext::index ASBRContext::find_nearest_configuration(const std::vector<ASBRContext::vertex>& V,
                                                           const ASBRContext::vertex& q) {
    if (V.empty()) {
        // Return an invalid index to indicate that V is empty
        return std::numeric_limits<ASBRContext::index>::max();
    }
    // Initializations of the distance and the index
    double minimum_euclidean_distance{std::numeric_limits<double>::max()};
    ASBRContext::index minimum_index{std::numeric_limits<ASBRContext::index>::max()};

    for (size_t i{0}; i < V.size(); ++i){
        double euclidean_distance = euclidean_norm(V[i], q);
        // Update the Euclidean distance with the smallest distance.
        if (euclidean_distance < minimum_euclidean_distance) {
            minimum_euclidean_distance = euclidean_distance;
            minimum_index = i; // updates the minimum index
        }
    }
    return minimum_index;
}

/***************************************************HELPER FUNCTIONS***************************************************/
// TODO ---> Implement a boolean generator or a boolean alternator
// The alternating approach was much more efficient with a minimum time of 90ms to plan for the bugatti.
bool ASBRContext::random_bool() {
    static bool currentValue = false; // Initialize to the opposite of what you want first
    currentValue = !currentValue; // Toggle the value
    return currentValue;
}
// The minimum time to plan for the boolean generator was 351ms for the bugatti.
//bool ASBRContext::random_bool() {
//    static std::uniform_int_distribution<> dis(0, 1);
//    return dis(generator);
//}
// TODO --> implement the euclidean norm helper function
double ASBRContext::euclidean_norm(const ASBRContext::vertex &q1, const ASBRContext::vertex &q2) {
    double euclidean_total{0.0};
    for (size_t i{0}; i < q1.size(); ++i){
        double diff = q2[i] - q1[i];
        euclidean_total += diff * diff;
    }
    return std::sqrt(euclidean_total);
}
//
//// TODO ---> implement the extend function
//bool ASBRContext::extend(std::vector<ASBRContext::vertex>& V_from, std::vector<index>& parent_from,
//                         std::vector<ASBRContext::weight>& w_from, ){
//
//}

ASBRContext::path ASBRContext::est(const ASBRContext::vertex& q_init, const ASBRContext::vertex& q_goal) {
    // Disclaimer
    std::cout << "When this Bidirectional Expansive Space Algorithm was first penned by yours truly, both God"
                 " and I understood its workings.\nNow, it seems God alone holds that knowledge." << std::endl;
    ASBRContext::path P;
    std::cout << "Starting EST with q_goal: ";
    for (const auto& coord : q_goal) std::cout << coord << " ";
    std::cout << std::endl;

    std::vector<ASBRContext::vertex> V_root{q_init}, V_goal{q_goal};
    std::vector<ASBRContext::index> parent{0}, parent_goal{0};
    std::vector<ASBRContext::weight> w{1.0}, w_goal{1.0};

    const int MAX_ITERATIONS = 10000;
    double nearThresholdVal = 18;
    bool isPath = false;
    long connectionPointRootIdx = -1;
    long connectionPointGoalIdx = -1;

    for (int i = 0; i < MAX_ITERATIONS; ++i) {
        std::cout << "Iteration: " << i << std::endl;

        if (random_bool()) {
            ASBRContext::index idx_q_rand_root = select_config_from_tree(w);
            ASBRContext::vertex q_rand = sample_nearby(V_root[idx_q_rand_root], q_goal);
            std::cout << "Selected q_rand near q_init: ";
            for (const auto& coord : q_rand) std::cout << coord << " ";
            std::cout << std::endl;

            ASBRContext::index idx_q_near_root = find_nearest_configuration(V_root, q_rand);
            if (is_local_path_collision_free(V_root[idx_q_near_root], q_rand)) {
                V_root.push_back(q_rand);
                parent.push_back(idx_q_near_root);
                w.push_back(1.0 / (1.0 + w[idx_q_near_root]));

                ASBRContext::index idx_q_near_goal = find_nearest_configuration(V_goal, q_rand);
                double distance = euclidean_norm(V_goal[idx_q_near_goal], q_rand);
                std::cout << "The distance: " << distance << std::endl;
                if (euclidean_norm(q_rand, V_goal[idx_q_near_goal]) < nearThresholdVal &&
                    is_local_path_collision_free(q_rand, V_goal[idx_q_near_goal])) {
                    V_root.push_back(V_goal[idx_q_near_goal]);
                    connectionPointRootIdx = V_root.size() - 2; // size len - 1, last node.. we ne
                    connectionPointGoalIdx = idx_q_near_goal;
                    isPath = true;
                    std::cout << "Path found!" << std::endl;
                    break;
                }
            }
        } else {
            ASBRContext::index idx_q_rand_goal = select_config_from_tree(w_goal);
            ASBRContext::vertex q_rand = sample_nearby(V_goal[idx_q_rand_goal], q_init);
            std::cout << "Selected q_rand near q_goal: ";
            for (const auto& coord : q_rand) std::cout << coord << " ";
            std::cout << std::endl;

            ASBRContext::index idx_q_near_goal = find_nearest_configuration(V_goal, q_rand);
            if (is_local_path_collision_free(V_goal[idx_q_near_goal], q_rand)) {
                parent_goal.push_back(idx_q_near_goal);
                V_goal.push_back(q_rand);
                w.push_back(1.0 / (1.0 + w_goal[idx_q_near_goal]));

                ASBRContext::index idx_q_near_root = find_nearest_configuration(V_root, q_rand);
                double distance2 = euclidean_norm(V_root[idx_q_near_root], q_rand);
                std::cout << "The distance: " << distance2 << std::endl;
                if (euclidean_norm(q_rand, V_root[idx_q_near_root]) < nearThresholdVal &&
                    is_local_path_collision_free(q_rand, V_root[idx_q_near_root])) {
                    V_goal.push_back(V_root[idx_q_near_root]);
                    connectionPointGoalIdx = V_goal.size() - 2;
                    connectionPointRootIdx = idx_q_near_root;
                    isPath = true;
                    std::cout << "Path found!" << std::endl;
                    break;
                }
            }
        }
    }

    if (isPath) {
        std::cout << "Constructing path from connection points." << std::endl;
        ASBRContext::path P_root = search_path(V_root, parent, 0, connectionPointRootIdx);
        ASBRContext::path P_goal = search_path(V_goal, parent_goal, 0, connectionPointGoalIdx);
        P_root.insert(P_root.end(), P_goal.rbegin(), P_goal.rend());
        std::cout << "Path length: " << P_root.size() << std::endl;
        return P_root;
    } else {
        std::cout << "No path found after " << MAX_ITERATIONS << " iterations." << std::endl;
        P = {};
        return P;
    }
}


bool ASBRContext::solve( planning_interface::MotionPlanResponse &res ){

    // Create a new empty trajectory
    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel,
                                                                getGroupName()));
    res.trajectory_->clear();

    // copy the initial/final joints configurations to vectors qfin and qini
    // This is mainly for convenience.
    std::vector<double> qstart, qfinal;

    for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
        qfinal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
        qstart.push_back(request_.start_state.joint_state.position[i]);
    }

    // start the timer
    rclcpp::Clock clock;
    rclcpp::Time t1 = clock.now();
    path P = est( qstart, qfinal );
    rclcpp::Time t2 = clock.now();
    std::cout << "Your path has length " << P.size() << std::endl;
    // end the timer

    // The rest is to fill in the animation.
    moveit::core::RobotState robotstate( robotmodel );
    robotstate.setJointGroupPositions( "manipulator", qstart );
    res.trajectory_->addSuffixWayPoint( robotstate, 0.001 );

    for( std::size_t i=1; i<P.size(); i++ ){
        for( double t=0.0; t<=1.0; t+=0.01 ){
            vertex q = interpolate( P[i-1], P[i], t );
            robotstate.setJointGroupPositions( "manipulator", q );
            res.trajectory_->addSuffixWayPoint( robotstate, 0.001 );
        }
    }

    //
    rclcpp::Duration planning_time = t2-t1;
    res.planning_time_ = planning_time.seconds();
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    return true;

}

bool ASBRContext::solve( planning_interface::MotionPlanDetailedResponse& )
{ return true; }

void ASBRContext::clear(){}

bool ASBRContext::terminate(){return true;}