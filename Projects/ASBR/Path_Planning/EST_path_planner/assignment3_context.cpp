/* Developer: Seyi R. Afolayan
 * Work: Implementing the skeleton functions in the header file
 * Course: Algorithm for Sensor-Based Robotics
*/
/************************************************BEGIN CODE**************************************/

// Libraries and header files
#include "assignment3_context.h"
#include <moveit/planning_scene/planning_scene.h>
#include <random> // added to generate random numbers
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <limits>


ASBRContext::ASBRContext( const moveit::core::RobotModelConstPtr& robotmodel,
                          const std::string& name,
                          const std::string& group ):
        planning_interface::PlanningContext( name, group ),
        robotmodel( robotmodel ){}

ASBRContext::~ASBRContext(){}

bool ASBRContext::state_collides( const vertex& q ) const {

    // create a robot state
    moveit::core::RobotState robotstate( robotmodel );
    robotstate.setJointGroupPositions( "manipulator", q );

    if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
    else
    { return false; }

}

/******************************************INTERPOLATION*****************************************/
ASBRContext::vertex ASBRContext::interpolate( const ASBRContext::vertex& qA,
                                              const ASBRContext::vertex& qB,
                                              double t ){

    ASBRContext::vertex qt( qA.size(), 0.0 );
    for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
    return qt;
}

/**********************************RANDOM CONFIGURATION SELECTION***********************************/
ASBRContext::index ASBRContext::select_config_from_tree(const std::vector<ASBRContext::weight>& w) {
    if (w.empty()) {
        throw std::runtime_error("Weight vector is empty. Cannot select a configuration.");
    }

    // Initialize the Mersenne Twister pseudo-random number generator with a random seed
    // Consider initializing this outside the function if called frequently for performance
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // Construct a uniform real distribution from 0 to the sum of weights
    static std::uniform_real_distribution<> dis(0.0, std::accumulate(w.begin(), w.end(), 0.0));

    // Generate a random point within the distribution
    double random_point = dis(gen);

    // Use cumulative weights to find the corresponding index
    double cum_weight = 0.0;
    ASBRContext::index i = 0;
    for (; i < w.size(); ++i) {
        cum_weight += w[i];
        if (random_point <= cum_weight) {
            break;
        }
    }

    // Ensure i does not exceed the bounds of w
    if (i >= w.size()) {
        i = w.size() - 1; // Use the last index as a fallback
    }

    return i; // Return the selected index
}

///****************************************SAMPLE NEARBY**********************************************/
ASBRContext::vertex ASBRContext::sample_nearby(const ASBRContext::vertex& q, const ASBRContext::vertex& q_goal) {
    static std::random_device random_num;
    static std::mt19937 gen(random_num());
    static std::normal_distribution<> gaussian_generator(0, M_PI);  // Adjust stddev to control deviation
    static std::uniform_real_distribution<double> goal_random(0, 1);
    const double GOAL_BIAS{0.30};

    // Check goal bias first; if not colliding with the goal directly, return goal configuration
    if (goal_random(gen) < GOAL_BIAS && !state_collides(q_goal)) {
        return q_goal;
    }

    ASBRContext::vertex q_rand(q.size());
    int attempts = 0;
    const int MAX_ATTEMPTS = 1000;

    do {
        for (size_t i = 0; i < q.size(); ++i) {
            double noise = gaussian_generator(gen);
            // Normalize sampled angle within [-pi, pi]
            double new_angle = std::fmod(q[i] + noise + M_PI, 2 * M_PI) - M_PI;
            q_rand[i] = new_angle;
        }

        if (!state_collides(q_rand)) {
            return q_rand;  // Return the first non-colliding configuration found
        }

        ++attempts;
    } while (attempts < MAX_ATTEMPTS);

    // If a non-colliding configuration isn't found within the max attempts, fallback to the initial configuration
    return q;
}



/**********************************LOCAL PATH COLLISION CHECK**************************************/
// TODO
bool ASBRContext::is_local_path_collision_free( const ASBRContext::vertex& q,
                                                const ASBRContext::vertex& q_rand ){

    // TODO find if the straightline path between q_near and q_rand is collision free
    // First check
    if (state_collides(q) || state_collides(q_rand))
        return false;

    double interpolation_resolution = 0.005; // resolution definition to work somewhat like discreet steps
    for (double t{interpolation_resolution}; t < 1.0; t += interpolation_resolution) {
        ASBRContext::vertex qt = interpolate(q, q_rand, t);
        // Hypothetical to check if discrete is in collision with some weird obstacle
        if (state_collides(qt))
            return false; //
    }
    // Else, qt at t is not in collision
    return true;
}

ASBRContext::path ASBRContext::search_path(const std::vector<vertex>& V,
                                           const std::vector<index>& parent,
                                           const index& idx_init,
                                           const index& idx_goal) {
    ASBRContext::path P;

    if (idx_init >= V.size() || idx_goal >= V.size() || idx_goal >= parent.size()) {
        // Return an empty path if indices are invalid or if idx_goal has no parent
        return P;
    }

    std::vector<bool> visited(V.size(), false);  // To prevent cycles
    index current_idx = idx_goal;

    while (current_idx != idx_init) {
        if (visited[current_idx] || current_idx >= V.size()) {
            // Return an empty path if a cycle is detected or index is out of bounds
            return {};
        }
        P.push_back(V[current_idx]);
        visited[current_idx] = true;
        current_idx = parent[current_idx];  // Move to the parent of the current node
    }

    P.push_back(V[idx_init]);  // Add the initial configuration

    std::reverse(P.begin(), P.end());  // Reverse to get path from start to goal
    return P;
}

/***************************************NEAREST CONFIGURATION************************************************/
ASBRContext::index ASBRContext::find_nearest_configuration(const std::vector<ASBRContext::vertex>& V, const ASBRContext::vertex& q) {
    if (V.empty()) {
        // Return an invalid index to indicate that V is empty
        return std::numeric_limits<ASBRContext::index>::max();
    }

    double min_dist = std::numeric_limits<double>::max();
    ASBRContext::index min_idx = std::numeric_limits<ASBRContext::index>::max();

    for (size_t i = 0; i < V.size(); ++i) {
        double dist = 0.0;
        for (size_t j = 0; j < q.size(); ++j) {
            double diff = V[i][j] - q[j];
            dist += diff * diff; // Use direct multiplication for squaring
        }

        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }

    return min_idx;
}


// function to generate random bool
bool ASBRContext::random_bool() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 1);
    return dis(gen);
}

// TODO --> implement the euclidean norm helper function
double ASBRContext::euclidean_norm(const ASBRContext::vertex &q1, const ASBRContext::vertex &q2) {
    double cumm{0.0};
    for (size_t i{0}; i < q1.size(); ++i){
        double diff = q2[i] - q1[i];
        cumm += diff * diff;
    }
    return std::sqrt(cumm);
}

ASBRContext::path ASBRContext::est(const ASBRContext::vertex& q_init, const ASBRContext::vertex& q_goal) {

    ASBRContext::path P;
    std::cout << "Starting EST with q_goal: ";
    for (const auto& coord : q_goal) std::cout << coord << " ";
    std::cout << std::endl;

    std::vector<ASBRContext::vertex> V_root{q_init}, V_goal{q_goal};
    std::vector<ASBRContext::index> parent{0}, parent_goal{0};
    std::vector<ASBRContext::weight> w{1.0}, w_goal{1.0};

    const int MAX_ITERATIONS = 10000;
    double nearThresholdVal = 20;
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
                    connectionPointRootIdx = V_root.size() - 2;
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
