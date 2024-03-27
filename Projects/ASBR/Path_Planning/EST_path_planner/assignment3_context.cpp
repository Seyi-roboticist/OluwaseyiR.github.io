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
// TODO
ASBRContext::index ASBRContext::select_config_from_tree( const std::vector<ASBRContext::weight>& w ){
  ASBRContext::index i;

  // TODO
  // I think the essence of this function is to try to generate configuration of lesser explored areas
  // - Understanding Weighted Sampling
  // Use the weights to return the index of a random configuration in the tree
  std::random_device random_num; // generates a truly random number - a device/paper for creating tokens
  // Using the Mersenne Twister psuedo-random number
  std::mt19937 gen(random_num()); // generates random number - random token

  // Constructing a Uniform Real Distribution Number
  // Using the accumulate function to put all the tokens in one big brown basket
  std::uniform_real_distribution<> random_point_generator(0.0, std::accumulate(w.begin(), w.end(), 0.0));

  // generating a random number - I am going to draw a token from the big brown basket
  double random_point = random_point_generator(gen);

  // The name of the algorithm is called the roulette wheel sampling in compsci
  double cum_weight{0.0}; // cumulative weight
  for ( i = 0; i < w.size(); ++i) {
      cum_weight += w[i];
      if (random_point <= cum_weight){
          break;
      }
  }
  return i; // returning the random index generated
}

///****************************************SAMPLE NEARBY**********************************************/
//ASBRContext::vertex ASBRContext::sample_nearby( const ASBRContext::vertex& q ){
//    ASBRContext::vertex q_rand(q.size()); // error handling definition
//
//    const double MAX_DEVIATION = M_PI/90;
//    std::random_device random_num;
//    std::mt19937 gen(random_num());
//    std::normal_distribution <> gaussian_generator(0, 1);
//
//    const int MAX_ATTEMPTS{1000};
//    int attempts{0};
//    const double denomFix = 1e-6;
//
//    do {
//        for (size_t i{0}; i < q.size(); ++i)
//            q_rand[i] = gaussian_generator(gen);
//
//        double q_rand_norm = std::sqrt(std::inner_product(q_rand.begin(), q_rand.end(), q_rand.begin(), 0.0));
//
//        ++attempts;
//        if (attempts >= MAX_ATTEMPTS) {
//            return q;
//        }
//
//        if(q_rand_norm < denomFix) {
//            continue;
//        }
//
//        for (size_t i{0}; i < q.size(); ++i) {
//            q_rand[i] = q[i] + MAX_DEVIATION * (q_rand[i] / q_rand_norm);
//        }
//
//    } while (state_collides(q_rand));
//
//    std::cout << "Printing q_rand[1]" << q_rand[1] << " " << q[1] << std::endl;
//
//    return q_rand;
//}

//ASBRContext::vertex ASBRContext::sample_nearby(const ASBRContext::vertex& q) {
//    ASBRContext::vertex q_rand(q.size());
//    const double MAX_DEVIATION = M_PI / 10;
//    std::random_device random_num;
//    std::mt19937 gen(random_num());
//    std::normal_distribution<> gaussian_generator(0, 1);
//
//    const int MAX_ATTEMPTS = 10000;
//    int attempts = 0;
//
//    do {
//        double q_rand_norm = 0;
//        // Generate a random direction
//        for (size_t i = 0; i < q.size(); ++i) {
//            q_rand[i] = gaussian_generator(gen);
//            q_rand_norm += q_rand[i] * q_rand[i];
//        }
//        q_rand_norm = std::sqrt(q_rand_norm);
//
//        // Scale the random vector to have a norm of MAX_DEVIATION
//        for (size_t i = 0; i < q.size(); ++i) {
//            // Normalize if norm is greater than 1 to stay within the MAX_DEVIATION sphere
//            if (q_rand_norm > 1.0)
//                q_rand[i] *= MAX_DEVIATION / q_rand_norm;
//            else
//                q_rand[i] *= MAX_DEVIATION;
//
//            // Apply the random deviation and wrap the joint angle within [-π, π)
//            q_rand[i] = q[i] + q_rand[i];
//            q_rand[i] = wrap_angle(q_rand[i]);
//        }
//        ++attempts;
//        if (attempts >= MAX_ATTEMPTS)
//            throw std::runtime_error("Maximum attempts reached without finding a collision-free nearby configuration.");
//    } while (state_collides(q_rand));
//
//    return q_rand;
//}
//
//double ASBRContext::wrap_angle(double angle) {
//    // Wrap the angle to the range [-π, π)
//    while (angle < -M_PI) angle += 2 * M_PI;
//    while (angle >= M_PI) angle -= 2 * M_PI;
//    return angle;
//}

/****************************************SAMPLE NEARBY**********************************************/
// TODO
ASBRContext::vertex ASBRContext::sample_nearby( const ASBRContext::vertex& q ){
    ASBRContext::vertex q_rand(q.size()); // error handling definition
    // TODO
    // Generate a random configuration near q
    // Let me define a nearby distance to be within 3 degrees.
    const double MAX_DEVIATION = M_PI/60; // The workspace for the UR5 using the trajectory controller is -pi to pi
    std::random_device random_num;
    std::mt19937 gen(random_num());
    std::normal_distribution <> gaussian_generator(0, 1); // normal (gaussian) distribution

    // Attempt counter and limit to check for samples nearby
    const int MAX_ATTEMPTS{60000};
    int attempts{0};

    // Main logic for generating the nearby sample
    do {
        for (size_t i{0}; i < q.size(); ++i)
            q_rand[i] = gaussian_generator(gen); // sample a random value for each dimension
        // Calculating the Euclidean Norm
        // double q_rand_norm = std::sqrt(std::inner_product(q_rand.begin(), q_rand.end(), q_rand.begin(), 0.0));
        for (size_t i{0}; i < q.size(); ++i){
            q_rand[i] = q[i] + MAX_DEVIATION * q_rand[i];
            // q_rand[i] = q[i] + MAX_DEVIATION * (q_rand[i] / q_rand_norm); // centre + (deviation * val(- 1, 1)).
        }
        ++attempts;
        // I need to return something if we exceed the limit
        if (attempts >= MAX_ATTEMPTS)
            return q;
    } while (state_collides(q_rand));

    std::cout << "Printing q_rand[1]" << q_rand[1] << " " << q[1] << std::endl;
    return q_rand;
}


/**********************************LOCAL PATH COLLISION CHECK**************************************/
// TODO
bool ASBRContext::is_local_path_collision_free( const ASBRContext::vertex& q,
						const ASBRContext::vertex& q_rand ){

  // TODO find if the straightline path between q_near and q_rand is collision free
  // First check
  if (state_collides(q) || state_collides(q_rand))
      return false;

  double interpolation_resolution = 0.05; // resolution definition to work somewhat like discreet steps
  for (double t{interpolation_resolution}; t < 1.0; t += interpolation_resolution) {
      ASBRContext::vertex qt = interpolate(q, q_rand, t);
      // Hypothetical to check if discrete is in collision with some weird obstacle
      if (state_collides(qt))
          return false; //
  }
  // Else, qt at t is not in collision
  return true;
}
//
// *****************************SEARCH PATH ---> HEAD RECURSION ALGORITHM******************************/
// TODO
// check search path
ASBRContext::path ASBRContext::search_path( const std::vector<vertex>& V,
                                            const std::vector<index>& parent,
                                            const index& idx_init,
                                            const index& idx_goal ){
    ASBRContext::path P;
    // If things are getting weird, try to pass parent by reference instead of value (I changed to reference)
    // TODO Once q_goal has been added to the tree, find the path (sequence of configurations) between
    // q_init and q_goal (hint: this is easier by using recursion).
    // Base case
    if (idx_init == idx_goal)
        P.push_back(V[idx_init]); // advanced way {V[idx_init]}

        // Think about implement the parent naturally
        // Recursive case
    else{
        // Pretty much saying don't go all the way to the goal yet;
        // First get to the place we came from just before we reached the goal.
        // So the output will be from q_goal to q_init
        // Note that parent is an array of indices, and we are recursively chaining from back to front
        P = search_path(V, parent, idx_init, parent[idx_goal]); // the recursive call
        P.push_back(V[idx_goal]); // append this goal to the path and call again if necessary
    }
    // cout from q_init to q_goal
    // std::reverse(P.begin(), P.end()); // reversing the path -- this is wrong
    return P;
}

/***************************************NEAREST CONFIGURATION************************************************/
ASBRContext::index ASBRContext::find_nearest_configuration( const std::vector<ASBRContext::vertex>& V, const ASBRContext::vertex& q ){
    // Let me start by setting a large minimum distance
    double min_dist = std::numeric_limits<double>::max(); // just like I did in the minimax algorithm
    ASBRContext::index min_idx = std::numeric_limits<ASBRContext::index>::max(); // if there is no element in the node vectors V, return -1

    // Iterating the 2D area 
    for (size_t i = 0; i < V.size(); ++i) {
        double dist = 0.0;
        for (size_t j = 0; j < q.size(); ++j) {
            double diff = V[i][j] - q[j];
            dist += std::pow(diff, 2); // squaring both sides or just simple diff * diff
        }

        // Computing the distance
        dist = std::sqrt(dist);
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }
    return min_idx;
}

// TODO
ASBRContext::path ASBRContext::est( const ASBRContext::vertex& q_init,
				    const ASBRContext::vertex& q_goal ){

  ASBRContext::path P;
  std::cout << q_goal[0] << " "<< q_goal[1] << " "<< q_goal[2] << " "<< q_goal[3] << " "<< q_goal[4] << " "<< q_goal[5] << " "<< q_goal[6]<<std::endl;
  // TODO implement EST algorithm and return the path (an ordered sequence of configurations).
  // recursively update the tree (V, E))
  // do the same for the parent nodes
  // Building the networks ----> Initializations
  std::vector<ASBRContext::vertex> V{q_init};//V_goal{q_goal}; // Initializing the two tree with the corresponding nodes
  std::vector<ASBRContext::index> parent{0};// parent_goal{-1};  // Initializing parent to an empty array for now
  std::vector<ASBRContext::weight> w{1.0}; // Weights ---> keeps track of the weights of the edges
  bool connected = false;  // Setting a flag to track if there is a valid path from q_init to q_goal

  // flip the order for one of the trees
  // Since I have q_init and q_goal as input parameters, I need to initialize their indices as well.
  ASBRContext::index idx_q_init{0};
  ASBRContext::index idx_q_goal{std::numeric_limits<ASBRContext::index>::max()}; // Use max value as a placeholder.
  const int MAX_ITERATIONS{20000};

  // EST Algorithm Implementation
  std::cout << "Starting the legendary EST algorithm implementation..." << std::endl;
  for(int i{0}; i < MAX_ITERATIONS; ++i) {
      ASBRContext::index idx_q_rand = select_config_from_tree(w); // select random config index based on weights
      // w1 and w2 for two configs and sample (q_rand1 and q_randgoal)
      ASBRContext::vertex q_rand = sample_nearby(V[idx_q_rand]); // let's get a sample near that configuration

      // Multiple trees --> consider adding extra parameter for tree

      // Finding the nearest configuration in the tree using the Euclidean Norm already embedded in the function
      ASBRContext::index idx_q_near = find_nearest_configuration(V, q_rand); // checks for nearest random congfig in the tree (graph of nodes)
      std::cout << "Iteration: " << i << std::endl;
      std::cout << "Selected random configuration index: " << idx_q_rand << std::endl;

      // 292 -- 294 check V[idx_
      // Push to private repo and try with bidirectional

      /***
       * If  a nice path from the nearest configuration (q_near) to the new random configuration (q_rand)
       * exists, add the new configuration to the vertex V --> building V as we go.
       ***/
       if(is_local_path_collision_free(V[idx_q_near], q_rand)) {
           V.push_back(q_rand); // adds the q_rand to the vertex V (graph of nodes)
           parent.push_back(idx_q_near); // since we are extending from node q_near to node q_rand (Parent-child relationship)
           w.push_back(1.0 / (1.0 + w[idx_q_near])); // using the formula in the sample-based path planning slide
           std::cout << "Added new configuration to tree. V size now: " << V.size() << std::endl;
           /**
            * If there exist a path from the new configuration to the goal, set the flag and terminate the loop
            **/
            if (is_local_path_collision_free(q_rand, q_goal)) {
                V.push_back(q_goal); // Adds the q_goal to the tree
                parent.push_back(V.size() - 2); // since parent is using indices, we will just do last - 2.
                idx_q_goal = V.size() - 1;
                connected = true;
                std::cout << "Added new configuration to tree. V size now: " << V.size() << std::endl;
                std::cout << "Path to goal found. Exiting loop." << std::endl;
                break;
            }
            else {
                std::cout << "Local path collision detected. No configuration added." << std::endl;
            }
       }

  }

       // Returning the path P using search path function if it is connected
 if (connected){
           std::cout << "Searching for path back to start..." << std::endl;
           P = search_path(V, parent, idx_q_init, idx_q_goal);
           std::cout << "Path found with size: " << P.size() << std::endl;
       }

  if(!connected){
        std::cout << "The path is not connected!" << std::endl;
  }
  return P;
}

// check if the initial and final path can be connected 

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
