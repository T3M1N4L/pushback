#include "ltv.h"
#include "lemlib/util.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

// ============================= LTV (Linear Time-Varying) Path Follower ============================= //
// Advanced path following using LQR (Linear Quadratic Regulator) control
// Computes optimal gains at each timestep by solving DARE (Discrete Algebraic Riccati Equation)
// More sophisticated than Ramsete - uses state-space control theory

// ==================== 2D Vector Helper Class ====================
LTVPathFollower::Vector2::Vector2(float x, float y) : x(x), y(y) {}  // X and Y coordinates in meters

// Generate LaTeX formatted output for visualization/debugging
std::string LTVPathFollower::Vector2::latex() const {
    std::ostringstream oss;
    oss << "\\left(" << std::fixed << this->x << "," << std::fixed << this->y << "\\right)";
    return oss.str();
}

// ==================== Angle Error Calculation ====================
// Wrap angle difference to [-π, π] for shortest angular path
double LTVPathFollower::angleError(double robotAngle, double targetAngle) {
    constexpr double TWO_PI = 2.0 * M_PI;
    double diff = std::fmod(targetAngle - robotAngle, TWO_PI);  // Get raw difference
    if (diff < -M_PI) diff += TWO_PI;        // Wrap around: -270° becomes +90°
    else if (diff >= M_PI) diff -= TWO_PI;   // Wrap around: +270° becomes -90°
    return diff;  // Returns shortest angular distance
}

// ==================== LTV Controller Constructor ====================
// Initialize with velocity controller for motor control
// Note: LTV computes optimal control actions (v, ω), velocity controller converts to voltages
LTVPathFollower::LTVPathFollower(const VelocityControllerConfig& config)
    : controller(
          config.kV,              // Velocity feedforward gain
          config.KA_straight,     // Linear acceleration gain
          config.KA_turn,         // Angular acceleration gain
          config.KS_straight,     // Static friction (linear)
          config.KS_turn,         // Static friction (angular)
          config.KP_straight,     // Proportional gain
          config.KI_straight,     // Integral gain
          99999.0,                // Integral threshold (effectively disabled)
          12.8f * INCH_TO_METER   // Track width in meters (12.8 inches converted)
      ) {}

// ============================= DARE Solver (Discrete Algebraic Riccati Equation) ============================= //
// Solves: Aᵀ X (A - B K) + Q = X  where K = (R + Bᵀ X B)⁻¹ Bᵀ X A
// This gives the optimal state feedback gains for LQR control
// Iteratively converges to solution using fixed-point iteration
Eigen::MatrixXf LTVPathFollower::dareSolver(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R) {
    Eigen::MatrixXf X = Q;  // Initialize solution with state cost matrix Q
    Eigen::MatrixXf X_prev; // Previous iteration for convergence check
    Eigen::MatrixXf K;      // Optimal gain matrix

    // Iterative solver: update X until convergence
    for (int i = 0; i < 1000; ++i) {  // Max 1000 iterations to prevent infinite loop
        X_prev = X;  // Store previous value
        
        // Compute optimal gain: K = (R + Bᵀ X B)⁻¹ Bᵀ X A
        K = (R + B.transpose() * X * B).inverse() * B.transpose() * X * A;
        
        // Riccati equation update: X = Aᵀ X (A - B K) + Q
        X = A.transpose() * X * (A - B * K) + Q;

        // Check convergence: if change in X is < 1e-4, solution has stabilized
        if ((X - X_prev).norm() < 1e-4) {
            break;  // Converged, exit early
        }
    }
    return X;  // Return converged solution matrix
}

// ============================= Continuous to Discrete Conversion (c2d) ============================= //
// Converts continuous-time state-space model to discrete-time using matrix exponential approximation
// Continuous: dx/dt = A x + B u  →  Discrete: x[k+1] = Ad x[k] + Bd u[k]
// Uses 2nd order Taylor series: e^(MΔt) ≈ I + MΔt + (MΔt)²/2
std::pair<Eigen::MatrixXf, Eigen::MatrixXf> LTVPathFollower::discretizeAB(
    const Eigen::MatrixXf& contA, const Eigen::MatrixXf& contB, double dtSeconds) {

    int states = contA.rows();  // Number of states (3 for x, y, θ)
    int inputs = contB.cols();  // Number of inputs (2 for v, ω)

    // ==================== Build Augmented Matrix ====================
    // Construct M = [A | B]  (states+inputs) x (states+inputs)
    //               [0 | 0]
    Eigen::MatrixXf M(states + inputs, states + inputs);
    M.setZero();  // Initialize to zero
    M.topLeftCorner(states, states) = contA;    // Top-left: A matrix
    M.topRightCorner(states, inputs) = contB;   // Top-right: B matrix
    // Bottom rows remain zero

    // ==================== Matrix Exponential Approximation ====================
    // Φ = e^(MΔt) ≈ I + MΔt + (MΔt)²/2  (2nd order Taylor series)
    Eigen::MatrixXf Mdt = M * dtSeconds;  // Scale by timestep
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(M.rows(), M.cols());  // Identity matrix
    Eigen::MatrixXf M2 = Mdt * Mdt;  // (MΔt)²
    
    Eigen::MatrixXf phi = I + Mdt + (M2 / 2.0);  // Taylor approximation

    // ==================== Extract Discrete Matrices ====================
    // Ad = top-left corner of Φ (states x states)
    // Bd = top-right corner of Φ (states x inputs)
    Eigen::MatrixXf discA = phi.topLeftCorner(states, states);   // Discrete A
    Eigen::MatrixXf discB = phi.topRightCorner(states, inputs);  // Discrete B

    return {discA, discB};  // Return discrete-time matrices
}

// ============================= Path Precomputation ============================= //
// Load trajectories in background task to avoid lag during autonomous
void LTVPathFollower::precompute_paths(const std::vector<std::string>& path_names) {
    auto* stored = new std::vector<std::string>(path_names);  // Heap allocate for task
    pros::Task t(precompute_paths_task, stored);  // Spawn background task
}

// Background task: load all trajectories from CSV files
void LTVPathFollower::precompute_paths_task(void* param) {
    auto* path_names = static_cast<std::vector<std::string>*>(param);
    precomputed_paths.clear();  // Clear old paths
    precomputed_paths.reserve(path_names->size());  // Preallocate space
    
    for (const auto& name : *path_names) {
        precomputed_paths.push_back(prepare_trajectory(name));  // Load and parse CSV
        pros::delay(10);  // Yield to prevent blocking
    }
    delete path_names;  // Clean up heap allocation
}

// ============================= CSV Parsing Utility ============================= //
// Parses strings like "(1.2, 3.4) (5.6, 7.8)" into vector of pairs
std::vector<std::pair<double,double>> LTVPathFollower::parse_pairs(const std::string& line) {
    std::vector<std::pair<double,double>> result;
    std::string temp;  // Buffer for content between parentheses
    bool inside_parens = false;  // Track if we're inside ()
    
    for (char c : line) {
        if (c == '(') {  // Start of pair
            temp.clear();
            inside_parens = true;
        } else if (c == ')') {  // End of pair
            // Parse the pair: "1.2, 3.4" → (1.2, 3.4)
            std::replace(temp.begin(), temp.end(), ',', ' ');  // Replace comma with space
            std::istringstream ss(temp);
            double first, second;
            ss >> first >> second;  // Parse two numbers
            result.emplace_back(first, second);  // Add to result
            inside_parens = false;
        } else if (inside_parens) {  // Character inside parentheses
            temp += c;  // Accumulate
        }
    }
    return result;
}

// ============================= Trajectory Preparation ============================= //
// Parses CSV data into trajectory of states (x, y, v, ω, θ)
// CSV format: X = [(x1,y1) (x2,y2) ...], L = [(t1,v1) (t2,v2) ...], A = [(t1,w1) (t2,w2) ...]
std::vector<State> LTVPathFollower::prepare_trajectory(const std::string& data) {
    std::istringstream ss(data);  // String stream for line-by-line parsing
    std::vector<std::pair<double,double>> X, L, A;  // Position, Linear velocity, Angular velocity
    std::string line;
    
    // ==================== Parse CSV Lines ====================
    // Read each line and extract arrays based on prefix
    while (std::getline(ss, line)) {
        if (line.find("X =") != std::string::npos) X = parse_pairs(line.substr(line.find('[')));       // Position data
        else if (line.find("L =") != std::string::npos) L = parse_pairs(line.substr(line.find('[')));  // Linear velocity
        else if (line.find("A =") != std::string::npos) A = parse_pairs(line.substr(line.find('[')));  // Angular velocity
    }

    size_t n = X.size();
    if (n == 0) return {};  // Empty trajectory
    std::vector<State> states(n);

    // ==================== Extract State Data ====================
    // Fill x, y, linear_vel, angular_vel from parsed arrays
    for (size_t i = 0; i < n; i++) {
        states[i].x = X[i].first;              // X coordinate (meters)
        states[i].y = X[i].second;             // Y coordinate (meters)
        states[i].linear_vel = L[i].second;    // Linear velocity (m/s) - second element is velocity
        states[i].angular_vel = A[i].second;   // Angular velocity (rad/s)
    }

    // ==================== Calculate Headings ====================
    // Estimate heading from path geometry (finite differences)
    if (n > 1) states[0].heading = atan2(states[1].y - states[0].y, states[1].x - states[0].x);  // First waypoint
    
    for (size_t i = 1; i < n - 1; i++) {  // Middle waypoints (use centered difference)
        states[i].heading = atan2(states[i + 1].y - states[i - 1].y, states[i + 1].x - states[i - 1].x);
    }
    
    if (n > 1) states[n - 1].heading = atan2(states[n - 1].y - states[n - 2].y, states[n - 1].x - states[n - 2].x);  // Last waypoint

    return states;
}

// ============================= LTV Path Following Main Loop ============================= //
// Follows a trajectory using LQR control with time-varying gains computed at each step
void LTVPathFollower::followPath(const std::string& path_name, const ltvConfig& l_config) {
    std::vector<State> trajectory;
    
    // ==================== Load Trajectory ====================
    // Try to use precomputed path from background loading if available
    if(l_config.path_index >= 0 && (size_t)(l_config.path_index) < precomputed_paths.size()) {
        if (!precomputed_paths.at(l_config.path_index).empty()) {
            trajectory = precomputed_paths.at(l_config.path_index);  // Use cached trajectory
        }
    }
    if (trajectory.empty()) trajectory = prepare_trajectory(path_name);  // Fallback: parse CSV now
    if (trajectory.empty()) return;  // No valid path data, abort

    // ==================== LQR Tuning Matrices ====================
    // Q matrix: Penalties for state deviations (higher = track state more aggressively)
    // q_x: X position error penalty
    // q_y: Y position error penalty
    // q_theta: Heading error penalty
    Eigen::Matrix3f Q_mat; 
    Q_mat << l_config.q_x, 0, 0,              // Penalize X error
             0, l_config.q_y, 0,              // Penalize Y error
             0, 0, l_config.q_theta;          // Penalize heading error

    // R matrix: Penalties for control effort (higher = use less aggressive control)
    // r_vel: Linear velocity control penalty (smoothness vs tracking)
    // r_ang: Angular velocity control penalty
    Eigen::Matrix2f R_mat;
    R_mat << l_config.r_vel, 0,               // Penalize linear velocity commands
             0, l_config.r_ang;               // Penalize angular velocity commands

    if(l_config.test) {
        chassis.setPose(trajectory[0].x / INCH_TO_METER, trajectory[0].y / INCH_TO_METER, M_PI_2 - trajectory[0].heading, true);
    } else if(l_config.turnFirst) {
        double targetH = lemlib::radToDeg(M_PI_2 - trajectory[0].heading);
        chassis.turnToHeading(l_config.backwards ? targetH + 180 : targetH, 1000);
    }

    std::vector<std::string> logs;  // Storage for logging trajectory data
    int trajectory_size = trajectory.size();  // Number of waypoints
    
    double dt = 0.01;  // Timestep: 10ms (100 Hz control loop)

    // ============================= Main Control Loop ============================= //
    // Iterate through each waypoint in trajectory and compute LQR control
    for (int i = 0; i < trajectory_size; ++i) {
        uint32_t start_time_ms = pros::millis();  // Timestamp for precise loop timing
        const auto &target_state = trajectory[i];  // Current waypoint target

        // ==================== Get Current Robot State ====================
        lemlib::Pose current_pose = chassis.getPose(true);  // Read odometry (inches, LemLib convention)
        current_pose.x *= INCH_TO_METER;  // Convert inches → meters
        current_pose.y *= INCH_TO_METER;  // Convert inches → meters
        current_pose.theta = M_PI_2 - current_pose.theta;  // Convert LemLib angle (0 = +Y) → standard (0 = +X)

        // ==================== Compute State Errors ====================
        // Adjust target heading if driving backwards (flip 180°)
        double targetHeadingAdjusted = target_state.heading + (l_config.backwards ? M_PI : 0);
        double errorTheta = angleError(current_pose.theta, targetHeadingAdjusted);  // Wrapped to [-π, π]
        
        // Global frame error vector: [error_x, error_y, error_θ]ᵀ
        Eigen::Vector3d global_error;
        global_error << target_state.x - current_pose.x,  // X position error (meters)
                        target_state.y - current_pose.y,  // Y position error (meters)
                        errorTheta;                        // Heading error (radians)

        // ==================== Transform Error to Robot Frame ====================
        // LQR requires errors in robot's local coordinate frame
        // Rotation matrix: transforms global → local using current heading
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix <<  std::cos(current_pose.theta), std::sin(current_pose.theta), 0,   // x_local = x_global*cos + y_global*sin
                           -std::sin(current_pose.theta), std::cos(current_pose.theta), 0,   // y_local = -x_global*sin + y_global*cos
                            0, 0, 1;                                                          // θ_local = θ_global (no change)

        Eigen::Vector3d error = rotation_matrix * global_error;  // [error_x_robot, error_y_robot, error_θ]ᵀ

        // ==================== Extract Reference Velocities ====================
        // Feedforward commands from trajectory planner
        float v_ref = target_state.linear_vel * (l_config.backwards ? -1.0 : 1.0);  // Linear velocity (flip if backwards)
        float w_ref = target_state.angular_vel;  // Angular velocity (rad/s)

        // ==================== State-Space Linearization ====================
        // Ensure v_calc is never too small to avoid singularities in linearization
        // Minimum velocity threshold: 0.25 m/s (~10 in/s)
        float v_calc = v_ref;
        if (std::abs(v_calc) < 0.25) {
            v_calc = (v_calc >= 0) ? 0.25 : -0.25;  // Clamp to minimum magnitude
        }

        // ==================== Continuous State-Space Model ====================
        // Linear Time-Varying model: dx/dt = A x + B u
        // State: x = [x_error, y_error, θ_error]ᵀ
        // Control: u = [v, ω]ᵀ (linear and angular velocities)
        // Linearized around current velocity v_calc
        Eigen::Matrix3f A;
        A << 0, 0, 0,         // dx/dt = 0*x + 0*y + 0*θ + v (velocity directly affects x)
             0, 0, v_calc,    // dy/dt = 0*x + 0*y + v*θ (cross-track error grows with heading error)
             0, 0, 0;         // dθ/dt = 0*x + 0*y + 0*θ + ω (angular velocity directly affects θ)

        // Input matrix: how control inputs affect state derivatives
        Eigen::Matrix<float, 3, 2> B;
        B << 1, 0,  // dx/dt += 1*v + 0*ω  (linear velocity moves robot forward in local frame)
             0, 0,  // dy/dt += 0*v + 0*ω  (lateral velocity is zero for differential drive)
             0, 1;  // dθ/dt += 0*v + 1*ω  (angular velocity rotates robot)

        // ==================== Discretize State-Space Model ====================
        // Convert continuous-time model to discrete-time for digital control
        // Uses matrix exponential approximation (2nd order Taylor series)
        auto discAB = discretizeAB(A, B, dt);  // Returns {A_d, B_d}

        // ==================== Solve DARE for Optimal Gains ====================
        // Solve Discrete Algebraic Riccati Equation to get cost-to-go matrix X
        Eigen::MatrixXf X = dareSolver(discAB.first, discAB.second, Q_mat, R_mat);
        
        // Compute optimal feedback gain matrix K = (R + Bᵀ X B)⁻¹ Bᵀ X A
        // This minimizes J = Σ (xᵀ Q x + uᵀ R u) over infinite horizon
        Eigen::MatrixXf K = (R_mat + discAB.second.transpose() * X * discAB.second).inverse() 
                           * discAB.second.transpose() * X * discAB.first;

        // ==================== Compute Optimal Control ====================
        // LQR control law: u = -K * error (negative feedback)
        // Returns correction to add to feedforward command
        Eigen::Vector2f u = K * error.cast<float>();  // [Δv, Δω]ᵀ

        // Total command = feedforward + feedback correction
        float v_cmd = v_ref + u(0);  // Commanded linear velocity (m/s)
        float w_cmd = w_ref + u(1);  // Commanded angular velocity (rad/s)

        // ==================== Read Motor Velocities ====================
        // Get actual wheel speeds for velocity controller feedback
        float left_actual_mps = leftMotors.get_actual_velocity() * rpm_to_mps_factor;   // Convert RPM → m/s
        float right_actual_mps = rightMotors.get_actual_velocity() * rpm_to_mps_factor; // Convert RPM → m/s

        // ==================== Velocity Controller ====================
        // Convert (v, ω) commands to left/right motor voltages
        // Uses feedforward (kV, kA, kS) + feedback (kP, kI) control
        DrivetrainVoltages output_voltages = controller.update(
            v_cmd,              // Desired linear velocity (m/s)
            w_cmd,              // Desired angular velocity (rad/s)
            left_actual_mps,    // Current left wheel velocity (m/s)
            right_actual_mps    // Current right wheel velocity (m/s)
        );
    
        // ==================== Apply Motor Commands ====================
        // Send voltages to motors (convert V → mV for PROS API)
        rightMotors.move_voltage(output_voltages.rightVoltage * 1000.0);  // Right side voltage
        leftMotors.move_voltage(output_voltages.leftVoltage * 1000.0);    // Left side voltage

        // ==================== Logging (Optional) ====================
        if(l_config.log) {
            std::ostringstream ss;
            ss << Vector2(current_pose.x, current_pose.y).latex() << ",";  // LaTeX format for plotting
            logs.push_back(ss.str());  // Store for later output
        }

        // ==================== Loop Timing ====================
        // Maintain precise 10ms loop period (100 Hz)
        pros::Task::delay_until(&start_time_ms, 10);  // Sleep until 10ms elapsed since loop start
    }

    // ============================= Post-Trajectory Correction ============================= //
    // Optional: use LemLib's moveToPose for final position correction
    if(!l_config.test && l_config.end_correction) {
        pros::delay(100);  // Brief pause before correction
        
        // Drive to final waypoint using pure pursuit
        chassis.moveToPose(
            trajectory.back().x / INCH_TO_METER,           // Target X (inches)
            trajectory.back().y / INCH_TO_METER,           // Target Y (inches)
            lemlib::radToDeg(M_PI_2 - trajectory.back().heading),  // Target heading (degrees, LemLib convention)
            2000,                                           // 2 second timeout
            {.lead = l_config.mpose_lead}                  // Pure pursuit lead distance
        );
        chassis.waitUntilDone();  // Block until movement complete
    }
    
    // ==================== Stop Motors ====================
    rightMotors.brake();  // Apply brakes to right motors
    leftMotors.brake();   // Apply brakes to left motors

    // ==================== Output Logs ====================
    // Print trajectory data if logging enabled
    if(l_config.log) {
        for (const auto& line : logs) {
            std::cout << line;  // Print LaTeX formatted position data
            pros::delay(50);    // Small delay between lines
        }
    }
}
