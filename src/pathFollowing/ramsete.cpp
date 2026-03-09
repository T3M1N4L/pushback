#include "ramsete.h" 
#include "lemlib/util.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm> 

// ==================== 2D Vector Helper Class ====================
// Simple 2D vector for position representation in path following
RamsetePathFollower::Vector2::Vector2(float x, float y) : x(x), y(y) {}  // X and Y coordinates in meters

// Generate LaTeX formatted output for visualization/debugging
std::string RamsetePathFollower::Vector2::latex() const {
    std::ostringstream oss;
    oss << "\\left(" << std::fixed << this->x << "," << std::fixed << this->y << "\\right)";
    return oss.str();
}

// ==================== Sinc Function (sin(x)/x) ====================
// Taylor series approximation near x=0 for numerical stability
// Used in Ramsete controller to prevent division by zero
double RamsetePathFollower::sinc(double x) {
    if (std::abs(x) < 1e-6) {  // Near zero, use Taylor series: sinc(x) ≈ 1 - x²/6 + x⁴/120
        return 1.0 - (x * x) / 6.0 + (x * x * x * x) / 120.0;
    }
    return std::sin(x) / x;  // Normal case: sin(x)/x
}

// ==================== Angle Error Calculation ====================
// Wrap angle difference to [-π, π] for shortest angular path
double RamsetePathFollower::angleError(double robotAngle, double targetAngle) {
    constexpr double TWO_PI = 2.0 * M_PI;
    double diff = std::fmod(targetAngle - robotAngle, TWO_PI);  // Get raw difference
    if (diff < -M_PI) diff += TWO_PI;        // Wrap around: -270° becomes +90°
    else if (diff >= M_PI) diff -= TWO_PI;   // Wrap around: +270° becomes -90°
    return diff;  // Returns shortest angular distance
}

// ==================== Ramsete Controller Constructor ====================
// Ramsete is a non-linear feedback controller for trajectory following
// Parameters:
//   b: Damping parameter (larger = more damping, typical: 2.0)
//   zeta: Convergence rate parameter (larger = faster convergence, typical: 0.7)
//   controller: Velocity controller for motor feedforward/feedback
RamsetePathFollower::RamsetePathFollower(const VelocityControllerConfig& config, float b_, float zeta_)
    : controller(
          config.kV, config.KA_straight, config.KA_turn,    // Feedforward gains (kV, kA)
          config.KS_straight, config.KS_turn,                // Static friction compensation (kS)
          config.KP_straight, config.KI_straight,            // Feedback gains (kP, kI)
          99999.0, TRACK_WIDTH * INCH_TO_METER               // Max accel (irrelevant), track width
      ), b(b_), zeta(zeta_) {}  // Store Ramsete tuning parameters

// ==================== Start Path Following ====================
// Spawns a background task to follow a trajectory from CSV file
void RamsetePathFollower::followPath(const std::string& path_name, const ramseteConfig& r_config) {
    if (is_running) {      // If already running a path
        cancel();          // Request cancellation
        waitUntilDone();   // Wait for task to finish
    }

    // Initialize state for new path
    is_running = true;
    cancel_request = false;
    distance_traveled = 0.0f;

    // Package parameters for background task
    TaskParams* params = new TaskParams{this, path_name, r_config};

    // Spawn background task to execute path following
    task = new pros::Task(task_trampoline, params, "RamseteTask");
    
    pros::delay(10);  // Let task start
}

// ==================== Task Entry Point ====================
// Trampoline function to call instance method from static context
void RamsetePathFollower::task_trampoline(void* params) {
    TaskParams* p = static_cast<TaskParams*>(params);
    p->instance->followPathImpl(p->path_name, p->config);  // Call actual implementation
    delete p;  // Clean up parameter struct
}

// ==================== Wait Utilities ====================
// Block until path following completes
void RamsetePathFollower::waitUntilDone() {
    while (is_running) {
        pros::delay(10);
    }
}

// Block until robot has traveled specified distance along path
void RamsetePathFollower::waitUntil(float dist_inches) {
    while (is_running && distance_traveled < dist_inches) {
        pros::delay(10);
    }
}

// Request cancellation of currently running path
void RamsetePathFollower::cancel() {
    cancel_request = true;
}

// Check if path follower is currently running
bool RamsetePathFollower::isRunning() {
    return is_running;
}

// ============================= Main Path Following Implementation ============================= //
// Ramsete control loop: track precomputed trajectory using non-linear feedback
void RamsetePathFollower::followPathImpl(const std::string& path_name, const ramseteConfig& r_config) {
    std::vector<State> trajectory;
    
    // ==================== Load Trajectory ====================
    // Try to use precomputed path if available
    if(r_config.path_index >= 0 && (size_t)(r_config.path_index) < precomputed_paths.size()) {
        if (!precomputed_paths.at(r_config.path_index).empty()) {
            trajectory = precomputed_paths.at(r_config.path_index);  // Use cached trajectory
        }
    }
    // Otherwise load from CSV file
    if (trajectory.empty()) {
        trajectory = prepare_trajectory(path_name);  // Parse CSV and generate states
    }
    if (trajectory.empty()) {  // Failed to load
        is_running = false;
        return;
    }

    // ==================== Initial Positioning ====================
    if(r_config.test) {  // Test mode: teleport robot to start of path
        chassis.setPose(trajectory[0].x / INCH_TO_METER, trajectory[0].y / INCH_TO_METER, M_PI_2 - trajectory[0].heading, true);
    } else if(r_config.turnFirst) {  // Turn to face initial heading before starting
        double targetH = lemlib::radToDeg(M_PI_2 - trajectory[0].heading);  // Convert to LemLib angle
        chassis.turnToHeading(r_config.backwards ? targetH + 180 : targetH, 1000);  // 180° offset if driving backwards
    }

    std::vector<std::string> logs;  // For debugging output
    int trajectory_size = trajectory.size();
    
    // ==================== Ramsete Tuning Constants ====================
    const float min_k_sq_vel = 0.5f;           // Minimum velocity squared for gain calculation (prevents division by zero)
    const int path_dt_ms = 10;                 // Path timestep (10ms = 100Hz)
    
    const double success_tolerance_inches = 0.5;  // End position tolerance (within 0.5")
    const int max_settle_time_ms = 1500;          // Max time to wait at end (1.5 seconds)
    
    // ==================== State Tracking ====================
    lemlib::Pose start_pose = chassis.getPose();  // Starting position for distance calculation
    uint32_t global_start_time = pros::millis();  // Timestamp for trajectory indexing
    
    bool is_settling = false;      // True when past end of trajectory
    uint32_t settle_start_time = 0;  // Time when settling began

    // ============================= Main Control Loop ============================= //
    while (!cancel_request) {
        uint32_t now = pros::millis();
        
        // ==================== Trajectory Interpolation ====================
        // Calculate current target state from trajectory based on elapsed time
        float t_elapsed_sec = (now - global_start_time) / 1000.0f;  // Seconds since path start
        float exact_index = t_elapsed_sec / (path_dt_ms / 1000.0f);  // Fractional index into trajectory
        int idx = static_cast<int>(exact_index);  // Integer part

        State target_state;  // Target position/velocity/heading for this iteration
        float current_b = r_config.b;  // Ramsete damping parameter (may be increased at end) 

        // ==================== Normal Trajectory Following ====================
        if (idx < trajectory_size - 1) {  // Still following trajectory
            // Linear interpolation between waypoints for smooth motion
            float alpha = exact_index - idx;  // Fractional part (0.0 to 1.0)
            const State& s0 = trajectory[idx];      // Current waypoint
            const State& s1 = trajectory[idx+1];    // Next waypoint

            // Interpolate position (meters)
            target_state.x = s0.x + alpha * (s1.x - s0.x);
            target_state.y = s0.y + alpha * (s1.y - s0.y);
            
            // Interpolate velocity (m/s)
            target_state.linear_vel = s0.linear_vel + alpha * (s1.linear_vel - s0.linear_vel);
            target_state.angular_vel = s0.angular_vel + alpha * (s1.angular_vel - s0.angular_vel);
            
            // Interpolate heading with proper angle wrapping
            double dh = s1.heading - s0.heading;  // Heading change
            while (dh > M_PI) dh -= 2 * M_PI;     // Wrap to [-π, π]
            while (dh < -M_PI) dh += 2 * M_PI;
            target_state.heading = s0.heading + alpha * dh;  // Interpolated heading
        } 
        // ==================== End of Trajectory (Settling) ====================
        else {  // Past end of trajectory
            if (!is_settling) {  // First time reaching end
                is_settling = true;
                settle_start_time = now;  // Start settle timer
            }

            // Check if robot is close enough to end position
            lemlib::Pose p = chassis.getPose();
            double p_x_m = p.x * INCH_TO_METER;  // Convert to meters
            double p_y_m = p.y * INCH_TO_METER;

            // Distance from final waypoint
            double dist_to_end_m = std::hypot(
                trajectory.back().x - p_x_m,
                trajectory.back().y - p_y_m
            );
            
            double dist_to_end_in = dist_to_end_m / INCH_TO_METER;  // Convert to inches

            // Success condition: within tolerance
            if (dist_to_end_in < success_tolerance_inches) {
                break;  // Exit control loop
            }
            // Timeout condition: taking too long
            if (now - settle_start_time > max_settle_time_ms) {
                break;  // Give up and exit
            }

            // Target is final waypoint with zero velocity
            target_state = trajectory.back();
            target_state.linear_vel = 0;   // Stop at end
            target_state.angular_vel = 0;

            current_b = r_config.b * 4.0;  // Increase damping at end for more aggressive correction
        }

        // ==================== Get Robot Position ====================
        lemlib::Pose current_pose = chassis.getPose(true);  // Get real-time odometry
        distance_traveled = start_pose.distance(current_pose);  // Total distance traveled (for waitUntil)

        // Convert from LemLib coordinates (inches, theta from +X axis) to standard coordinates (meters, theta from +Y axis)
        current_pose.x *= INCH_TO_METER;  // Inches to meters
        current_pose.y *= INCH_TO_METER;  
        current_pose.theta = M_PI_2 - current_pose.theta;  // Rotate coordinate system: 0° = +Y instead of +X

        // ==================== Calculate Position Error ====================
        double targetHeadingAdjusted = target_state.heading + (r_config.backwards ? M_PI : 0);  // Flip heading if driving backwards
        double errorTheta = angleError(current_pose.theta, targetHeadingAdjusted);  // Shortest angular error

        // Global error vector (world frame)
        Eigen::Vector3d global_error;
        global_error << target_state.x - current_pose.x,  // X error (meters)
                        target_state.y - current_pose.y,  // Y error (meters)
                        errorTheta;                        // Heading error (radians)

        // ==================== Transform Error to Robot Frame ====================
        // Rotate global error into robot's local coordinate system
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix <<  std::cos(current_pose.theta), std::sin(current_pose.theta), 0,   // X axis in robot frame
                           -std::sin(current_pose.theta), std::cos(current_pose.theta), 0,   // Y axis in robot frame
                            0, 0, 1;                                                           // Theta unchanged

        Eigen::Vector3d local_error = rotation_matrix * global_error;  // Transform to robot frame
        float e_x = local_error(0);  // Forward error (robot's perspective)
        float e_y = local_error(1);  // Lateral error (robot's perspective)
        float e_t = local_error(2);  // Heading error (unchanged)

        // ==================== Ramsete Control Law ====================
        // Non-linear feedback controller for trajectory tracking
        float vd = target_state.linear_vel * (r_config.backwards ? -1.0 : 1.0);  // Target forward velocity (flip if backwards)
        float wd = target_state.angular_vel;  // Target angular velocity

        // Adaptive gain: k = 2*ζ*√(ω²+b*v²)
        // Larger k when moving fast or turning sharply = stronger correction
        float k = 2.0 * r_config.zeta * std::sqrt(wd * wd + current_b * std::max((float)(vd * vd), min_k_sq_vel));

        // Ramsete control law:
        //   v = vd*cos(e_θ) + k*e_x
        //   ω = ωd + k*e_θ + b*vd*sinc(e_θ)*e_y
        float v_desired_ramsete = vd * std::cos(e_t) + k * e_x;  // Desired forward velocity with correction
        float w_desired_ramsete = wd + k * e_t + (current_b * vd * sinc(e_t) * e_y);  // Desired angular velocity with correction

        // ==================== Velocity Control ====================
        // Convert desired v/ω to wheel velocities using feedforward + feedback controller
        float left_actual_mps = leftMotors.get_actual_velocity() * rpm_to_mps_factor;   // Left wheel speed (m/s)
        float right_actual_mps = rightMotors.get_actual_velocity() * rpm_to_mps_factor; // Right wheel speed (m/s)

        // Velocity controller converts (v, ω) to left/right voltages
        // Uses feedforward (kV, kA, kS) + feedback (kP, kI) for accurate tracking
        DrivetrainVoltages output_voltages = controller.update(
            v_desired_ramsete,   // Desired forward velocity (m/s)
            w_desired_ramsete,   // Desired angular velocity (rad/s)
            left_actual_mps,     // Actual left speed (m/s)
            right_actual_mps     // Actual right speed (m/s)
        );
    
        // Apply voltages to motors (convert V to mV)
        rightMotors.move_voltage(output_voltages.rightVoltage * 1000.0);
        leftMotors.move_voltage(output_voltages.leftVoltage * 1000.0);

        // ==================== Debug Logging ====================
        if(r_config.log) {  // Log position for visualization
            std::ostringstream ss;
            ss << Vector2(current_pose.x, current_pose.y).latex() << ",";  // LaTeX format: (x,y),
            logs.push_back(ss.str());
        }

        pros::delay(10);  // 100Hz control loop
    }

    // ==================== Path Complete - Stop Motors ====================
    rightMotors.brake();  // Coast to stop
    leftMotors.brake();
    
    // ==================== End Correction (Optional) ====================
    // Use LemLib's moveToPose for final precise positioning
    if(!r_config.test && r_config.end_correction && !cancel_request) {
        chassis.moveToPose(
            trajectory.back().x / INCH_TO_METER,          // Final X (inches)
            trajectory.back().y / INCH_TO_METER,          // Final Y (inches)
            lemlib::radToDeg(M_PI_2 - trajectory.back().heading),  // Final heading (degrees)
            1000,                                          // Timeout (ms)
            {.lead = r_config.mpose_lead}                 // Lead distance for pure pursuit
        );
        chassis.waitUntilDone();  // Block until moveToPose finishes
    }
    
    if(r_config.log) {
        for (const auto& line : logs) {
            std::cout << line;
            pros::delay(50);
        }
    }

    is_running = false;
}

void RamsetePathFollower::precompute_paths(const std::vector<std::string>& path_names) {
    auto* stored = new std::vector<std::string>(path_names);
    pros::Task t(precompute_paths_task, stored);
}

void RamsetePathFollower::precompute_paths_task(void* param) {
    auto* path_names = static_cast<std::vector<std::string>*>(param);

    precomputed_paths.clear();
    precomputed_paths.reserve(path_names->size());

    for (const auto& name : *path_names) {
        precomputed_paths.push_back(prepare_trajectory(name));
        pros::delay(10); 
    }
    delete path_names;
}

std::vector<std::pair<double,double>> RamsetePathFollower::parse_pairs(const std::string& line) {
    std::vector<std::pair<double,double>> result;
    std::string temp;
    bool inside_parens = false;
    for (char c : line) {
        if (c == '(') {
            temp.clear();
            inside_parens = true;
        } else if (c == ')') { 
            std::replace(temp.begin(), temp.end(), ',', ' ');
            std::istringstream ss(temp);
            double first, second;
            ss >> first >> second;
            result.emplace_back(first, second);
            inside_parens = false;
        } else if (inside_parens) {
            temp += c;
        }
    }
    return result;
}

std::vector<State> RamsetePathFollower::prepare_trajectory(const std::string& data) {
    std::istringstream ss(data);
    std::vector<std::pair<double,double>> X, L, A;
    std::string line;
    X.reserve(1000); L.reserve(1000); A.reserve(1000);
    
    while (std::getline(ss, line)) {
        if (line.find("X =") != std::string::npos) X = parse_pairs(line.substr(line.find('[')));
        else if (line.find("L =") != std::string::npos) L = parse_pairs(line.substr(line.find('[')));
        else if (line.find("A =") != std::string::npos) A = parse_pairs(line.substr(line.find('[')));
    }

    size_t n = X.size();
    if (n == 0) return {};

    std::vector<State> states(n);

    for (size_t i = 0; i < n; i++) {
        states[i].x = X[i].first;
        states[i].y = X[i].second;
        states[i].linear_vel = L[i].second;
        states[i].angular_vel = A[i].second;
    }

    if (n > 1) {
        states[0].heading = atan2(states[1].y - states[0].y, states[1].x - states[0].x);
        
        for (size_t i = 1; i < n - 1; i++) {
            double dy = states[i + 1].y - states[i - 1].y;
            double dx = states[i + 1].x - states[i - 1].x;
            states[i].heading = atan2(dy, dx);
        }
        
        states[n - 1].heading = atan2(states[n - 1].y - states[n - 2].y, states[n - 1].x - states[n - 2].x);
    }

    for(size_t i = 1; i < n; ++i) {
        double diff = states[i].heading - states[i-1].heading;
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        states[i].heading = states[i-1].heading + diff;
    }

    return states;
}
