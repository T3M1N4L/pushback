#include "globals.h"
#include <cmath>
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/util.hpp"
#include "MCL.h"

using namespace std;

// ------------------------ Monte Carlo Localization (MCL) ------------------------
// Particle filter for robust robot position estimation using distance sensors
// More reliable than odometry alone in environments with walls/obstacles

namespace MCL {
  // ==================== Global Position Estimate ====================
  double X = 0, Y = 0, theta = 0;  // Best estimate of robot position (inches, inches, degrees)

  // ==================== Random Number Generator ====================
  mt19937 Random(random_device {} ());  // Mersenne Twister for particle noise generation

  // ==================== MCL State Variables ====================
  double Velo = 0;           // Current robot velocity (inches per second)
  double Deviation;          // Sensor measurement error for weight calculation
  double weights_sum;        // Sum of all particle weights for normalization
  double start_theta;        // Initial heading at MCL start

  // ==================== Particle Cloud ====================
  vector<Particle> Particles(num_particles);  // Collection of position hypotheses 

  // ==================== Distance Sensors ====================
  // Sensors mounted on robot for obstacle detection and localization
  // Format: MCLDistanceSensor(sensor, position_on_robot, direction)
  vector<MCLDistanceSensor> Sensors = {
   MCLDistanceSensor(frontDistance, Point(-3, -0.75), FRONT),     // Front sensor at -3" forward, -0.75" lateral
   MCLDistanceSensor(rightDistance, Point(6.3, -0.5), RIGHT),     // Right sensor at 6.3" right, -0.5" lateral
   MCLDistanceSensor(leftDistance, Point(-6.4, -0.5), LEFT),      // Left sensor at -6.4" left, -0.5" lateral
   MCLDistanceSensor(backDistance, Point(-3, -10.5), BACK),       // Back sensor at -3" back, -10.5" lateral
 };

  // ==================== Field Geometry ====================
  Field field_;                                // Field boundary and wall definitions for ray-casting
  vector<MCLDistanceSensor> activeSensors;     // Sensors with valid readings this cycle
  
  // ==================== Thread Safety ====================
  pros::Mutex particle_mutex;                  // Protects particle data from concurrent access

  // ==================== Function Prototypes ====================
  double getAvgVelocity(void) noexcept;        // Calculate average drive velocity from motor RPM
  void StartMCL(double x_, double y_, double theta_);  // Initialize particle cloud at starting position
  void MonteCarlo(void);                       // Main MCL loop - prediction, update, resample

  // ==================== Timing Constants ====================
  const double LOOP_DELAY_MS = 20.0;           // MCL update rate (50 Hz)
  const double LOOP_DT_SEC = LOOP_DELAY_MS / 1000.0;  // Time step in seconds for velocity integration 

  // ============================= MCL Initialization ============================= //
  // Scatter particles around starting position with Gaussian noise
  void StartMCL(double x_, double y_, double theta_) {
    particle_mutex.take();  // Lock particle data structure
    
    // Reset particle cloud
    Particles.clear();
    Particles.resize(num_particles); 

    // Create Gaussian distributions for initial particle spread
    uniform_real_distribution<double> start_dist_x(-start_std_pos[0], start_std_pos[0]);       // X position noise
    uniform_real_distribution<double> start_dist_y(-start_std_pos[1], start_std_pos[1]);       // Y position noise
    uniform_real_distribution<double> start_dist_theta(-start_std_pos[2], start_std_pos[2]);   // Heading noise

    // Spawn particles around starting position
    for (int i = 0; i < num_particles; ++i) {
      Particles[i].x = x_ + start_dist_x(Random);                         // Add noise to X
      Particles[i].y = y_ + start_dist_y(Random);                         // Add noise to Y
      Particles[i].theta = lemlib::radToDeg(theta_) + start_dist_theta(Random);  // Add noise to heading
      Particles[i].weight = 1.0;                                          // Equal weight initially
    }
    
    particle_mutex.give();  // Unlock particle data
  }

  // ============================= Resampling Buffers ============================= //
  vector<Particle> Resampled(num_particles);   // Buffer for resampled particles (low variance resampling)
  vector<double> CDF(num_particles);           // Cumulative distribution function for resampling

  // ============================= Main MCL Loop ============================= //
  // Runs continuously: predict particle motion, update weights from sensors, resample
  void MonteCarlo(void) {
    while (true) {
      uint32_t start_time = pros::millis();  // Track loop timing
      
      // ==================== PREDICTION STEP ====================
      // Move all particles based on robot velocity (motion model)
      Velo = getAvgVelocity();                                    // Get current robot speed (ips)
      double distance_step = Velo * LOOP_DT_SEC;                  // Distance traveled this timestep
      normal_distribution<double> dist_pos(0, std::abs(distance_step * 0.25));  // Add motion noise (25% of distance)

      // Get current robot heading from odometry
      lemlib::Pose odomPose = chassis.getPose(true);
      const double theta_ = odomPose.theta;                      // Robot heading (degrees)
      const double rotated_theta = M_PI_2 - theta_;              // Convert to standard math angle
      const float cos_theta = cosf(rotated_theta);               // Precompute trig functions
      const float sin_theta = sinf(rotated_theta);

      particle_mutex.take();  // Lock for particle updates

      // Move each particle forward in robot's heading direction
      for (auto& p : Particles) {
        p.theta = theta_;                                         // All particles share robot's heading
        p.step = Point(cos_theta, sin_theta);                    // Unit vector in heading direction
        p.x += (distance_step * cos_theta) + dist_pos(Random);   // Update X with noise
        p.y += (distance_step * sin_theta) + dist_pos(Random);   // Update Y with noise
        p.x = std::clamp(p.x, -field_.HalfSize, field_.HalfSize);  // Keep particles on field
        p.y = std::clamp(p.y, -field_.HalfSize, field_.HalfSize);
      }
      
      // ==================== MEASUREMENT STEP ====================
      // Read all distance sensors and filter out invalid readings
      for (auto& sensor : Sensors) {
        sensor.Measure();  // Query sensor for distance reading
      }

      // Build list of sensors with valid measurements
      activeSensors.clear();
      for (auto& sensor : Sensors) {
        if (sensor.measurement > -1) {  // -1 indicates no object detected or sensor error
          activeSensors.push_back(sensor);
        }
      }

      // ==================== UPDATE STEP ====================
      // Assign weights to particles based on how well sensor predictions match reality
      weights_sum = 0;
      if (!activeSensors.empty()) {
        for (auto& P : Particles) {
          double wt = 1.0;  // Start with perfect weight
          for (auto& sensor : activeSensors) {
            // Ray-cast from particle position to predict what sensor should see
            const double predicted = field_.get_sensor_distance(P, sensor);
            if (predicted < 0) {  // Particle is off-field or ray-cast failed
              wt = 0;  // Zero weight = impossible position
              break;
            }
            // Calculate Gaussian likelihood of measurement given prediction
            Deviation = (predicted - sensor.measurement);  // Prediction error
            wt *= exp((Deviation * Deviation) * inv_varience) * inv_base;  // Gaussian weight
          }
          weights_sum += wt;  // Accumulate for normalization
          P.weight = wt;      // Store weight in particle
        }
      }

      // ==================== WEIGHT NORMALIZATION ====================
      // Ensure weights sum to 1.0 for valid probability distribution
      if (weights_sum < MIN_WEIGHT) {  // All particles have terrible fits (kidnapped robot problem)
        const double Weight = inv_num_particles;  // Reset to uniform distribution
        for (auto &p : Particles) p.weight = Weight;
      } else {  // Normal case: normalize weights to sum to 1.0
        const double inv = 1.0 / weights_sum;
        for (auto &p : Particles) p.weight *= inv;
      }

      // ==================== RESAMPLING STEP ====================
      // Low-variance resampling: duplicate high-weight particles, discard low-weight ones
      
      // Build cumulative distribution function for resampling
      CDF[0] = Particles[0].weight;
      for (int i = 1; i < num_particles; ++i) {
        CDF[i] = CDF[i - 1] + Particles[i].weight;  // Running sum of weights
      }

     // Low-variance systematic resampling (better than random resampling)
      uniform_real_distribution<double> dist(0, inv_num_particles);  // Random offset for first sample
      if(Resampled.size() != num_particles) Resampled.resize(num_particles);

      for (int i = 0; i < num_particles; ++i) {
        const double sample_point = i * (inv_num_particles) + dist(Random);  // Evenly spaced samples
        auto it = lower_bound(CDF.begin(), CDF.end(), sample_point);          // Binary search in CDF
        int index = std::distance(CDF.begin(), it);                           // Index of particle to duplicate
        if(index >= num_particles) index = num_particles - 1;                 // Clamp to valid range
        Resampled[i] = Particles[index];                                      // Copy particle to new generation
      }
      Particles = Resampled;  // Replace old particle cloud with resampled one

      // ==================== POSITION ESTIMATE ====================
      // Compute weighted average of particle positions as final estimate
      double new_x = 0.0, new_y = 0.0, total_weight = 0.0;
      for (const auto& p : Particles) {
        new_x += p.x * p.weight;       // Weighted sum of X coordinates
        new_y += p.y * p.weight;       // Weighted sum of Y coordinates
        total_weight += p.weight;      // Total weight (should be ~1.0 after normalization)
      }

      // Divide by total weight to get weighted average
      if (total_weight > 0) {
        new_x /= total_weight;
        new_y /= total_weight;
      }
      
      // Update global position estimate (thread-safe)
      X = new_x;
      Y = new_y;

      particle_mutex.give();  // Unlock particle data

      double dist_error = std::hypot(odomPose.x - X, odomPose.y - Y);

      cout << " time: (" << pros::millis() - start_time << ") " << std::endl;
      
      pros::Task::delay_until(&start_time, LOOP_DELAY_MS);
    }
  }

  // ============================= Velocity Calculation ============================= //
  // Convert motor RPM to linear robot velocity in inches per second
  const float wheel_circumference = (float)lemlib::Omniwheel::NEW_325 * M_PI;  // 3.25" wheel circumference
  const float gear_ratio = 4.0f / 3.0f;                                        // 4:3 gearing (84 to 60 tooth)
  const float rpm_to_ips_factor = (wheel_circumference / gear_ratio) / 60.0f;  // Conversion factor: RPM -> inches/sec

  // Get average forward velocity from left and right motor speeds
  double getAvgVelocity(void) noexcept {
    const double V = (leftMotors.get_actual_velocity() * rpm_to_ips_factor +   // Left side velocity (ips)
                      rightMotors.get_actual_velocity() * rpm_to_ips_factor) / 2.0; // Average with right side
    return V;
  }
}
