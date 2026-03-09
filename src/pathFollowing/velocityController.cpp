#include "velocityController.h"
#include <cmath>

// ============================= Velocity Controller ============================= //
// Feedforward + Feedback controller for differential drive robots
// Converts desired linear/angular velocities (v, ω) to left/right motor voltages
// Uses system identification parameters (kV, kA, kS) + PID feedback (kP, kI)

// ==================== Sign Function ====================
// Returns -1, 0, or 1 based on sign of x (for static friction direction)
int VoltageController::sign(double x) {
        return (x > 0) - (x < 0);  // Clever bit math: positive=1-0=1, negative=0-1=-1, zero=0-0=0
}

// ==================== Constructor ====================
// Initialize controller with system identification gains and robot parameters
// Parameters:
//   kv: Velocity feedforward (V per m/s) - measured from steady-state velocity tests
//   kaStraight: Linear acceleration feedforward (V per m/s²) - from Newton's 2nd law
//   kaTurn: Angular acceleration feedforward (V per rad/s²) - from moment of inertia
//   ksStraight: Static friction compensation (V) - overcome stiction when starting
//   ksTurn: Rotational static friction (V) - overcome turning resistance
//   kp: Proportional feedback gain - corrects velocity errors
//   ki: Integral feedback gain - eliminates steady-state errors
//   integralThreshold: Only accumulate integral when error < threshold (prevent windup)
//   trackWidth: Distance between left/right wheels (meters) for differential drive
VoltageController::VoltageController(double kv, double kaStraight, double kaTurn, double ksStraight, double ksTurn, double kp,
                      double ki, double integralThreshold, double trackWidth)
        : kV(kv), kaStraight(kaStraight), kaTurn(kaTurn), ksStraight(ksStraight), ksTurn(ksTurn), kP(kp), kI(ki),
          integralThreshold(integralThreshold), trackWidth(trackWidth) {}

// ============================= Main Update Loop ============================= //
// Called every 10ms (100Hz) to compute motor voltages from desired velocities
// Returns: DrivetrainVoltages struct with left/right voltages (-12V to +12V)
DrivetrainVoltages VoltageController::update(double targetLinearVelocity, double targetAngularVelocity, double measuredLeftVelocity,
                            double measuredRightVelocity) {

    // ==================== Calculate Accelerations ====================
    // Numerical differentiation: acceleration = (v_new - v_old) / dt
    // dt = 0.01 seconds (100Hz control loop)
    double deltaW = (targetAngularVelocity - prevAngularVelocity) / 0.01;  // Angular acceleration (rad/s²)
    double deltaV = (targetLinearVelocity - prevLinearVelocity) / 0.01;    // Linear acceleration (m/s²)

    // Store for next iteration
    prevAngularVelocity = targetAngularVelocity;
    prevLinearVelocity = targetLinearVelocity;

    // ==================== Differential Drive Kinematics ====================
    // Convert (v, ω) to individual wheel velocities
    // v_left  = v - ω * (trackWidth / 2)  [subtract because left wheel slower when turning right]
    // v_right = v + ω * (trackWidth / 2)  [add because right wheel faster when turning right]
    double leftVelocity = targetLinearVelocity - targetAngularVelocity * (trackWidth / 2.0);   // Target left wheel velocity (m/s)
    double rightVelocity = targetLinearVelocity + targetAngularVelocity * (trackWidth / 2.0);  // Target right wheel velocity (m/s)

    // ==================== Calculate Velocity Errors ====================
    // Error = target - actual (positive error means we're moving too slow)
    double leftError = leftVelocity - measuredLeftVelocity;   // Left velocity error (m/s)
    double rightError = rightVelocity - measuredRightVelocity; // Right velocity error (m/s)

    // ==================== Integral Term (Anti-Windup) ====================
    // Accumulate error over time to eliminate steady-state offset
    // Reset integral when error crosses zero (prevents windup during direction changes)
    if ((leftError < 0) != (prevLeftError < 0)) {  // Error sign changed (crossed zero)
        leftIntegral = 0;  // Reset integral to prevent accumulated windup
    }
    // Only accumulate integral when error is small (within threshold)
    // This prevents integral windup when error is large (robot far from target)
    if (std::abs(leftError) < integralThreshold) {
        leftIntegral += leftError * 0.01;  // Accumulate: integral += error * dt
    }
    
    // Same for right side
    if ((rightError < 0) != (prevRightError < 0)) {  // Error sign changed
        rightIntegral = 0;  // Reset integral
    }
    if (std::abs(rightError) < integralThreshold) {  // Within threshold
        rightIntegral += rightError * 0.01;  // Accumulate
    }

    // ==================== Feedforward Terms ====================
    // Model-based voltage prediction (no feedback, just physics)
    
    // Acceleration feedforward: V = ka * a
    // Split into linear (F=ma) and angular (τ=Iα) components
    // Left wheel: subtract angular because turning right reduces left speed
    // Right wheel: add angular because turning right increases right speed
    double kaLeft = (kaStraight * deltaV) - (kaTurn * deltaW);   // Left acceleration voltage
    double kaRight = (kaStraight * deltaV) + (kaTurn * deltaW);  // Right acceleration voltage

    // Static friction compensation: V = ks * sign(velocity)
    // Overcomes stiction (static friction) when starting from rest
    // sign() gives direction: +1 forward, -1 backward, 0 stopped
    double ksLeft = (ksStraight * sign(leftVelocity)) - (ksTurn * sign(targetAngularVelocity));   // Left friction voltage
    double ksRight = (ksStraight * sign(rightVelocity)) + (ksTurn * sign(targetAngularVelocity)); // Right friction voltage

    // ==================== Compute Total Voltage ====================
    // Sum of feedforward (model-based) + feedback (error correction)
    // Voltage = kV*v + ka*a + ks*sign(v) + kP*error + kI*integral
    //           ^^^^^   ^^^^   ^^^^^^^^^   ^^^^^^^^^^^^^^^^^^^
    //        velocity  accel   friction    feedback correction
    
    // Calculate unclamped voltages (commented code used std::clamp to ±12V)
    double leftVoltage =
        (kV * leftVelocity) +       // Velocity feedforward: EMF (back-EMF)
        (kaLeft) +                  // Acceleration feedforward
        (ksLeft) +                  // Static friction compensation
        (kP * leftError) +          // Proportional feedback: immediate error correction
        (kI * leftIntegral);        // Integral feedback: eliminate steady-state error
        
    double rightVoltage =
        (kV * rightVelocity) +      // Velocity feedforward
        (kaRight) +                 // Acceleration feedforward
        (ksRight) +                 // Static friction compensation
        (kP * rightError) +         // Proportional feedback
        (kI * rightIntegral);       // Integral feedback
    
    // ==================== Voltage Scaling (Preserve Ratio) ====================
    // If either voltage exceeds ±12V, scale both down proportionally
    // This preserves the left/right ratio (maintains turning correctness)
    // Convert to millivolts: 12V = 12000mV
    double ratio = std::max(fabs(leftVoltage), fabs(rightVoltage)) / 12000.0;  // Calculate max voltage ratio
    if (ratio > 1) {  // If over voltage limit
        leftVoltage /= ratio;   // Scale down left to stay within ±12V
        rightVoltage /= ratio;  // Scale down right (same ratio maintains turn accuracy)
    }

    return {leftVoltage, rightVoltage};  // Return voltages in millivolts
}
