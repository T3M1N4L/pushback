# LemLib PID Tuner

An intuitive PID tuning tool for LemLib that allows you to adjust your robot's PID constants on-the-fly using the V5 controller and brain screen. No code recompilation needed!

---

## Quick Start

### 1. Include the Header

```cpp
#include "main.h"
#include "pid_tuner.hpp"
```

### 2. Create the PID Tuner

```cpp
// Your existing chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

// Create PID tuner (pass your chassis and controller)
PIDTuner pidTuner(chassis, controller);
```

### 3. Initialize in `initialize()`

```cpp
void initialize() {
    chassis.calibrate();
    
    // Set initial PID values to match your ControllerSettings
    pidTuner.pid_tuner_set_linear(8.5, 0, 43, 3);    // kP, kI, kD, windupRange
    pidTuner.pid_tuner_set_angular(5.5, 0, 42.4, 3);
    
    // Optional: Configure increment values for adjustments
    pidTuner.pid_tuner_increment_p_set(0.1);
    pidTuner.pid_tuner_increment_i_set(0.001);
    pidTuner.pid_tuner_increment_d_set(0.5);
    pidTuner.pid_tuner_increment_windup_set(0.1);
}
```

### 4. Add to `opcontrol()` Loop

```cpp
void opcontrol() {
    while (true) {
        // Toggle PID tuner with X button
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            pidTuner.pid_tuner_toggle();
        }
        
        // Update tuner every loop (handles button inputs)
        pidTuner.pid_tuner_iterate();
        
        // Your normal driver control code
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);
        
        pros::delay(10);
    }
}
```

---

## Controller Button Reference

| Button | Function |
|--------|----------|
| **X** | Toggle PID tuner on/off |
| **LEFT/RIGHT** | Switch between Linear and Angular controllers |
| **UP/DOWN** | Select which constant to adjust (kP, kI, kD, windupRange) |
| **A** | Increase the selected constant |
| **Y** | Decrease the selected constant |
| **B** | Run test movement (Linear: move 48", Angular: turn 90°) |

---

## Tuning Guide

### Linear PID (Forward/Backward Movement)

1. **Enable the tuner**: Press **X** button
2. **Select Linear controller**: Press **LEFT** arrow (if not already selected)
3. **Tune kP** (Proportional):
   - Start at 0, press **A** to increase
   - Goal: Robot moves toward target quickly
   - Too high: Robot oscillates back and forth
   - Too low: Robot moves too slowly
   - **Test with B button**: Robot moves forward 48 inches
4. **Tune kD** (Derivative):
   - Press **DOWN** twice to select kD
   - Press **A** to add damping
   - Goal: Reduces overshoot and oscillation
   - Test each change with **B** button
5. **Tune kI** (Integral - usually not needed):
   - Only use if robot doesn't quite reach target
   - Very small values (0.001 - 0.01)
6. **Adjust windupRange** if using kI:
   - Press **DOWN** to select windupRange
   - Prevents integral from growing too large

### Angular PID (Turning)

1. **Switch to Angular**: Press **RIGHT** arrow
2. **Tune kP**:
   - Same process as linear
   - **Test with B button**: Robot turns to 90°
3. **Tune kD**:
   - Reduces turn overshoot
   - Test with **B** button after each change
4. **kI and windupRange**: Usually not needed for turning

### Tips for Good Tuning

- **Start with kP = 0, kI = 0, kD = 0**
- **Tune one constant at a time**
- **Use the B button test frequently** to see how changes affect movement
- **Small adjustments**: Use the increment settings to make precise changes
- **kP first, then kD**: This is the most common tuning order
- **kI last**: Only if needed to eliminate steady-state error

---

## Brain Screen Display

When the tuner is enabled, the brain screen shows:

```
Linear Controller          ← Current controller
                          
  kP: 8.500               
> kI: 0.000               ← Arrow shows selected constant
  kD: 43.000              
  Windup: 3.000           
                          
L/R:Switch A:+ Y:-        ← Button hints
U/D:Select B:Test X:Exit  
```

---

## API Reference

### Setup Functions

```cpp
// Set Linear controller PID values
pidTuner.pid_tuner_set_linear(float kP, float kI, float kD, float windupRange);

// Set Angular controller PID values
pidTuner.pid_tuner_set_angular(float kP, float kI, float kD, float windupRange);

// Set adjustment increments (how much each A/Y press changes values)
pidTuner.pid_tuner_increment_p_set(double increment);
pidTuner.pid_tuner_increment_i_set(double increment);
pidTuner.pid_tuner_increment_d_set(double increment);
pidTuner.pid_tuner_increment_windup_set(double increment);
```

### Control Functions

```cpp
// Enable/disable/toggle the tuner
pidTuner.pid_tuner_enable();
pidTuner.pid_tuner_disable();
pidTuner.pid_tuner_toggle();

// Check if tuner is active
bool isActive = pidTuner.pid_tuner_enabled();

// Process controller inputs (call every loop)
pidTuner.pid_tuner_iterate();
```

### Debug Functions

```cpp
// Enable/disable terminal printing
pidTuner.pid_tuner_print_terminal_set(true);

// Get current increment values
double pInc = pidTuner.pid_tuner_increment_p_get();
double iInc = pidTuner.pid_tuner_increment_i_get();
double dInc = pidTuner.pid_tuner_increment_d_get();
double wInc = pidTuner.pid_tuner_increment_windup_get();
```

---

## Test Movements

When you press **B** button while tuning:

### Linear Controller Test
- Resets chassis position to `(0, 0, 0)`
- Executes `chassis.moveToPoint(0, 48, 10000)`
- Robot moves forward 48 inches
- 10 second timeout

### Angular Controller Test
- Resets chassis position to `(0, 0, 0)`
- Executes `chassis.turnToHeading(90, 10000)`
- Robot turns to 90 degrees
- 10 second timeout

---

## Troubleshooting

**Tuner doesn't activate when I press X**
- Make sure you're calling `pidTuner.pid_tuner_iterate()` in your opcontrol loop
- Check that the controller is properly initialized

**Brain screen is blank**
- The tuner automatically initializes the screen when enabled
- Other code might be using `pros::lcd` - the tuner takes over when active

**Robot doesn't move during test (B button)**
- Make sure chassis is calibrated
- Check that motors are configured correctly
- Verify tracking wheels/sensors are working

**Values change too fast/slow**
- Adjust increment values with `pid_tuner_increment_*_set()` functions
- Smaller increments = finer control

**Robot oscillates wildly**
- Your kP is too high - decrease it with Y button
- Add kD to dampen oscillation

**Robot doesn't reach target**
- kP might be too low - increase with A button
- Consider adding small amount of kI (usually not necessary)

---

## Technical Details

### Implementation
- **Header-only**: All code is in `include/pid_tuner.hpp`
- **No LemLib modifications**: Works with unmodified LemLib 0.5.6+
- **Uses placement new**: Updates const PID members by reconstructing objects
- **Real-time updates**: Changes apply immediately to chassis.lateralPID and chassis.angularPID

### How It Works
LemLib's PID class has const members (kP, kI, kD) that can't be modified directly. The tuner uses C++ placement new to:
1. Destroy the existing PID object
2. Construct a new one in the same memory location with updated values
3. This preserves all other chassis state while updating PID constants

### Compatibility
- **LemLib**: 0.5.6+ (tested)
- **PROS**: 4.0+
- **VEX V5**: All V5 brains

---

## Example: Complete Setup

```cpp
#include "main.h"
#include "pid_tuner.hpp"

// Motor and sensor setup
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({4, 5, 6}, pros::MotorGearset::blue);
pros::Imu imu(7);

// LemLib configuration
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10, 
                              lemlib::Omniwheel::NEW_4, 360, 2);
                              
lemlib::ControllerSettings linearController(10, 0, 50, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(4, 0, 40, 3, 1, 100, 3, 500, 0);
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

// PID Tuner
PIDTuner pidTuner(chassis, controller);

void initialize() {
    chassis.calibrate();
    pidTuner.pid_tuner_set_linear(10, 0, 50, 3);
    pidTuner.pid_tuner_set_angular(4, 0, 40, 3);
}

void opcontrol() {
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            pidTuner.pid_tuner_toggle();
        }
        pidTuner.pid_tuner_iterate();
        
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);
        
        pros::delay(10);
    }
}
```

---

## FAQ

**Q: Do I need to modify LemLib source code?**  
A: No! The tuner is completely standalone and works with your existing LemLib installation.

**Q: Can I use this in competition?**  
A: The tuner works, but you'll want to set your final values in code and disable the tuner for competition.

**Q: How do I save my tuned values?**  
A: Copy the values from the brain screen and update your `ControllerSettings` in your code.

**Q: Does this work with tracking wheels?**  
A: Yes, it works with any LemLib chassis configuration.

**Q: Can I tune other PID controllers with this?**  
A: This is specifically for chassis lateral (linear) and angular PIDs. Other PIDs need separate tuning.

---

**Need help?** Check the troubleshooting section above or review the example code.
