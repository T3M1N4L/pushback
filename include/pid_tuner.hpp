#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.hpp"
#include "pros/llemu.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <new>
#include <cmath>

/**
 * @brief Standalone PID Tuner for LemLib
 * 
 * Header-only PID tuner that works with your existing lemlib::Chassis.
 * No separate .cpp file needed - everything is inline.
 */
class PIDTuner {
    private:
        lemlib::Chassis& chassis;
        pros::Controller& controller;
        
        bool tuner_enabled;
        bool tuner_print_terminal;
        int tuner_current_controller; // 0 = Linear, 1 = Angular
        int tuner_current_constant;   // 0 = kP, 1 = kI, 2 = kD, 3 = windupRange

        struct PIDValues {
            float kP, kI, kD, windupRange;
        };
        PIDValues tuner_linear_values;
        PIDValues tuner_angular_values;

        double tuner_p_increment, tuner_i_increment, tuner_d_increment, tuner_windup_increment;

        void tuner_update_display() {
            if (!tuner_enabled) return;
            pros::lcd::clear();
            std::string title = (tuner_current_controller == 0 ? "Linear" : "Angular") + std::string(" Controller");
            pros::lcd::set_text(0, title);
            
            PIDValues& v = tuner_current_controller == 0 ? tuner_linear_values : tuner_angular_values;
            std::string kpLine = "  kP: " + tuner_format_value(v.kP);
            std::string kiLine = "  kI: " + tuner_format_value(v.kI);
            std::string kdLine = "  kD: " + tuner_format_value(v.kD);
            std::string windupLine = "  Windup: " + tuner_format_value(v.windupRange);
            
            if (tuner_current_constant == 0) kpLine = "> " + kpLine.substr(2);
            if (tuner_current_constant == 1) kiLine = "> " + kiLine.substr(2);
            if (tuner_current_constant == 2) kdLine = "> " + kdLine.substr(2);
            if (tuner_current_constant == 3) windupLine = "> " + windupLine.substr(2);
            
            pros::lcd::set_text(1, kpLine);
            pros::lcd::set_text(2, kiLine);
            pros::lcd::set_text(3, kdLine);
            pros::lcd::set_text(4, windupLine);
            pros::lcd::set_text(5, "");
            pros::lcd::set_text(6, "L/R:Switch A:+ Y:-");
            pros::lcd::set_text(7, "U/D:Select B:Test X:Exit");
            
            if (tuner_print_terminal) tuner_print_values();
        }

        void tuner_update_pid() {
            chassis.lateralPID.~PID();
            new (&chassis.lateralPID) lemlib::PID(tuner_linear_values.kP, tuner_linear_values.kI, 
                                                   tuner_linear_values.kD, tuner_linear_values.windupRange);
            chassis.angularPID.~PID();
            new (&chassis.angularPID) lemlib::PID(tuner_angular_values.kP, tuner_angular_values.kI,
                                                   tuner_angular_values.kD, tuner_angular_values.windupRange);
        }

        void tuner_increase_value() {
            PIDValues& v = tuner_current_controller == 0 ? tuner_linear_values : tuner_angular_values;
            if (tuner_current_constant == 0) v.kP += tuner_p_increment;
            else if (tuner_current_constant == 1) v.kI += tuner_i_increment;
            else if (tuner_current_constant == 2) v.kD += tuner_d_increment;
            else if (tuner_current_constant == 3) v.windupRange += tuner_windup_increment;
            if (v.kP < 0) v.kP = 0;
            if (v.kI < 0) v.kI = 0;
            if (v.kD < 0) v.kD = 0;
            if (v.windupRange < 0) v.windupRange = 0;
            tuner_update_pid();
            tuner_update_display();
        }

        void tuner_decrease_value() {
            PIDValues& v = tuner_current_controller == 0 ? tuner_linear_values : tuner_angular_values;
            if (tuner_current_constant == 0) v.kP -= tuner_p_increment;
            else if (tuner_current_constant == 1) v.kI -= tuner_i_increment;
            else if (tuner_current_constant == 2) v.kD -= tuner_d_increment;
            else if (tuner_current_constant == 3) v.windupRange -= tuner_windup_increment;
            if (v.kP < 0) v.kP = 0;
            if (v.kI < 0) v.kI = 0;
            if (v.kD < 0) v.kD = 0;
            if (v.windupRange < 0) v.windupRange = 0;
            tuner_update_pid();
            tuner_update_display();
        }

        void tuner_move_up() { 
            if (--tuner_current_constant < 0) tuner_current_constant = 3; 
            tuner_update_display(); 
        }
        void tuner_move_down() { 
            if (++tuner_current_constant > 3) tuner_current_constant = 0; 
            tuner_update_display(); 
        }
        void tuner_move_left() { tuner_current_controller = 0; tuner_update_display(); }
        void tuner_move_right() { tuner_current_controller = 1; tuner_update_display(); }

        void tuner_run_test() {
            if (!tuner_enabled) return;
            
            // Reset chassis position to 0,0,0
            chassis.setPose(0, 0, 0);
            
            if (tuner_current_controller == 0) {
                // Linear PID test: move to point (0, 48)
                controller.rumble(".");
                chassis.moveToPoint(0, 48, 10000);
            } else {
                // Angular PID test: turn to heading 90
                controller.rumble(".");
                chassis.turnToHeading(90, 10000);
            }
        }

        void tuner_print_values() {
            std::cout << "\n=== PID Tuner Values ===" << std::endl;
            std::cout << "Linear: kP=" << tuner_linear_values.kP << " kI=" << tuner_linear_values.kI 
                      << " kD=" << tuner_linear_values.kD << " windup=" << tuner_linear_values.windupRange << std::endl;
            std::cout << "Angular: kP=" << tuner_angular_values.kP << " kI=" << tuner_angular_values.kI 
                      << " kD=" << tuner_angular_values.kD << " windup=" << tuner_angular_values.windupRange << std::endl;
        }

        std::string tuner_format_value(float value, int precision = 3) {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(precision) << value;
            return stream.str();
        }

    public:
        PIDTuner(lemlib::Chassis& chassis, pros::Controller& controller)
            : chassis(chassis), controller(controller), tuner_enabled(false), tuner_print_terminal(false),
              tuner_current_controller(0), tuner_current_constant(0), tuner_p_increment(0.1),
              tuner_i_increment(0.001), tuner_d_increment(0.5), tuner_windup_increment(0.1) {
            tuner_linear_values = {0, 0, 0, 0};
            tuner_angular_values = {0, 0, 0, 0};
        }

        void pid_tuner_enable() {
            if (!tuner_enabled) {
                tuner_enabled = true;
                pros::lcd::initialize();
                tuner_update_display();
            }
        }

        void pid_tuner_disable() {
            if (tuner_enabled) {
                tuner_enabled = false;
                pros::lcd::shutdown();
            }
        }

        void pid_tuner_toggle() { tuner_enabled ? pid_tuner_disable() : pid_tuner_enable(); }
        bool pid_tuner_enabled() { return tuner_enabled; }

        void pid_tuner_iterate() {
            if (!tuner_enabled) return;
            
            // Controller switching (Left/Right)
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) tuner_move_left();
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) tuner_move_right();
            
            // Constant selection (Up/Down)
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) tuner_move_up();
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) tuner_move_down();
            
            // Value adjustment (A/Y)
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) tuner_increase_value();
            else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) tuner_decrease_value();
            
            // Test movement (B button)
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) tuner_run_test();
        }

        void pid_tuner_increment_p_set(double p) { tuner_p_increment = std::abs(p); }
        void pid_tuner_increment_i_set(double i) { tuner_i_increment = std::abs(i); }
        void pid_tuner_increment_d_set(double d) { tuner_d_increment = std::abs(d); }
        void pid_tuner_increment_windup_set(double w) { tuner_windup_increment = std::abs(w); }
        
        double pid_tuner_increment_p_get() { return tuner_p_increment; }
        double pid_tuner_increment_i_get() { return tuner_i_increment; }
        double pid_tuner_increment_d_get() { return tuner_d_increment; }
        double pid_tuner_increment_windup_get() { return tuner_windup_increment; }
        
        void pid_tuner_print_terminal_set(bool input) { tuner_print_terminal = input; }
        bool pid_tuner_print_terminal_enabled() { return tuner_print_terminal; }

        void pid_tuner_set_linear(float kP, float kI, float kD, float windupRange) {
            tuner_linear_values = {kP, kI, kD, windupRange};
            tuner_update_pid();
        }

        void pid_tuner_set_angular(float kP, float kI, float kD, float windupRange) {
            tuner_angular_values = {kP, kI, kD, windupRange};
            tuner_update_pid();
        }
};
