/**
 * @file pid_tuner.hpp
 * @brief Robodash PID Tuner
 * @ingroup pid_tuner
 */

#pragma once
#include "robodash/api.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.hpp"
#include <string>

namespace rd {

/**
 * @defgroup pid_tuner PID Tuner
 * @brief Live PID tuning interface
 *
 * Allows real-time adjustment of Lateral and Angular PID values with live telemetry feedback
 */

/**
 * @brief PID Tuner class
 * @ingroup pid_tuner
 */
class PIDTuner {
	/// @addtogroup pid_tuner
	/// @{

  private:
	rd_view_t *view;
	lemlib::Chassis* chassis;
	pros::Controller* controller;

	// UI Container elements
	lv_obj_t *header_bar;
	lv_obj_t *mode_toggle_btn;
	lv_obj_t *mode_toggle_label;
	lv_obj_t *main_container;
	lv_obj_t *left_panel;
	lv_obj_t *right_panel;

	// PID Editor UI (left panel)
	struct pid_row {
		lv_obj_t *container;
		lv_obj_t *label;
		lv_obj_t *minus_btn;
		lv_obj_t *value_label;
		lv_obj_t *plus_btn;
	} pid_rows[4]; // kP, kI, kD, Windup

	// Telemetry UI (right panel)
	lv_obj_t *position_box;
	lv_obj_t *x_label;
	lv_obj_t *y_label;
	lv_obj_t *heading_box;
	lv_obj_t *theta_label;
	lv_obj_t *theta_unit;
	lv_obj_t *tachometer;

	// State
	enum Mode { LAT = 0, ANG = 1 };
	Mode current_mode;
	int selected_row; // 0=kP, 1=kI, 2=kD, 3=Windup
	
	struct PIDValues {
		float kP, kI, kD, windupRange;
	};
	PIDValues lat_values;
	PIDValues ang_values;

	// Increment values
	float p_increment;
	float i_increment;
	float d_increment;
	float windup_increment;

	// Initialize UI
	void init_header();
	void init_main_panels();
	void init_pid_editor();
	void init_telemetry_panel();
	void init_tachometer();

	// Update methods
	void update_mode_toggle();
	void update_pid_displays();
	void update_row_highlight();
	void update_telemetry();
	void draw_tachometer(float theta);
	void handle_controller_input();

	// Button callbacks
	static void mode_toggle_cb(lv_event_t *event);
	static void pid_adjust_cb(lv_event_t *event);
	static void tacho_draw_cb(lv_event_t *event);

	// Helper to get current PID values
	PIDValues& get_current_values();
	void apply_pid_to_chassis();

  public:
	/**
	 * @brief Create a PID Tuner screen
	 * @param name Name to display on screen
	 * @param chassis Pointer to the lemlib chassis
	 * @param controller Pointer to the controller (optional, for controller input)
	 */
	PIDTuner(std::string name, lemlib::Chassis* chassis, pros::Controller* controller = nullptr);

	/**
	 * @brief Update telemetry values from chassis
	 */
	void update();

	/**
	 * @brief Set lateral PID values
	 * @param kP Proportional gain
	 * @param kI Integral gain
	 * @param kD Derivative gain
	 * @param windupRange Windup range
	 */
	void set_lateral_pid(float kP, float kI, float kD, float windupRange);

	/**
	 * @brief Set angular PID values
	 * @param kP Proportional gain
	 * @param kI Integral gain
	 * @param kD Derivative gain
	 * @param windupRange Windup range
	 */
	void set_angular_pid(float kP, float kI, float kD, float windupRange);

	/**
	 * @brief Set increment values for PID adjustment
	 * @param p kP increment
	 * @param i kI increment
	 * @param d kD increment
	 * @param windup Windup increment
	 */
	void set_increments(float p, float i, float d, float windup);

	/**
	 * @brief Focus this view
	 */
	void focus();

	/// @}
};

} // namespace rd
