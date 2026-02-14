/**
 * @file motor_telemetry.hpp
 * @brief Robodash Motor Telemetry
 * @ingroup motor_telemetry
 */

#pragma once
#include "robodash/api.h"
#include "pros/motor_group.hpp"
#include <string>
#include <vector>
#include <tuple>

namespace rd {

/**
 * @defgroup motor_telemetry Motor Telemetry
 * @brief Real-time motor monitoring screen
 *
 * Displays velocity, power, current, temperature, and torque for up to 8 motors
 * with color-coded status indicators and progress bars.
 */

/**
 * @brief Motor data structure
 * @ingroup motor_telemetry
 */
typedef struct motor_data {
	int port;
	const char *name;
	float velocity_rpm;
	float power_w;
	float current_a;
	float temp_c;
	float torque_nm;
	int gearset; // 0=red/100rpm, 1=green/200rpm, 2=blue/600rpm
} motor_data_t;

/**
 * @brief Motor Telemetry class
 * @ingroup motor_telemetry
 */
class MotorTelemetry {
	/// @addtogroup motor_telemetry
	/// @{

  private:
	rd_view_t *view;

	// UI Container elements
	lv_obj_t *header_bar;
	lv_obj_t *motor_grid;

	// Metric selection UI
	lv_obj_t *left_arrow;
	lv_obj_t *right_arrow;
	lv_obj_t *metric_label;

	// Motor card containers (max 8)
	struct motor_card {
		lv_obj_t *container;
		lv_obj_t *status_led;
		lv_obj_t *port_label;
		lv_obj_t *value_label;
		lv_obj_t *unit_label;
		lv_obj_t *progress_bar;
	} cards[8];

	// Current state
	int active_metric; // 0=VEL, 1=PWR, 2=CUR, 3=TEMP, 4=TRQ
	int motor_count;
	
	// Stored motor groups for auto-update
	bool has_stored_groups;
	std::vector<std::tuple<pros::MotorGroup*, const char*>> stored_groups;

	// Initialize UI
	void init_header();
	void init_motor_grid(int count);
	void init_motor_card(int index, bool is_small);

	// Update methods
	void update_card(int index, const motor_data_t &data, bool is_small);
	void update_led(lv_obj_t *led, float temp);
	void update_metric_display(int index, const motor_data_t &data, bool is_small);
	void update_metric_label();

	// Arrow callback helpers
	static void left_arrow_cb(lv_event_t *event);
	static void right_arrow_cb(lv_event_t *event);

  public:
	/**
	 * @brief Create a Motor Telemetry screen
	 * @param name Name to display on screen
	 * @param motor_count Number of motors to display (1-8)
	 */
	MotorTelemetry(std::string name = "Motor Telemetry", int motor_count = 8);
	
	/**
	 * @brief Create a Motor Telemetry screen with auto motor counting
	 * @param name Name to display on screen
	 * @param groups Vector of {motor_group, name} tuples - motor count auto-detected
	 */
	MotorTelemetry(std::string name, const std::vector<std::tuple<pros::MotorGroup*, const char*>> &groups);

	/**
	 * @brief Update all motor data
	 * @param motors Array/vector of motor data
	 */
	void update(const std::vector<motor_data_t> &motors);

	/**
	 * @brief Update motor data from motor groups
	 * @param groups Vector of {motor_group, name} tuples (ports extracted automatically)
	 */
	void update_from_groups(const std::vector<std::tuple<pros::MotorGroup*, const char*>> &groups);
	
	/**
	 * @brief Auto-update using stored motor groups (if constructor with groups was used)
	 */
	void auto_update();

	/**
	 * @brief Set this view to the active view
	 */
	void focus();

	/// @}
};

} // namespace rd
