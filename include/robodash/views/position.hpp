/**
 * @file position.hpp
 * @brief Robodash Position Display
 * @ingroup position
 */

#pragma once
#include "robodash/api.h"
#include "lemlib/chassis/chassis.hpp"
#include <string>
#include <vector>

namespace rd {

/**
 * @defgroup position Position Display
 * @brief Live robot position display with field image
 *
 * Displays real-time robot position (X, Y, Theta) alongside a switchable field image
 */

/**
 * @brief Position Display class
 * @ingroup position
 */
class Position {
	/// @addtogroup position
	/// @{

  private:
	rd_view_t *view;
	lemlib::Chassis* chassis;

	// UI Elements
	lv_obj_t *position_box;
	lv_obj_t *x_label;
	lv_obj_t *y_label;
	lv_obj_t *heading_box;
	lv_obj_t *theta_label;
	lv_obj_t *theta_unit;
	lv_obj_t *tachometer;
	lv_obj_t *field_image;
	lv_obj_t *field_label;
	lv_obj_t *loading_label;

	// Field tracking
	std::vector<std::string> field_paths;
	std::vector<std::string> field_names;
	int current_field_index;

	// UI Setup functions
	void init_position_display(lv_obj_t *parent);
	void init_field_display(lv_obj_t *parent);
	void update_field_display();
	void init_tachometer();
	void draw_tachometer(float theta);

	// Callbacks
	static void prev_field_cb(lv_event_t *event);
	static void next_field_cb(lv_event_t *event);

  public:
	/**
	 * @brief Create a new Position Display
	 *
	 * @param chassis Pointer to the lemlib chassis for position data
	 * @param field_images Vector of field image paths (e.g., {"/fields/skills.bin", "/fields/match.bin"})
	 * @param field_names Vector of field names for display
	 */
	Position(lemlib::Chassis* chassis, const std::vector<std::string>& field_images, const std::vector<std::string>& field_names);

	/**
	 * @brief Update position display with current chassis data
	 */
	void update();

	/**
	 * @brief Set this view to the active view
	 */
	void focus();

	/// @}
};

} // namespace rd
