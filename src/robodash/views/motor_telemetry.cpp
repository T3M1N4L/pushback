#include "robodash/views/motor_telemetry.hpp"
#include "robodash/apix.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/error.h"
#include "pros/rtos.hpp"
#include <cmath>

// ============================= Constants ============================= //

// Grace period after motor reconnects before trusting readings (milliseconds)
static const uint32_t RECONNECT_GRACE_MS = 250;

// ============================= Color Definitions ============================= //

// Use existing robodash colors from colors.c
// color_bg, color_border, color_shade are defined globally

// Custom metric colors for telemetry
#define COLOR_VEL lv_color_hex(0x22c55e)
#define COLOR_PWR lv_color_hex(0x0ea5e9)
#define COLOR_CUR lv_color_hex(0xeab308)
#define COLOR_TEMP_GREEN lv_color_hex(0x22c55e)
#define COLOR_TEMP_YELLOW lv_color_hex(0xeab308)
#define COLOR_TEMP_RED lv_color_hex(0xef4444)
#define COLOR_TRQ lv_color_hex(0xa78bfa)

// UI element colors
#define COLOR_TEXT_DIM lv_color_hex(0x444444)
#define COLOR_TEXT_MED lv_color_hex(0x555555)
#define COLOR_CARD_BG lv_color_hex(0x000000)
#define COLOR_PROGRESS_TRACK lv_color_hex(0x000000)

// ============================= Metric Data ============================= //

static const char *metric_labels[] = {"VEL", "PWR", "CUR", "TEMP", "TRQ"};
static const char *metric_units[] = {"RPM", "W", "A", "C", "Nm"};

static lv_color_t get_metric_color(int metric_index) {
	switch (metric_index) {
		case 0: return COLOR_VEL;
		case 1: return COLOR_PWR;
		case 2: return COLOR_CUR;
		case 4: return COLOR_TRQ;
		default: return COLOR_VEL;
	}
}

static lv_color_t get_temp_color(float temp) {
	if (temp > 55.0f) return COLOR_TEMP_RED;
	if (temp > 45.0f) return COLOR_TEMP_YELLOW;
	return COLOR_TEMP_GREEN;
}

static float get_metric_value(const rd::motor_data_t &data, int metric_index) {
	switch (metric_index) {
		case 0: return data.velocity_rpm;
		case 1: return data.power_w;
		case 2: return data.current_a;
		case 3: return data.temp_c;
		case 4: return data.torque_nm;
		default: return 0.0f;
	}
}

// ============================= Arrow Click Callbacks ============================= //

void rd::MotorTelemetry::left_arrow_cb(lv_event_t *event) {
	rd::MotorTelemetry *screen = (rd::MotorTelemetry *)lv_event_get_user_data(event);
	if (!screen) return;
	
	// Cycle to previous metric (wraps around)
	screen->active_metric--;
	if (screen->active_metric < 0) screen->active_metric = 4;
	
	screen->update_metric_label();
}

void rd::MotorTelemetry::right_arrow_cb(lv_event_t *event) {
	rd::MotorTelemetry *screen = (rd::MotorTelemetry *)lv_event_get_user_data(event);
	if (!screen) return;
	
	// Cycle to next metric (wraps around)
	screen->active_metric++;
	if (screen->active_metric > 4) screen->active_metric = 0;
	
	screen->update_metric_label();
}

// ============================= Constructor ============================= //

rd::MotorTelemetry::MotorTelemetry(std::string name, int motor_count) {
	this->view = rd_view_create(name.c_str());
	this->active_metric = 0; // Start with Velocity
	this->motor_count = motor_count > 8 ? 8 : (motor_count < 1 ? 1 : motor_count);
	this->has_stored_groups = false;
	this->has_stored_motors = false;

	// Set pure black background and account for 32px top bar
	lv_obj_set_style_bg_color(view->obj, color_bg, 0);
	lv_obj_set_style_pad_top(view->obj, 0, 0);
	lv_obj_set_height(view->obj, 240); // 272 - 32 = 240

	// Initialize UI components
	init_header();
	init_motor_grid(this->motor_count);
	update_metric_label();
}

rd::MotorTelemetry::MotorTelemetry(std::string name, const std::vector<std::tuple<pros::MotorGroup*, const char*>> &groups) {
	this->view = rd_view_create(name.c_str());
	this->active_metric = 0; // Start with Velocity
	this->has_stored_groups = true;
	this->has_stored_motors = false;
	this->stored_groups = groups;
	
	// Count total motors from all groups
	int total_motors = 0;
	for (const auto &[group, name] : groups) {
		total_motors += group->get_port_all().size();
	}
	this->motor_count = total_motors > 8 ? 8 : (total_motors < 1 ? 1 : total_motors);

	// Set pure black background and account for 32px top bar
	lv_obj_set_style_bg_color(view->obj, color_bg, 0);
	lv_obj_set_style_pad_top(view->obj, 0, 0);
	lv_obj_set_height(view->obj, 240); // 272 - 32 = 240

	// Initialize UI components
	init_header();
	init_motor_grid(this->motor_count);
	update_metric_label();
}

rd::MotorTelemetry::MotorTelemetry(std::string name, const std::vector<std::tuple<pros::Motor*, const char*>> &motors) {
	this->view = rd_view_create(name.c_str());
	this->active_metric = 0; // Start with Velocity
	this->has_stored_groups = false;
	this->has_stored_motors = true;
	this->stored_motors = motors;
	
	// Count motors
	this->motor_count = motors.size() > 8 ? 8 : (motors.size() < 1 ? 1 : motors.size());

	// Set pure black background and account for 32px top bar
	lv_obj_set_style_bg_color(view->obj, color_bg, 0);
	lv_obj_set_style_pad_top(view->obj, 0, 0);
	lv_obj_set_height(view->obj, 240); // 272 - 32 = 240

	// Initialize UI components
	init_header();
	init_motor_grid(this->motor_count);
	update_metric_label();
}

// ============================= Header Initialization ============================= //

void rd::MotorTelemetry::init_header() {
	// Header bar - 30px height, full width
	header_bar = lv_obj_create(view->obj);
	lv_obj_set_size(header_bar, LV_PCT(100), 30);
	lv_obj_set_pos(header_bar, 0, 0);
	lv_obj_set_style_bg_color(header_bar, color_bg, 0);
	lv_obj_set_style_border_width(header_bar, 0, 0);
	lv_obj_set_style_pad_all(header_bar, 0, 0);
	lv_obj_set_style_radius(header_bar, 0, 0);
	lv_obj_clear_flag(header_bar, LV_OBJ_FLAG_SCROLLABLE);

	// Left arrow button (closer to center)
	left_arrow = lv_btn_create(header_bar);
	lv_obj_add_style(left_arrow, &style_transp, 0);
	lv_obj_set_size(left_arrow, 48, 30);
	lv_obj_set_style_pad_all(left_arrow, 0, 0);
	lv_obj_align(left_arrow, LV_ALIGN_LEFT_MID, 120, 0);
	lv_obj_add_event_cb(left_arrow, left_arrow_cb, LV_EVENT_CLICKED, this);
	lv_obj_set_style_text_opa(left_arrow, 128, LV_STATE_PRESSED);

	lv_obj_t *left_img = lv_img_create(left_arrow);
	lv_obj_align(left_img, LV_ALIGN_CENTER, 0, 0);
	lv_img_set_src(left_img, LV_SYMBOL_LEFT);

	// Center metric label
	metric_label = lv_label_create(header_bar);
	lv_label_set_text(metric_label, "VELOCITY");
	lv_obj_set_style_text_font(metric_label, &lv_font_montserrat_14, 0);
	lv_obj_set_style_pad_all(metric_label, 0, 0);
	lv_obj_align(metric_label, LV_ALIGN_CENTER, 0, 0);

	// Right arrow button (closer to center)
	right_arrow = lv_btn_create(header_bar);
	lv_obj_add_style(right_arrow, &style_transp, 0);
	lv_obj_set_size(right_arrow, 48, 30);
	lv_obj_set_style_pad_all(right_arrow, 0, 0);
	lv_obj_align(right_arrow, LV_ALIGN_RIGHT_MID, -120, 0);
	lv_obj_add_event_cb(right_arrow, right_arrow_cb, LV_EVENT_CLICKED, this);
	lv_obj_set_style_text_opa(right_arrow, 128, LV_STATE_PRESSED);

	lv_obj_t *right_img = lv_img_create(right_arrow);
	lv_obj_align(right_img, LV_ALIGN_CENTER, 0, 0);
	lv_img_set_src(right_img, LV_SYMBOL_RIGHT);
}

// ============================= Motor Grid Initialization ============================= //

void rd::MotorTelemetry::init_motor_grid(int count) {
	// Motor grid fills remaining height (240 - 30 = 210px available)
	motor_grid = lv_obj_create(view->obj);
	lv_obj_set_size(motor_grid, LV_PCT(100), 210);
	lv_obj_set_pos(motor_grid, 0, 30);
	lv_obj_set_style_bg_color(motor_grid, color_bg, 0);
	lv_obj_set_style_border_width(motor_grid, 0, 0);
	lv_obj_set_style_radius(motor_grid, 0, 0);
	lv_obj_set_style_pad_all(motor_grid, 6, 0);
	lv_obj_clear_flag(motor_grid, LV_OBJ_FLAG_SCROLLABLE);

	// Calculate grid layout
	int cols, rows;
	switch (count) {
		case 1: cols = 1; rows = 1; break;
		case 2: cols = 2; rows = 1; break;
		case 3:
		case 4: cols = 2; rows = 2; break;
		case 5:
		case 6: cols = 3; rows = 2; break;
		case 7:
		case 8: cols = 4; rows = 2; break;
		default: cols = 4; rows = 2; break;
	}

	// Use grid layout
	lv_obj_set_layout(motor_grid, LV_LAYOUT_GRID);
	
	// Calculate cell sizes (210 - 12 padding = 198px available height)
	int grid_width = 480 - 12; // 6px padding on each side
	int grid_height = 210 - 12;
	int gap = 6;
	
	int cell_width = (grid_width - (cols - 1) * gap) / cols;
	int cell_height = (grid_height - (rows - 1) * gap) / rows;

	// Set grid template
	static lv_coord_t col_dsc[9];
	static lv_coord_t row_dsc[3];
	
	for (int i = 0; i < cols; i++) {
		col_dsc[i] = cell_width;
	}
	col_dsc[cols] = LV_GRID_TEMPLATE_LAST;
	
	for (int i = 0; i < rows; i++) {
		row_dsc[i] = cell_height;
	}
	row_dsc[rows] = LV_GRID_TEMPLATE_LAST;
	
	lv_obj_set_grid_dsc_array(motor_grid, col_dsc, row_dsc);
	lv_obj_set_style_pad_column(motor_grid, gap, 0);
	lv_obj_set_style_pad_row(motor_grid, gap, 0);

	// Determine if small mode (>= 5 motors)
	bool is_small = (count >= 5);

	// Create motor cards
	for (int i = 0; i < count; i++) {
		init_motor_card(i, is_small);
		
		// Calculate grid position
		int row = i / cols;
		int col = i % cols;
		
		// For partial bottom rows, add offset for centering
		int items_in_row = (row == rows - 1) ? (count - row * cols) : cols;
		if (items_in_row < cols) {
			// Center the partial row
			int offset = (cols - items_in_row) / 2;
			col += offset;
		}
		
		lv_obj_set_grid_cell(cards[i].container, LV_GRID_ALIGN_STRETCH, col, 1,
		                     LV_GRID_ALIGN_STRETCH, row, 1);
	}
}

// ============================= Motor Card Initialization ============================= //

void rd::MotorTelemetry::init_motor_card(int index, bool is_small) {
	// Card container
	lv_obj_t *card = lv_obj_create(motor_grid);
	lv_obj_set_style_bg_color(card, COLOR_CARD_BG, 0);
	lv_obj_set_style_border_width(card, 1, 0);
	lv_obj_set_style_border_color(card, color_border, 0);
	lv_obj_set_style_radius(card, 4, 0); // Rounded corners
	lv_obj_set_style_pad_all(card, 0, 0);
	lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_style_clip_corner(card, true, 0);

	cards[index].container = card;

	// Status LED (top-left corner)
	lv_obj_t *led = lv_obj_create(card);
	int led_size = is_small ? 6 : 8;
	lv_obj_set_size(led, led_size, led_size);
	lv_obj_set_pos(led, 6, 6);
	lv_obj_set_style_radius(led, LV_RADIUS_CIRCLE, 0);
	lv_obj_set_style_border_width(led, 0, 0);
	lv_obj_set_style_bg_color(led, COLOR_TEMP_GREEN, 0);
	lv_obj_set_style_shadow_color(led, COLOR_TEMP_GREEN, 0);
	lv_obj_set_style_shadow_width(led, 6, 0);
	lv_obj_set_style_shadow_spread(led, 0, 0);
	cards[index].status_led = led;

	// Center container for labels
	lv_obj_t *center = lv_obj_create(card);
	lv_obj_set_size(center, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_set_style_bg_opa(center, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(center, 0, 0);
	lv_obj_set_style_pad_all(center, 0, 0);
	lv_obj_center(center);
	lv_obj_set_flex_flow(center, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(center, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	// Port + Name label
	lv_obj_t *port_label = lv_label_create(center);
	lv_label_set_text(port_label, "P0 ---");
	int port_font_size = is_small ? 10 : 13;
	lv_obj_set_style_text_font(port_label, port_font_size <= 10 ? &lv_font_montserrat_10 : &lv_font_montserrat_14, 0);
	lv_obj_set_style_text_color(port_label, COLOR_TEXT_MED, 0);
	cards[index].port_label = port_label;

	// Value label (hero)
	lv_obj_t *value_label = lv_label_create(center);
	lv_label_set_text(value_label, "0");
	int value_font_size = is_small ? 22 : 36;
	const lv_font_t *value_font = is_small ? &lv_font_montserrat_24 : &lv_font_montserrat_36;
	lv_obj_set_style_text_font(value_label, value_font, 0);
	lv_obj_set_style_text_color(value_label, COLOR_VEL, 0);
	lv_obj_set_style_text_align(value_label, LV_TEXT_ALIGN_CENTER, 0);
	lv_obj_set_width(value_label, is_small ? 80 : 100); // Fixed width to prevent shifting
	int value_margin = is_small ? 2 : 4;
	lv_obj_set_style_pad_top(value_label, value_margin, 0);
	cards[index].value_label = value_label;

	// Unit label
	lv_obj_t *unit_label = lv_label_create(center);
	lv_label_set_text(unit_label, "RPM");
	int unit_font_size = is_small ? 10 : 12;
	lv_obj_set_style_text_font(unit_label, unit_font_size <= 10 ? &lv_font_montserrat_10 : &lv_font_montserrat_12, 0);
	lv_obj_set_style_text_color(unit_label, COLOR_TEXT_DIM, 0);
	int unit_margin = is_small ? 1 : 3;
	lv_obj_set_style_pad_top(unit_label, unit_margin, 0);
	cards[index].unit_label = unit_label;

	// Progress bar (bottom, full width)
	lv_obj_t *bar = lv_bar_create(card);
	lv_obj_set_size(bar, lv_pct(100), 3);
	lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, 0);
	lv_obj_set_style_bg_color(bar, COLOR_PROGRESS_TRACK, 0);
	lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
	lv_obj_set_style_border_width(bar, 0, 0);
	lv_obj_set_style_radius(bar, 0, 0);
	lv_bar_set_range(bar, 0, 100);
	lv_bar_set_value(bar, 0, LV_ANIM_OFF);
	lv_obj_set_style_bg_color(bar, COLOR_VEL, LV_PART_INDICATOR);
	lv_obj_set_style_border_width(bar, 0, LV_PART_INDICATOR);
	lv_obj_set_style_radius(bar, 0, LV_PART_INDICATOR);
	lv_obj_set_style_anim_time(bar, 150, 0); // 150ms animation
	cards[index].progress_bar = bar;
}

// ============================= Update Methods ============================= //

void rd::MotorTelemetry::update_led(lv_obj_t *led, float temp) {
	lv_color_t color = get_temp_color(temp);
	lv_obj_set_style_bg_color(led, color, 0);
	lv_obj_set_style_shadow_color(led, color, 0);

	// Pulse animation for critical temperature
	if (temp > 55.0f) {
		static lv_anim_t a;
		lv_anim_init(&a);
		lv_anim_set_var(&a, led);
		lv_anim_set_values(&a, LV_OPA_60, LV_OPA_COVER);
		lv_anim_set_time(&a, 500);
		lv_anim_set_playback_time(&a, 500);
		lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
		lv_anim_set_exec_cb(&a, [](void *obj, int32_t val) {
			lv_obj_set_style_bg_opa((lv_obj_t *)obj, val, 0);
		});
		lv_anim_start(&a);
	} else {
		lv_obj_set_style_bg_opa(led, LV_OPA_COVER, 0);
	}
}

void rd::MotorTelemetry::update_metric_display(int index, const motor_data_t &data, bool is_small) {
	if (!cards[index].value_label) return; // Safety check
	
	if (!data.connected) {
		lv_label_set_text(cards[index].value_label, "--");
		lv_label_set_text(cards[index].unit_label, "No Motor");
		lv_obj_set_style_text_color(cards[index].value_label, COLOR_TEXT_DIM, 0);
		lv_bar_set_value(cards[index].progress_bar, 0, LV_ANIM_OFF);
		return;
	}

	float value = get_metric_value(data, active_metric);
	
	// Update value label with proper formatting
	char value_text[16];
	switch (active_metric) {
		case 0: // VEL - 0 decimals
			snprintf(value_text, sizeof(value_text), "%.0f", value);
			break;
		case 1: // PWR - 1 decimal
			snprintf(value_text, sizeof(value_text), "%.1f", value);
			break;
		case 2: // CUR - 2 decimals
			snprintf(value_text, sizeof(value_text), "%.2f", value);
			break;
		case 3: // TEMP - 1 decimal
			snprintf(value_text, sizeof(value_text), "%.1f", value);
			break;
		case 4: // TRQ - 2 decimals
			snprintf(value_text, sizeof(value_text), "%.2f", value);
			break;
	}
	
	lv_label_set_text(cards[index].value_label, value_text);
	lv_label_set_text(cards[index].unit_label, metric_units[active_metric]);

	// Update value color
	lv_color_t value_color = (active_metric == 3) ? get_temp_color(data.temp_c) : get_metric_color(active_metric);
	lv_obj_set_style_text_color(cards[index].value_label, value_color, 0);

	// Update progress bar
	float max_val;
	if (active_metric == 0) { // Velocity - use gearing-based max
		// Gearing: 0=100rpm (red), 1=200rpm (green), 2=600rpm (blue)
		const float vel_max[] = {100.0f, 200.0f, 600.0f};
		int gear = data.gearing;
		if (gear < 0 || gear > 2) gear = 2; // Default to 600rpm if invalid
		max_val = vel_max[gear];
	} else {
		const float other_max[] = {11.0f, 2.5f, 65.0f, 2.5f}; // PWR, CUR, TEMP, TRQ
		max_val = other_max[active_metric - 1];
	}
	
	// Use absolute value for progress bar (so negative values still show)
	int percentage = (int)((std::fabs(value) / max_val) * 100.0f);
	
	// Always use LV_ANIM_OFF to prevent render conflicts
	lv_bar_set_value(cards[index].progress_bar, percentage, LV_ANIM_OFF);
	lv_obj_set_style_bg_color(cards[index].progress_bar, value_color, LV_PART_INDICATOR);
}

void rd::MotorTelemetry::update_card(int index, const motor_data_t &data, bool is_small) {
	if (!cards[index].container) return; // Safety check
	
	// Update port + name
	char port_text[32];
	snprintf(port_text, sizeof(port_text), "P%d %s", data.port, data.name);
	lv_label_set_text(cards[index].port_label, port_text);

	// Gray out if disconnected
	if (!data.connected) {
		lv_obj_set_style_bg_color(cards[index].status_led, COLOR_TEXT_DIM, 0);
		lv_obj_set_style_shadow_width(cards[index].status_led, 0, 0);
		lv_label_set_text(cards[index].value_label, "--");
		lv_obj_set_style_text_color(cards[index].value_label, COLOR_TEXT_DIM, 0);
		lv_label_set_text(cards[index].unit_label, "No Motor");
		lv_bar_set_value(cards[index].progress_bar, 0, LV_ANIM_OFF);
		return;
	}

	// Update LED (always based on temperature) - No animations
	lv_color_t color = get_temp_color(data.temp_c);
	lv_obj_set_style_bg_color(cards[index].status_led, color, 0);
	lv_obj_set_style_shadow_color(cards[index].status_led, color, 0);
	lv_obj_set_style_bg_opa(cards[index].status_led, LV_OPA_COVER, 0);

	// Update metric display
	update_metric_display(index, data, is_small);
}

void rd::MotorTelemetry::update_metric_label() {
	const char *metric_names[] = {"VELOCITY", "POWER", "CURRENT", "TEMPERATURE", "TORQUE"};
	lv_color_t metric_colors[] = {COLOR_VEL, COLOR_PWR, COLOR_CUR, COLOR_TEMP_GREEN, COLOR_TRQ};
	
	lv_label_set_text(metric_label, metric_names[active_metric]);
	lv_obj_set_style_text_color(metric_label, metric_colors[active_metric], 0);
}

void rd::MotorTelemetry::update(const std::vector<motor_data_t> &motors) {
	bool is_small = (motor_count >= 5);
	
	int count = motors.size() > motor_count ? motor_count : motors.size();
	for (int i = 0; i < count; i++) {
		update_card(i, motors[i], is_small);
	}
}

void rd::MotorTelemetry::update_from_groups(const std::vector<std::tuple<pros::MotorGroup*, const char*>> &groups) {
	std::vector<motor_data_t> motors;
	uint32_t current_time = pros::millis();
	
	for (const auto &[group, name] : groups) {
		// Get port list from motor group
		auto ports = group->get_port_all();
		
		// Query each motor individually using C API
		for (size_t i = 0; i < ports.size(); i++) {
			motor_data_t motor_data;
			int8_t port = ports[i];
			int8_t abs_port = std::abs(port);
			motor_data.port = abs_port;
			motor_data.name = name;
			
			// === ROBUST CONNECTION DETECTION ===
			// Use multiple signals instead of just velocity
			double raw_vel = pros::c::motor_get_actual_velocity(abs_port);
			double raw_temp = pros::c::motor_get_temperature(abs_port);
			int32_t raw_current_ma = pros::c::motor_get_current_draw(abs_port);
			
			// Check all three signals for validity
			bool vel_valid = (raw_vel != PROS_ERR_F && !std::isnan(raw_vel) && !std::isinf(raw_vel));
			bool temp_valid = (raw_temp != PROS_ERR_F && !std::isnan(raw_temp) && raw_temp > 0.0);
			bool current_valid = (raw_current_ma != PROS_ERR);
			
			// Motor is connected if at least 2 out of 3 signals are valid
			// (more robust than single signal check)
			int valid_count = (vel_valid ? 1 : 0) + (temp_valid ? 1 : 0) + (current_valid ? 1 : 0);
			bool is_connected = (valid_count >= 2);
			
			// === RECONNECT GRACE PERIOD ===
			// Track connection state changes
			auto &state = motor_states[abs_port];
			bool just_reconnected = false;
			
			if (is_connected && !state.was_connected) {
				// Motor just reconnected - start grace period
				state.reconnect_time_ms = current_time;
				just_reconnected = true;
			}
			state.was_connected = is_connected;
			
			// During grace period, show as disconnected to avoid stale data
			if (just_reconnected && (current_time - state.reconnect_time_ms) < RECONNECT_GRACE_MS) {
				is_connected = false;
			}
			
			motor_data.connected = is_connected;
			
			if (is_connected) {
				// === GEARSET MAPPING ===
				// PROS motor_gearset_e_t values:
				// E_MOTOR_GEARSET_36 = 0 -> 100 RPM (red)
				// E_MOTOR_GEARSET_18 = 1 -> 200 RPM (green)
				// E_MOTOR_GEARSET_06 = 2 -> 600 RPM (blue)
				pros::motor_gearset_e_t gearset = pros::c::motor_get_gearing(abs_port);
				int gear_idx = (int)gearset;
				
				// Validate gearset or fall back to safe default (blue/600)
				if (gear_idx < 0 || gear_idx > 2) {
					gear_idx = 2; // Default to blue (600 RPM) if invalid
				}
				motor_data.gearing = gear_idx;
				
				// === VELOCITY PROCESSING ===
				// Apply motor reversal to velocity if port was negative
				float velocity = (float)raw_vel;
				if (port < 0) velocity = -velocity;
				
				motor_data.velocity_rpm = velocity;
				motor_data.power_w = (float)pros::c::motor_get_power(abs_port);
				motor_data.current_a = (float)(pros::c::motor_get_current_draw(abs_port) / 1000.0);
				motor_data.temp_c = (float)raw_temp;
				motor_data.torque_nm = (float)pros::c::motor_get_torque(abs_port);
			} else {
				// Disconnected - zero out all values
				motor_data.gearing = 2; // Default to blue for UI display
				motor_data.velocity_rpm = 0;
				motor_data.power_w = 0;
				motor_data.current_a = 0;
				motor_data.temp_c = 0;
				motor_data.torque_nm = 0;
			}
			
			motors.push_back(motor_data);
		}
	}
	
	update(motors);
}

void rd::MotorTelemetry::update_from_motors(const std::vector<std::tuple<pros::Motor*, const char*>> &motors) {
	std::vector<motor_data_t> motor_data_list;
	uint32_t current_time = pros::millis();
	
	for (const auto &[motor, name] : motors) {
		motor_data_t motor_data;
		int8_t port = motor->get_port();
		int8_t abs_port = std::abs(port);
		motor_data.port = abs_port;
		motor_data.name = name;
		
		// === ROBUST CONNECTION DETECTION ===
		// Use multiple signals instead of just velocity
		double raw_vel = pros::c::motor_get_actual_velocity(abs_port);
		double raw_temp = pros::c::motor_get_temperature(abs_port);
		int32_t raw_current_ma = pros::c::motor_get_current_draw(abs_port);
		
		// Check all three signals for validity
		bool vel_valid = (raw_vel != PROS_ERR_F && !std::isnan(raw_vel) && !std::isinf(raw_vel));
		bool temp_valid = (raw_temp != PROS_ERR_F && !std::isnan(raw_temp) && raw_temp > 0.0);
		bool current_valid = (raw_current_ma != PROS_ERR);
		
		// Motor is connected if at least 2 out of 3 signals are valid
		int valid_count = (vel_valid ? 1 : 0) + (temp_valid ? 1 : 0) + (current_valid ? 1 : 0);
		bool is_connected = (valid_count >= 2);
		
		// === RECONNECT GRACE PERIOD ===
		auto &state = motor_states[abs_port];
		bool just_reconnected = false;
		
		if (is_connected && !state.was_connected) {
			// Motor just reconnected - start grace period
			state.reconnect_time_ms = current_time;
			just_reconnected = true;
		}
		state.was_connected = is_connected;
		
		// During grace period, show as disconnected to avoid stale data
		if (just_reconnected && (current_time - state.reconnect_time_ms) < RECONNECT_GRACE_MS) {
			is_connected = false;
		}
		
		motor_data.connected = is_connected;
		
		if (is_connected) {
			// === GEARSET MAPPING ===
			pros::motor_gearset_e_t gearset = pros::c::motor_get_gearing(abs_port);
			int gear_idx = (int)gearset;
			
			// Validate gearset or fall back to safe default (blue/600)
			if (gear_idx < 0 || gear_idx > 2) {
				gear_idx = 2; // Default to blue (600 RPM) if invalid
			}
			motor_data.gearing = gear_idx;
			
			// === VELOCITY PROCESSING ===
			// Get velocity (already accounts for motor reversal via get_port())
			float velocity = (float)raw_vel;
			
			motor_data.velocity_rpm = velocity;
			motor_data.power_w = (float)pros::c::motor_get_power(abs_port);
			motor_data.current_a = (float)(pros::c::motor_get_current_draw(abs_port) / 1000.0);
			motor_data.temp_c = (float)raw_temp;
			motor_data.torque_nm = (float)pros::c::motor_get_torque(abs_port);
		} else {
			// Disconnected - zero out all values
			motor_data.gearing = 2; // Default to blue for UI display
			motor_data.velocity_rpm = 0;
			motor_data.power_w = 0;
			motor_data.current_a = 0;
			motor_data.temp_c = 0;
			motor_data.torque_nm = 0;
		}
		
		motor_data_list.push_back(motor_data);
	}
	
	update(motor_data_list);
}

void rd::MotorTelemetry::auto_update() {
	if (has_stored_groups) {
		update_from_groups(stored_groups);
	} else if (has_stored_motors) {
		update_from_motors(stored_motors);
	}
}

void rd::MotorTelemetry::focus() { rd_view_focus(this->view); }