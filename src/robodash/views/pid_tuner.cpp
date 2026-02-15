#include "robodash/views/pid_tuner.hpp"
#include "robodash/impl/styles.h"
#include "robodash/core.h"
#include <cmath>
#include <new>

// ============================= Constants ============================= //

#define PI 3.14159265358979323846

// ============================= Color Definitions ============================= //

// Matching Motor Telemetry theme
#define COLOR_ACCENT lv_color_hex(0x9333ea)    // Purple accent
#define COLOR_ACCENT_DIM lv_color_hex(0x7c3aed)
#define COLOR_TEXT_DIM lv_color_hex(0x444444)
#define COLOR_TEXT_MED lv_color_hex(0x888888)
#define COLOR_TEXT_BRIGHT lv_color_hex(0xffffff)
#define COLOR_CARD_BG lv_color_hex(0x080808)
#define COLOR_LAT lv_color_hex(0x22c55e)       // Green for Lateral
#define COLOR_ANG lv_color_hex(0x0ea5e9)       // Blue for Angular

// PID constant colors
#define COLOR_KP lv_color_hex(0xa78bfa)        // Purple for kP
#define COLOR_KI lv_color_hex(0x0ea5e9)        // Blue for kI
#define COLOR_KD lv_color_hex(0x22c55e)        // Green for kD
#define COLOR_WINDUP lv_color_hex(0xef4444)    // Red for Windup

// Telemetry colors
#define COLOR_X lv_color_hex(0xef4444)         // Red for X
#define COLOR_Y lv_color_hex(0x22c55e)         // Green for Y
#define COLOR_THETA lv_color_hex(0xa78bfa)     // Purple for Theta

// ============================= Constructor ============================= //

rd::PIDTuner::PIDTuner(std::string name, lemlib::Chassis* chassis, pros::Controller* controller) {
	this->view = rd_view_create(name.c_str());
	this->chassis = chassis;
	this->controller = controller;
	this->current_mode = LAT;
	this->selected_row = 0; // Start with kP selected
	
	// Initialize default values
	lat_values = {0, 0, 0, 0};
	ang_values = {0, 0, 0, 0};
	
	// Default increments
	p_increment = 0.1f;
	i_increment = 0.001f;
	d_increment = 0.5f;
	windup_increment = 0.1f;

	// Set pure black background and account for 32px top bar
	lv_obj_set_style_bg_color(view->obj, color_bg, 0);
	lv_obj_set_style_pad_top(view->obj, 0, 0);
	lv_obj_set_height(view->obj, 240); // 272 - 32 = 240

	// Initialize UI components
	init_header();
	init_main_panels();
	init_pid_editor();
	init_telemetry_panel();
	
	// Initial display update
	update_mode_toggle();
	update_pid_displays();
	update_row_highlight();
}

// ============================= Header Initialization ============================= //

void rd::PIDTuner::init_header() {
	// Header bar - 36px height, full width
	header_bar = lv_obj_create(view->obj);
	lv_obj_set_size(header_bar, LV_PCT(100), 36);
	lv_obj_set_pos(header_bar, 0, 0);
	lv_obj_set_style_bg_color(header_bar, color_bg, 0);
	lv_obj_set_style_border_width(header_bar, 0, 0);
	lv_obj_set_style_border_side(header_bar, LV_BORDER_SIDE_BOTTOM, 0);
	lv_obj_set_style_border_color(header_bar, color_border, 0);
	lv_obj_set_style_pad_all(header_bar, 0, 0);
	lv_obj_set_style_radius(header_bar, 0, 0);
	lv_obj_clear_flag(header_bar, LV_OBJ_FLAG_SCROLLABLE);

	// Container for two-button tab switcher (centered vertically)
	lv_obj_t *tab_container = lv_obj_create(header_bar);
	lv_obj_set_size(tab_container, 130, 26);
	lv_obj_align(tab_container, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_bg_opa(tab_container, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(tab_container, 0, 0);
	lv_obj_set_style_pad_all(tab_container, 0, 0);
	lv_obj_set_style_pad_column(tab_container, 4, 0);
	lv_obj_clear_flag(tab_container, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_layout(tab_container, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(tab_container, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(tab_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	// Lateral button
	mode_toggle_btn = lv_btn_create(tab_container);
	lv_obj_set_size(mode_toggle_btn, 63, 26);
	lv_obj_set_style_bg_color(mode_toggle_btn, COLOR_LAT, 0);
	lv_obj_set_style_border_color(mode_toggle_btn, COLOR_LAT, 0);
	lv_obj_set_style_border_width(mode_toggle_btn, 1, 0);
	lv_obj_set_style_radius(mode_toggle_btn, 4, 0);
	lv_obj_set_style_shadow_width(mode_toggle_btn, 0, 0);
	lv_obj_set_style_bg_color(mode_toggle_btn, COLOR_CARD_BG, LV_STATE_PRESSED);
	lv_obj_add_event_cb(mode_toggle_btn, mode_toggle_cb, LV_EVENT_CLICKED, this);

	mode_toggle_label = lv_label_create(mode_toggle_btn);
	lv_label_set_text(mode_toggle_label, "LAT");
	lv_obj_set_style_text_font(mode_toggle_label, &lv_font_montserrat_12, 0);
	lv_obj_set_style_text_color(mode_toggle_label, lv_color_hex(0x000000), 0);
	lv_obj_center(mode_toggle_label);

	// Angular button
	lv_obj_t *ang_btn = lv_btn_create(tab_container);
	lv_obj_set_size(ang_btn, 63, 26);
	lv_obj_set_style_bg_color(ang_btn, COLOR_CARD_BG, 0);
	lv_obj_set_style_border_color(ang_btn, color_border, 0);
	lv_obj_set_style_border_width(ang_btn, 1, 0);
	lv_obj_set_style_radius(ang_btn, 4, 0);
	lv_obj_set_style_shadow_width(ang_btn, 0, 0);
	lv_obj_set_style_bg_color(ang_btn, COLOR_CARD_BG, LV_STATE_PRESSED);
	lv_obj_set_user_data(ang_btn, (void*)1); // Mark as angular button
	lv_obj_add_event_cb(ang_btn, mode_toggle_cb, LV_EVENT_CLICKED, this);

	lv_obj_t *ang_label = lv_label_create(ang_btn);
	lv_label_set_text(ang_label, "ANG");
	lv_obj_set_style_text_font(ang_label, &lv_font_montserrat_12, 0);
	lv_obj_set_style_text_color(ang_label, COLOR_TEXT_MED, 0);
	lv_obj_center(ang_label);
}

// ============================= Main Panels ============================= //

void rd::PIDTuner::init_main_panels() {
	// Main container (below header)
	main_container = lv_obj_create(view->obj);
	lv_obj_set_size(main_container, LV_PCT(100), 204); // 240 - 36 = 204
	lv_obj_set_pos(main_container, 0, 36);
	lv_obj_set_style_bg_color(main_container, color_bg, 0);
	lv_obj_set_style_border_width(main_container, 0, 0);
	lv_obj_set_style_pad_all(main_container, 8, 0);
	lv_obj_set_style_radius(main_container, 0, 0);
	lv_obj_clear_flag(main_container, LV_OBJ_FLAG_SCROLLABLE);

	// Use flex layout for two columns
	lv_obj_set_layout(main_container, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(main_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

	// Left panel (PID editor) - 230px width
	left_panel = lv_obj_create(main_container);
	lv_obj_set_size(left_panel, 230, LV_PCT(100));
	lv_obj_set_style_bg_color(left_panel, COLOR_CARD_BG, 0);
	lv_obj_set_style_border_color(left_panel, color_border, 0);
	lv_obj_set_style_border_width(left_panel, 1, 0);
	lv_obj_set_style_radius(left_panel, 4, 0);
	lv_obj_set_style_pad_all(left_panel, 8, 0);
	lv_obj_clear_flag(left_panel, LV_OBJ_FLAG_SCROLLABLE);

	// Right panel (telemetry) - 226px width (480 - 16 padding - 230 - 8 gap = 226)
	right_panel = lv_obj_create(main_container);
	lv_obj_set_size(right_panel, 226, LV_PCT(100));
	lv_obj_set_style_bg_color(right_panel, COLOR_CARD_BG, 0);
	lv_obj_set_style_border_color(right_panel, color_border, 0);
	lv_obj_set_style_border_width(right_panel, 1, 0);
	lv_obj_set_style_radius(right_panel, 4, 0);
	lv_obj_set_style_pad_all(right_panel, 8, 0);
	lv_obj_clear_flag(right_panel, LV_OBJ_FLAG_SCROLLABLE);
	
	// Right panel flex layout
	lv_obj_set_layout(right_panel, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(right_panel, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(right_panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
}

// ============================= PID Editor ============================= //

void rd::PIDTuner::init_pid_editor() {
	const char* labels[] = {"kP", "kI", "kD", "Aw"};
	lv_color_t label_colors[] = {COLOR_KP, COLOR_KI, COLOR_KD, COLOR_WINDUP};
	
	// Use flex layout for vertical stacking
	lv_obj_set_layout(left_panel, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(left_panel, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(left_panel, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	for (int i = 0; i < 4; i++) {
		// Row container
		pid_rows[i].container = lv_obj_create(left_panel);
		lv_obj_set_size(pid_rows[i].container, 214, 40);
		lv_obj_set_style_bg_opa(pid_rows[i].container, LV_OPA_TRANSP, 0);
		lv_obj_set_style_border_width(pid_rows[i].container, 0, 0);
		lv_obj_set_style_pad_all(pid_rows[i].container, 0, 0);
		lv_obj_clear_flag(pid_rows[i].container, LV_OBJ_FLAG_SCROLLABLE);
		
		// Use flex layout for row
		lv_obj_set_layout(pid_rows[i].container, LV_LAYOUT_FLEX);
		lv_obj_set_flex_flow(pid_rows[i].container, LV_FLEX_FLOW_ROW);
		lv_obj_set_flex_align(pid_rows[i].container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

		// Label (left) - color coded
		pid_rows[i].label = lv_label_create(pid_rows[i].container);
		lv_label_set_text(pid_rows[i].label, labels[i]);
		lv_obj_set_width(pid_rows[i].label, 50);
		lv_obj_set_style_text_font(pid_rows[i].label, &lv_font_montserrat_12, 0);
		lv_obj_set_style_text_color(pid_rows[i].label, label_colors[i], 0);
		lv_obj_set_style_pad_left(pid_rows[i].label, 8, 0);

		// Minus button - white
		pid_rows[i].minus_btn = lv_btn_create(pid_rows[i].container);
		lv_obj_set_size(pid_rows[i].minus_btn, 32, 32);
		lv_obj_set_style_bg_color(pid_rows[i].minus_btn, COLOR_CARD_BG, 0);
		lv_obj_set_style_border_color(pid_rows[i].minus_btn, color_border, 0);
		lv_obj_set_style_border_width(pid_rows[i].minus_btn, 1, 0);
		lv_obj_set_style_radius(pid_rows[i].minus_btn, 4, 0);
		lv_obj_set_style_shadow_width(pid_rows[i].minus_btn, 0, 0);
		lv_obj_set_style_bg_color(pid_rows[i].minus_btn, color_border, LV_STATE_PRESSED);
		
		// Store row index as user data (negative for minus button)
		lv_obj_set_user_data(pid_rows[i].minus_btn, (void*)(intptr_t)(-i - 1));
		lv_obj_add_event_cb(pid_rows[i].minus_btn, pid_adjust_cb, LV_EVENT_CLICKED, this);

		lv_obj_t *minus_label = lv_label_create(pid_rows[i].minus_btn);
		lv_label_set_text(minus_label, LV_SYMBOL_MINUS);
		lv_obj_set_style_text_font(minus_label, &lv_font_montserrat_14, 0);
		lv_obj_set_style_text_color(minus_label, COLOR_TEXT_BRIGHT, 0);
		lv_obj_center(minus_label);

		// Value display - white
		pid_rows[i].value_label = lv_label_create(pid_rows[i].container);
		lv_label_set_text(pid_rows[i].value_label, "0.000");
		lv_obj_set_width(pid_rows[i].value_label, 60);
		lv_obj_set_style_text_font(pid_rows[i].value_label, &lv_font_montserrat_14, 0);
		lv_obj_set_style_text_color(pid_rows[i].value_label, COLOR_TEXT_BRIGHT, 0);
		lv_obj_set_style_text_align(pid_rows[i].value_label, LV_TEXT_ALIGN_CENTER, 0);

		// Plus button - white
		pid_rows[i].plus_btn = lv_btn_create(pid_rows[i].container);
		lv_obj_set_size(pid_rows[i].plus_btn, 32, 32);
		lv_obj_set_style_bg_color(pid_rows[i].plus_btn, COLOR_CARD_BG, 0);
		lv_obj_set_style_border_color(pid_rows[i].plus_btn, color_border, 0);
		lv_obj_set_style_border_width(pid_rows[i].plus_btn, 1, 0);
		lv_obj_set_style_radius(pid_rows[i].plus_btn, 4, 0);
		lv_obj_set_style_shadow_width(pid_rows[i].plus_btn, 0, 0);
		lv_obj_set_style_bg_color(pid_rows[i].plus_btn, color_border, LV_STATE_PRESSED);
		
		// Store row index as user data (positive for plus button)
		lv_obj_set_user_data(pid_rows[i].plus_btn, (void*)(intptr_t)(i + 1));
		lv_obj_add_event_cb(pid_rows[i].plus_btn, pid_adjust_cb, LV_EVENT_CLICKED, this);

		lv_obj_t *plus_label = lv_label_create(pid_rows[i].plus_btn);
		lv_label_set_text(plus_label, LV_SYMBOL_PLUS);
		lv_obj_set_style_text_font(plus_label, &lv_font_montserrat_14, 0);
		lv_obj_set_style_text_color(plus_label, COLOR_TEXT_BRIGHT, 0);
		lv_obj_center(plus_label);
	}
}

// ============================= Telemetry Panel ============================= //

void rd::PIDTuner::init_telemetry_panel() {
	// Position box (X and Y)
	position_box = lv_obj_create(right_panel);
	lv_obj_set_size(position_box, 210, 48);
	lv_obj_set_style_bg_color(position_box, color_bg, 0);
	lv_obj_set_style_border_color(position_box, color_border, 0);
	lv_obj_set_style_border_width(position_box, 1, 0);
	lv_obj_set_style_radius(position_box, 4, 0);
	lv_obj_set_style_pad_all(position_box, 6, 0);
	lv_obj_clear_flag(position_box, LV_OBJ_FLAG_SCROLLABLE);
	
	// Use flex for X and Y
	lv_obj_set_layout(position_box, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(position_box, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(position_box, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	// X label
	x_label = lv_label_create(position_box);
	lv_label_set_text(x_label, "X: 0.000");
	lv_obj_set_style_text_font(x_label, &lv_font_montserrat_14, 0);
	lv_obj_set_style_text_color(x_label, COLOR_X, 0);

	// Y label
	y_label = lv_label_create(position_box);
	lv_label_set_text(y_label, "Y: 0.000");
	lv_obj_set_style_text_font(y_label, &lv_font_montserrat_14, 0);
	lv_obj_set_style_text_color(y_label, COLOR_Y, 0);

	// Container for heading box and tachometer (side by side)
	lv_obj_t *heading_tacho_container = lv_obj_create(right_panel);
	lv_obj_set_size(heading_tacho_container, 210, 110);
	lv_obj_set_style_bg_opa(heading_tacho_container, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(heading_tacho_container, 0, 0);
	lv_obj_set_style_pad_all(heading_tacho_container, 0, 0);
	lv_obj_clear_flag(heading_tacho_container, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_layout(heading_tacho_container, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(heading_tacho_container, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(heading_tacho_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	// Heading box (left side)
	heading_box = lv_obj_create(heading_tacho_container);
	lv_obj_set_size(heading_box, 100, 100);
	lv_obj_set_style_bg_color(heading_box, color_bg, 0);
	lv_obj_set_style_border_width(heading_box, 0, 0);
	lv_obj_set_style_radius(heading_box, 0, 0);
	lv_obj_set_style_pad_all(heading_box, 6, 0);
	lv_obj_clear_flag(heading_box, LV_OBJ_FLAG_SCROLLABLE);
	
	lv_obj_set_layout(heading_box, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(heading_box, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(heading_box, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	// Big theta value - purple
	theta_label = lv_label_create(heading_box);
	lv_label_set_text(theta_label, "0.00");
	lv_obj_set_style_text_font(theta_label, &lv_font_montserrat_24, 0);
	lv_obj_set_style_text_color(theta_label, COLOR_THETA, 0);

	// Small "deg" unit
	theta_unit = lv_label_create(heading_box);
	lv_label_set_text(theta_unit, "deg");
	lv_obj_set_style_text_font(theta_unit, &lv_font_montserrat_10, 0);
	lv_obj_set_style_text_color(theta_unit, COLOR_TEXT_MED, 0);

	// Tachometer (100x100 canvas) - right side
	init_tachometer();
	// Add tachometer to the container instead of right_panel
	lv_obj_set_parent(tachometer, heading_tacho_container);
}

// ============================= Tachometer ============================= //

void rd::PIDTuner::init_tachometer() {
	// Tachometer container
	tachometer = lv_canvas_create(right_panel);
	lv_obj_set_size(tachometer, 100, 100);
	
	// Create canvas buffer (100x100, indexed 1 bit per pixel for simple drawing)
	static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(100, 100)];
	lv_canvas_set_buffer(tachometer, cbuf, 100, 100, LV_IMG_CF_TRUE_COLOR);
	
	// Fill with black background
	lv_canvas_fill_bg(tachometer, color_bg, LV_OPA_COVER);
	
	// Draw initial tachometer with 0 degrees
	draw_tachometer(0);
}

void rd::PIDTuner::draw_tachometer(float theta) {
	// Clear canvas
	lv_canvas_fill_bg(tachometer, color_bg, LV_OPA_COVER);
	
	int center_x = 50;
	int center_y = 50;
	
	// Draw tick marks around the circle (no numbers, no circle border)
	lv_draw_line_dsc_t tick_dsc;
	lv_draw_line_dsc_init(&tick_dsc);
	tick_dsc.color = color_border;
	tick_dsc.width = 1;
	tick_dsc.opa = LV_OPA_COVER;
	
	for (int angle = 0; angle < 360; angle += 30) {
		float angle_rad = (angle - 90) * PI / 180.0f;
		int outer_len = 45; // All ticks start from outer edge
		int inner_len = (angle % 90 == 0) ? 35 : 40; // Cardinal directions have longer ticks
		
		lv_point_t tick_points[2];
		tick_points[0].x = (lv_coord_t)(center_x + outer_len * cosf(angle_rad));
		tick_points[0].y = (lv_coord_t)(center_y + outer_len * sinf(angle_rad));
		tick_points[1].x = (lv_coord_t)(center_x + inner_len * cosf(angle_rad));
		tick_points[1].y = (lv_coord_t)(center_y + inner_len * sinf(angle_rad));
		lv_canvas_draw_line(tachometer, tick_points, 2, &tick_dsc);
	}
	
	// Draw center dot
	lv_draw_rect_dsc_t dot_dsc;
	lv_draw_rect_dsc_init(&dot_dsc);
	dot_dsc.bg_color = COLOR_THETA;
	dot_dsc.bg_opa = LV_OPA_COVER;
	dot_dsc.border_width = 0;
	dot_dsc.radius = 3;
	
	lv_canvas_draw_rect(tachometer, center_x - 3, center_y - 3, 6, 6, &dot_dsc);
	
	// Draw needle (pointing to theta)
	// Convert theta to radians and adjust for LVGL coordinate system
	float needle_angle = (theta - 90) * PI / 180.0f; // -90 to align 0 degrees to top
	int needle_len = 32;
	lv_coord_t needle_x = (lv_coord_t)(center_x + (int)(needle_len * cosf(needle_angle)));
	lv_coord_t needle_y = (lv_coord_t)(center_y + (int)(needle_len * sinf(needle_angle)));
	
	lv_draw_line_dsc_t line_dsc;
	lv_draw_line_dsc_init(&line_dsc);
	line_dsc.color = COLOR_THETA;
	line_dsc.width = 2;
	line_dsc.opa = LV_OPA_COVER;
	
	lv_point_t points[2];
	points[0].x = (lv_coord_t)center_x;
	points[0].y = (lv_coord_t)center_y;
	points[1].x = needle_x;
	points[1].y = needle_y;
	
	lv_canvas_draw_line(tachometer, points, 2, &line_dsc);
}

// ============================= Update Methods ============================= //

void rd::PIDTuner::update_mode_toggle() {
	// Get both buttons from header
	lv_obj_t *header = lv_obj_get_parent(lv_obj_get_parent(mode_toggle_btn));
	lv_obj_t *tab_container = lv_obj_get_child(header, 0);
	lv_obj_t *lat_btn = lv_obj_get_child(tab_container, 0);
	lv_obj_t *ang_btn = lv_obj_get_child(tab_container, 1);
	
	if (current_mode == LAT) {
		// Highlight LAT button
		lv_obj_set_style_bg_color(lat_btn, COLOR_LAT, 0);
		lv_obj_set_style_border_color(lat_btn, COLOR_LAT, 0);
		lv_obj_t *lat_label = lv_obj_get_child(lat_btn, 0);
		lv_obj_set_style_text_color(lat_label, lv_color_hex(0x000000), 0);
		
		// Dim ANG button
		lv_obj_set_style_bg_color(ang_btn, COLOR_CARD_BG, 0);
		lv_obj_set_style_border_color(ang_btn, color_border, 0);
		lv_obj_t *ang_label = lv_obj_get_child(ang_btn, 0);
		lv_obj_set_style_text_color(ang_label, COLOR_TEXT_MED, 0);
	} else {
		// Dim LAT button
		lv_obj_set_style_bg_color(lat_btn, COLOR_CARD_BG, 0);
		lv_obj_set_style_border_color(lat_btn, color_border, 0);
		lv_obj_t *lat_label = lv_obj_get_child(lat_btn, 0);
		lv_obj_set_style_text_color(lat_label, COLOR_TEXT_MED, 0);
		
		// Highlight ANG button
		lv_obj_set_style_bg_color(ang_btn, COLOR_ANG, 0);
		lv_obj_set_style_border_color(ang_btn, COLOR_ANG, 0);
		lv_obj_t *ang_label = lv_obj_get_child(ang_btn, 0);
		lv_obj_set_style_text_color(ang_label, lv_color_hex(0x000000), 0);
	}
	update_pid_displays();
}

void rd::PIDTuner::update_pid_displays() {
	PIDValues& vals = get_current_values();
	
	// Update value labels with proper precision
	char buf[16];
	
	// kP - 3 decimals
	snprintf(buf, sizeof(buf), "%.3f", vals.kP);
	lv_label_set_text(pid_rows[0].value_label, buf);
	
	// kI - 3 decimals
	snprintf(buf, sizeof(buf), "%.3f", vals.kI);
	lv_label_set_text(pid_rows[1].value_label, buf);
	
	// kD - 1 decimal
	snprintf(buf, sizeof(buf), "%.1f", vals.kD);
	lv_label_set_text(pid_rows[2].value_label, buf);
	
	// Windup - 3 decimals
	snprintf(buf, sizeof(buf), "%.3f", vals.windupRange);
	lv_label_set_text(pid_rows[3].value_label, buf);
}

void rd::PIDTuner::update_telemetry() {
	if (!chassis) return;
	
	lemlib::Pose pose = chassis->getPose();
	
	// Update position (3 decimals) - X red, Y green
	char buf[32];
	
	// X label with red color
	snprintf(buf, sizeof(buf), "X: %.3f", pose.x);
	lv_label_set_text(x_label, buf);
	lv_obj_set_style_text_color(x_label, COLOR_X, 0);
	
	// Y label with green color
	snprintf(buf, sizeof(buf), "Y: %.3f", pose.y);
	lv_label_set_text(y_label, buf);
	lv_obj_set_style_text_color(y_label, COLOR_Y, 0);
	
	// Update heading (2 decimals) - purple already set in init
	snprintf(buf, sizeof(buf), "%.2f", pose.theta);
	lv_label_set_text(theta_label, buf);
	
	// Update tachometer
	draw_tachometer(pose.theta);
}

void rd::PIDTuner::update() {
	handle_controller_input();
	update_telemetry();
}

// ============================= Callbacks ============================= //

void rd::PIDTuner::mode_toggle_cb(lv_event_t *event) {
	rd::PIDTuner *screen = (rd::PIDTuner *)lv_event_get_user_data(event);
	if (!screen) return;
	
	lv_obj_t *btn = lv_event_get_target(event);
	intptr_t btn_type = (intptr_t)lv_obj_get_user_data(btn);
	
	// Set mode based on which button was clicked
	screen->current_mode = (btn_type == 1) ? ANG : LAT;
	screen->update_mode_toggle();
}

void rd::PIDTuner::pid_adjust_cb(lv_event_t *event) {
	rd::PIDTuner *screen = (rd::PIDTuner *)lv_event_get_user_data(event);
	if (!screen) return;
	
	lv_obj_t *btn = lv_event_get_target(event);
	intptr_t data = (intptr_t)lv_obj_get_user_data(btn);
	
	bool is_plus = (data > 0);
	int row_index = is_plus ? (data - 1) : (-data - 1);
	
	PIDValues& vals = screen->get_current_values();
	
	float increment = 0;
	switch (row_index) {
		case 0: increment = screen->p_increment; break;
		case 1: increment = screen->i_increment; break;
		case 2: increment = screen->d_increment; break;
		case 3: increment = screen->windup_increment; break;
	}
	
	float* value_ptr = nullptr;
	switch (row_index) {
		case 0: value_ptr = &vals.kP; break;
		case 1: value_ptr = &vals.kI; break;
		case 2: value_ptr = &vals.kD; break;
		case 3: value_ptr = &vals.windupRange; break;
	}
	
	if (value_ptr) {
		if (is_plus) {
			*value_ptr += increment;
		} else {
			*value_ptr -= increment;
		}
		
		// Clamp to non-negative
		if (*value_ptr < 0) *value_ptr = 0;
	}
	
	screen->update_pid_displays();
	screen->apply_pid_to_chassis();
}

// ============================= Helper Methods ============================= //

rd::PIDTuner::PIDValues& rd::PIDTuner::get_current_values() {
	return (current_mode == LAT) ? lat_values : ang_values;
}

void rd::PIDTuner::apply_pid_to_chassis() {
	if (!chassis) return;
	
	// Destroy and reconstruct PID controllers with new values
	// This follows the pattern from the old pid_tuner.hpp
	chassis->lateralPID.~PID();
	new (&chassis->lateralPID) lemlib::PID(lat_values.kP, lat_values.kI, 
	                                        lat_values.kD, lat_values.windupRange);
	
	chassis->angularPID.~PID();
	new (&chassis->angularPID) lemlib::PID(ang_values.kP, ang_values.kI,
	                                        ang_values.kD, ang_values.windupRange);
}

// ============================= Public Methods ============================= //

void rd::PIDTuner::set_lateral_pid(float kP, float kI, float kD, float windupRange) {
	lat_values.kP = kP;
	lat_values.kI = kI;
	lat_values.kD = kD;
	lat_values.windupRange = windupRange;
	update_pid_displays();
	apply_pid_to_chassis();
}

void rd::PIDTuner::set_angular_pid(float kP, float kI, float kD, float windupRange) {
	ang_values.kP = kP;
	ang_values.kI = kI;
	ang_values.kD = kD;
	ang_values.windupRange = windupRange;
	update_pid_displays();
	apply_pid_to_chassis();
}

void rd::PIDTuner::set_increments(float p, float i, float d, float windup) {
	p_increment = p;
	i_increment = i;
	d_increment = d;
	windup_increment = windup;
}

void rd::PIDTuner::update_row_highlight() {
	// Corresponding colors for each row: kP, kI, kD, Windup
	lv_color_t row_colors[] = {COLOR_KP, COLOR_KI, COLOR_KD, COLOR_WINDUP};
	
	for (int i = 0; i < 4; i++) {
		if (i == selected_row) {
			// Selected row: rounded rectangle with colored border and tinted background
			lv_obj_set_style_border_color(pid_rows[i].container, row_colors[i], 0);
			lv_obj_set_style_border_width(pid_rows[i].container, 2, 0);
			lv_obj_set_style_bg_color(pid_rows[i].container, row_colors[i], 0);
			lv_obj_set_style_bg_opa(pid_rows[i].container, LV_OPA_20, 0);
			lv_obj_set_style_radius(pid_rows[i].container, 8, 0);
		} else {
			// Unselected rows: transparent with no border
			lv_obj_set_style_border_width(pid_rows[i].container, 0, 0);
			lv_obj_set_style_bg_opa(pid_rows[i].container, LV_OPA_TRANSP, 0);
			lv_obj_set_style_radius(pid_rows[i].container, 0, 0);
		}
	}
}

void rd::PIDTuner::handle_controller_input() {
	// Do nothing if no controller is connected or if this view isn't focused
	if (controller == nullptr) return;
	if (rd_view_get_current() != this->view) return;
	
	// LEFT/RIGHT: Switch between LAT and ANG modes
	if (controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
		current_mode = LAT;
		update_mode_toggle();
		update_pid_displays();
	}
	if (controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		current_mode = ANG;
		update_mode_toggle();
		update_pid_displays();
	}
	
	// UP/DOWN: Navigate between PID constants (kP, kI, kD, Windup)
	if (controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
		selected_row = (selected_row - 1 + 4) % 4;  // Wrap around: 0->3
		update_row_highlight();
	}
	if (controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		selected_row = (selected_row + 1) % 4;  // Wrap around: 3->0
		update_row_highlight();
	}
	
	// A: Increment the selected constant
	if (controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
		PIDValues* values = (current_mode == LAT) ? &lat_values : &ang_values;
		
		switch (selected_row) {
			case 0:  // kP
				values->kP += p_increment;
				break;
			case 1:  // kI
				values->kI += i_increment;
				break;
			case 2:  // kD
				values->kD += d_increment;
				break;
			case 3:  // Windup
				values->windupRange += windup_increment;
				break;
		}
		
		update_pid_displays();
		apply_pid_to_chassis();
	}
	
	// Y: Decrement the selected constant
	if (controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
		PIDValues* values = (current_mode == LAT) ? &lat_values : &ang_values;
		
		switch (selected_row) {
			case 0:  // kP
				values->kP -= p_increment;
				if (values->kP < 0) values->kP = 0;
				break;
			case 1:  // kI
				values->kI -= i_increment;
				if (values->kI < 0) values->kI = 0;
				break;
			case 2:  // kD
				values->kD -= d_increment;
				if (values->kD < 0) values->kD = 0;
				break;
			case 3:  // Windup
				values->windupRange -= windup_increment;
				if (values->windupRange < 0) values->windupRange = 0;
				break;
		}
		
		update_pid_displays();
		apply_pid_to_chassis();
	}
}

void rd::PIDTuner::focus() {
	rd_view_focus(this->view);
}
