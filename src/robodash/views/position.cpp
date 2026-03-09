#include "robodash/views/position.hpp"
#include "robodash/apix.h"
#include "pros/misc.hpp"
#include <cmath>
#include <cstdio>

// ==================== Mathematical Constants ====================
#define PI 3.14159265358979323846  // For angle calculations

// ==================== Color Palette ====================
// Custom purple theme matching robodash style
#define COLOR_ACCENT lv_color_hex(0x9333ea)    // Purple accent color (hue 266°)
#define COLOR_TEXT_DIM lv_color_hex(0x444444)     // Dim gray text (secondary info)
#define COLOR_TEXT_MED lv_color_hex(0x888888)     // Medium gray text (labels)
#define COLOR_TEXT_BRIGHT lv_color_hex(0xffffff)  // White text (primary)
#define COLOR_POS_X lv_color_hex(0xef4444)        // Red for X coordinate
#define COLOR_POS_Y lv_color_hex(0x22c55e)        // Green for Y coordinate
#define COLOR_POS_THETA lv_color_hex(0xa78bfa)    // Light purple for heading

// ============================= Position View Constructor ============================= //
// Two-panel layout: Left = Field image (240x240), Right = Position data (230x240)
rd::Position::Position(lemlib::Chassis* chassis, const std::vector<std::string>& field_images, const std::vector<std::string>& field_names, pros::Controller* controller) {
	this->chassis = chassis;               // LemLib chassis for odometry
	this->field_paths = field_images;      // Image paths (relative to S:/img/)
	this->field_names = field_names;       // Display names for field selector
	this->current_field_index = 0;         // Start with first field
	this->controller = controller;         // Controller for haptic feedback (unused currently)

	this->view = rd_view_create("Position");  // Create robodash view
	lv_obj_set_style_bg_color(view->obj, color_bg, 0);  // Dark background
	lv_obj_clear_flag(view->obj, LV_OBJ_FLAG_SCROLLABLE);  // Disable scrolling

	// ==================== Main Container (Flexbox Layout) ====================
	// Split screen: 240px left (field) + 230px right (data) = 470px (480px screen width)
	lv_obj_t *main_container = lv_obj_create(view->obj);
	lv_obj_add_style(main_container, &style_transp, 0);  // Transparent style
	lv_obj_set_size(main_container, 480, 240);           // Full screen
	lv_obj_align(main_container, LV_ALIGN_TOP_LEFT, 0, 0);
	lv_obj_set_layout(main_container, LV_LAYOUT_FLEX);   // Enable flexbox
	lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_ROW);  // Horizontal layout
	lv_obj_clear_flag(main_container, LV_OBJ_FLAG_SCROLLABLE);

	// ==================== Left Panel (Field Image) ====================
	// Square 240x240 area for field visualization
	lv_obj_t *left_panel = lv_obj_create(main_container);
	lv_obj_add_style(left_panel, &style_transp, 0);
	lv_obj_set_size(left_panel, 240, lv_pct(100));  // 240px width, 100% height
	lv_obj_set_layout(left_panel, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(left_panel, LV_FLEX_FLOW_COLUMN);  // Vertical stacking
	lv_obj_set_flex_align(left_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	lv_obj_clear_flag(left_panel, LV_OBJ_FLAG_SCROLLABLE);

	// ==================== Right Panel (Position Display) ====================
	// Remaining space for X/Y/Theta displays and tachometer
	lv_obj_t *right_panel = lv_obj_create(main_container);
	lv_obj_add_style(right_panel, &style_transp, 0);
	lv_obj_set_size(right_panel, 230, lv_pct(100));  // 230px width, 100% height
	lv_obj_set_layout(right_panel, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(right_panel, LV_FLEX_FLOW_COLUMN);  // Stack widgets vertically
	lv_obj_set_flex_align(right_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
	lv_obj_clear_flag(right_panel, LV_OBJ_FLAG_SCROLLABLE);

	init_field_display(left_panel);      // Setup field image with optimization flags
	init_position_display(right_panel);  // Setup X/Y/Theta labels and tachometer

	rd_view_set_anims(this->view, RD_ANIM_ON);  // Enable view transition animations
}

// ============================= Field Image Display Initialization ============================= //
// Optimized for performance: prevents image loading from causing UI lag
void rd::Position::init_field_display(lv_obj_t *parent) {
	// ==================== Field Container ====================
	// Container for field image with performance optimizations
	lv_obj_t *field_container = lv_obj_create(parent);
	lv_obj_add_style(field_container, &style_transp, 0);
	lv_obj_set_size(field_container, 240, 240);
	lv_obj_clear_flag(field_container, LV_OBJ_FLAG_SCROLLABLE);
	
	// CRITICAL OPTIMIZATION: Prevent layout recalculation when image updates
	lv_obj_add_flag(field_container, LV_OBJ_FLAG_IGNORE_LAYOUT);  // Ignore parent layout changes
	
	// CRITICAL OPTIMIZATION: Create opaque layer to isolate redraws
	lv_obj_set_style_bg_opa(field_container, LV_OPA_COVER, 0);  // Fully opaque = stop redraw propagation

	// ==================== Field Image ====================
	// Display field image loaded from SD card
	field_image = lv_img_create(field_container);
	lv_obj_align(field_image, LV_ALIGN_CENTER, 0, 0);
	lv_obj_clear_flag(field_image, LV_OBJ_FLAG_SCROLLABLE);
	
	// CRITICAL OPTIMIZATION: Isolate image from widget tree
	lv_obj_add_flag(field_image, LV_OBJ_FLAG_IGNORE_LAYOUT | LV_OBJ_FLAG_FLOATING);  // Floating = doesn't participate in layout
	
	// Disable anti-aliasing for faster rendering (field images don't need smoothing)
	lv_obj_set_style_img_recolor_opa(field_image, 0, 0);

	// ==================== Load Initial Image ====================
	// Asynchronous loading: doesn't block UI thread
	if (!field_paths.empty() && pros::usd::is_installed()) {
		std::string full_path = "S:/img/" + field_paths[0];  // SD card image path
		lv_img_set_src(field_image, full_path.c_str());      // LVGL loads and caches automatically
	}
}

// ============================= Position Display Initialization ============================= //
// Creates X/Y labels, theta display, tachometer, and field selector buttons
void rd::Position::init_position_display(lv_obj_t *parent) {
	// ==================== Field Selector Buttons ====================
	// Horizontal row of [< Previous] [Field Name] [Next >] buttons
	lv_obj_t *button_cont = lv_obj_create(parent);
	lv_obj_add_style(button_cont, &style_transp, 0);
	lv_obj_set_size(button_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);  // Auto-size to fit buttons
	lv_obj_set_layout(button_cont, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(button_cont, LV_FLEX_FLOW_ROW);  // Horizontal layout
	lv_obj_set_flex_align(button_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	lv_obj_set_style_pad_column(button_cont, 8, 0);  // 8px gap between buttons

	// ==================== Previous Field Button ====================
	lv_obj_t *prev_btn = lv_btn_create(button_cont);
	lv_obj_set_size(prev_btn, 32, 32);  // Small square button
	lv_obj_set_style_bg_color(prev_btn, color_bg, 0);      // Dark background
	lv_obj_set_style_border_color(prev_btn, color_border, 0);  // Border for visibility
	lv_obj_set_style_border_width(prev_btn, 1, 0);
	lv_obj_set_style_radius(prev_btn, 4, 0);  // Slightly rounded corners
	lv_obj_set_user_data(prev_btn, this);     // Store 'this' pointer for callback
	lv_obj_add_event_cb(prev_btn, &prev_field_cb, LV_EVENT_CLICKED, nullptr);  // Click handler

	lv_obj_t *prev_img = lv_img_create(prev_btn);
	lv_obj_align(prev_img, LV_ALIGN_CENTER, 0, 0);
	lv_img_set_src(prev_img, LV_SYMBOL_LEFT);  // Built-in "<" symbol
	lv_obj_set_style_text_color(prev_img, COLOR_ACCENT, 0);  // Purple arrow

	// ==================== Field Name Label ====================
	field_label = lv_label_create(button_cont);
	lv_label_set_text(field_label, field_names.empty() ? "Field" : field_names[0].c_str());  // Show first field name
	lv_obj_set_style_text_color(field_label, COLOR_ACCENT, 0);  // Purple text
	lv_obj_set_style_text_font(field_label, &lv_font_montserrat_14, 0);

	// ==================== Next Field Button ====================
	lv_obj_t *next_btn = lv_btn_create(button_cont);
	lv_obj_set_size(next_btn, 32, 32);
	lv_obj_set_style_bg_color(next_btn, color_bg, 0);
	lv_obj_set_style_border_color(next_btn, color_border, 0);
	lv_obj_set_style_border_width(next_btn, 1, 0);
	lv_obj_set_style_radius(next_btn, 4, 0);
	lv_obj_set_user_data(next_btn, this);
	lv_obj_add_event_cb(next_btn, &next_field_cb, LV_EVENT_CLICKED, nullptr);

	lv_obj_t *next_img = lv_img_create(next_btn);
	lv_obj_align(next_img, LV_ALIGN_CENTER, 0, 0);
	lv_img_set_src(next_img, LV_SYMBOL_RIGHT);  // Built-in ">" symbol
	lv_obj_set_style_text_color(next_img, COLOR_ACCENT, 0);

	// ==================== Position Box (X and Y) ====================
	// Box displaying X and Y coordinates side-by-side
	position_box = lv_obj_create(parent);
	lv_obj_set_size(position_box, 210, 48);  // Wide rectangle for two values
	lv_obj_set_style_bg_color(position_box, color_bg, 0);
	lv_obj_set_style_border_color(position_box, color_border, 0);
	lv_obj_set_style_border_width(position_box, 1, 0);
	lv_obj_set_style_radius(position_box, 4, 0);
	lv_obj_set_style_pad_all(position_box, 6, 0);
	lv_obj_clear_flag(position_box, LV_OBJ_FLAG_SCROLLABLE);
	
	// Use flexbox to distribute X and Y labels evenly
	lv_obj_set_layout(position_box, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(position_box, LV_FLEX_FLOW_ROW);  // Horizontal layout
	lv_obj_set_flex_align(position_box, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);  // Even spacing

	// X coordinate label (red)
	x_label = lv_label_create(position_box);
	lv_label_set_text(x_label, "X: 0.00");  // Default value
	lv_obj_set_style_text_font(x_label, &lv_font_montserrat_14, 0);
	lv_obj_set_style_text_color(x_label, COLOR_POS_X, 0);  // Red

	// Y coordinate label (green)
	y_label = lv_label_create(position_box);
	lv_label_set_text(y_label, "Y: 0.00");
	lv_obj_set_style_text_font(y_label, &lv_font_montserrat_14, 0);
	lv_obj_set_style_text_color(y_label, COLOR_POS_Y, 0);  // Green

	// Container for heading box and tachometer (side by side)
	lv_obj_t *heading_tacho_container = lv_obj_create(parent);
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

	// Big theta value
	theta_label = lv_label_create(heading_box);
	lv_label_set_text(theta_label, "0.00");
	lv_obj_set_style_text_font(theta_label, &lv_font_montserrat_24, 0);
	lv_obj_set_style_text_color(theta_label, COLOR_POS_THETA, 0);

	// Small "deg" unit
	theta_unit = lv_label_create(heading_box);
	lv_label_set_text(theta_unit, "deg");
	lv_obj_set_style_text_font(theta_unit, &lv_font_montserrat_10, 0);
	lv_obj_set_style_text_color(theta_unit, COLOR_TEXT_MED, 0);  // Gray

	// ==================== Tachometer (Compass Visualization) ====================
	// 100x100 canvas with compass needle showing robot heading
	init_tachometer();  // Create canvas and draw initial compass
	lv_obj_set_parent(tachometer, heading_tacho_container);  // Move tachometer into container (right side)
}

// ============================= Tachometer Initialization ============================= //
// Creates 100x100 canvas for drawing compass with tick marks and heading needle
void rd::Position::init_tachometer() {
	// ==================== Canvas Creation ====================
	tachometer = lv_canvas_create(view->obj);  // Create canvas widget
	lv_obj_set_size(tachometer, 100, 100);     // 100x100 square
	
	// ==================== Canvas Buffer ====================
	// Allocate pixel buffer for canvas (true color = RGB565)
	static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(100, 100)];  // 100x100 pixels
	lv_canvas_set_buffer(tachometer, cbuf, 100, 100, LV_IMG_CF_TRUE_COLOR);
	
	// Fill with dark background
	lv_canvas_fill_bg(tachometer, color_bg, LV_OPA_COVER);
	
	// ==================== Initial Draw ====================
	draw_tachometer(0);  // Draw compass needle at 0°
}

// ============================= Tachometer Drawing ============================= //
// Draws compass with tick marks and heading needle (called when heading changes)
void rd::Position::draw_tachometer(float theta) {
	// Clear canvas (erase previous frame)
	lv_canvas_fill_bg(tachometer, color_bg, LV_OPA_COVER);
	
	int center_x = 50;  // Center of 100x100 canvas
	int center_y = 50;
	
	// ==================== Draw Compass Tick Marks ====================
	// 12 tick marks at 30° intervals (like clock hours)
	lv_draw_line_dsc_t tick_dsc;
	lv_draw_line_dsc_init(&tick_dsc);
	tick_dsc.color = color_border;  // Gray tick marks
	tick_dsc.width = 1;
	tick_dsc.opa = LV_OPA_COVER;
	
	for (int angle = 0; angle < 360; angle += 30) {  // Every 30°
		float angle_rad = (angle - 90) * PI / 180.0f;  // Convert to radians, rotate -90° so 0° = top
		int outer_len = 45;  // Tick starts at edge of circle (radius 45)
		int inner_len = (angle % 90 == 0) ? 35 : 40;  // Cardinal directions (N/E/S/W) have longer ticks
		
		// Calculate tick start and end points
		lv_point_t tick_points[2];
		tick_points[0].x = (lv_coord_t)(center_x + outer_len * cosf(angle_rad));  // Outer point
		tick_points[0].y = (lv_coord_t)(center_y + outer_len * sinf(angle_rad));
		tick_points[1].x = (lv_coord_t)(center_x + inner_len * cosf(angle_rad));  // Inner point
		tick_points[1].y = (lv_coord_t)(center_y + inner_len * sinf(angle_rad));
		
		lv_canvas_draw_line(tachometer, tick_points, 2, &tick_dsc);  // Draw tick
	}
	
	// ==================== Draw Heading Needle ====================
	// Purple line from center pointing in robot's heading direction
	lv_draw_line_dsc_t needle_dsc;
	lv_draw_line_dsc_init(&needle_dsc);
	needle_dsc.color = COLOR_POS_THETA;  // Purple
	needle_dsc.width = 3;  // Thick line for visibility
	needle_dsc.opa = LV_OPA_COVER;
	needle_dsc.round_end = 1;    // Rounded endpoints
	needle_dsc.round_start = 1;
	
	float theta_rad = (theta - 90) * PI / 180.0f;  // Convert theta to radians, rotate -90°
	lv_point_t needle_points[2];
	needle_points[0].x = center_x;  // Start at center
	needle_points[0].y = center_y;
	needle_points[1].x = (lv_coord_t)(center_x + 45 * cosf(theta_rad));  // End at edge (radius 45)
	needle_points[1].y = (lv_coord_t)(center_y + 45 * sinf(theta_rad));
	
	lv_canvas_draw_line(tachometer, needle_points, 2, &needle_dsc);  // Draw needle
	
	// ==================== Draw Center Dot ====================
	// Small circle at center for aesthetic
	lv_draw_rect_dsc_t dot_dsc;
	lv_draw_rect_dsc_init(&dot_dsc);
	dot_dsc.bg_color = COLOR_POS_THETA;  // Purple
	dot_dsc.bg_opa = LV_OPA_COVER;
	dot_dsc.radius = LV_RADIUS_CIRCLE;  // Make rectangle a circle
	dot_dsc.border_width = 0;
	
	lv_canvas_draw_rect(tachometer, center_x - 4, center_y - 4, 8, 8, &dot_dsc);  // 8x8 circle
}

// ============================= Field Selector Callbacks ============================= //

// Previous field button: cycle backwards through field list
void rd::Position::prev_field_cb(lv_event_t *event) {
	rd::Position *position = (rd::Position *)lv_obj_get_user_data(lv_event_get_target(event));  // Get Position instance
	if (!position) return;
	
	position->current_field_index--;  // Decrement index
	if (position->current_field_index < 0) {  // Wrap around to end
		position->current_field_index = position->field_paths.size() - 1;
	}
	position->update_field_display();  // Reload image
}

// Next field button: cycle forwards through field list
void rd::Position::next_field_cb(lv_event_t *event) {
	rd::Position *position = (rd::Position *)lv_obj_get_user_data(lv_event_get_target(event));  // Get Position instance
	if (!position) return;
	
	position->current_field_index++;  // Increment index
	if (position->current_field_index >= (int)position->field_paths.size()) {  // Wrap around to start
		position->current_field_index = 0;
	}
	position->update_field_display();  // Reload image
}

// ============================= Field Image Update ============================= //
// Reload field image when user cycles through fields
void rd::Position::update_field_display() {
	if (current_field_index >= 0 && current_field_index < (int)field_paths.size()) {
		// ==================== Update Field Label ====================
		lv_label_set_text(field_label, field_names[current_field_index].c_str());
		
		// ==================== Load Image (with Caching) ====================
		if (pros::usd::is_installed()) {
			std::string full_path = "S:/img/" + field_paths[current_field_index];  // SD card path
			
			// CRITICAL OPTIMIZATION: Only reload if image actually changed
			// Prevents redundant SD card reads and image decoding
			static std::string last_loaded_image = "";
			if (full_path != last_loaded_image) {
				lv_img_set_src(field_image, full_path.c_str());  // LVGL loads and caches automatically
				last_loaded_image = full_path;  // Cache path for next comparison
			}
		}
	}
}

// ============================= Main Update Loop ============================= //
// Called periodically (50ms) to update position display from odometry
void rd::Position::update() {
	if (!chassis) return;  // No chassis = no odometry
	
	// ==================== Thread Safety Check ====================
	// CRITICAL: LVGL is NOT thread-safe - only update when this view is active
	if (rd_view_get_current() != this->view) return;  // Skip if view is not displayed

	// ==================== Get Robot Pose ====================
	lemlib::Pose pose = chassis->getPose();  // X (inches), Y (inches), theta (degrees)

	// ==================== Update X Coordinate ====================
	char x_text[32];
	snprintf(x_text, sizeof(x_text), "X: %.2f", pose.x);  // Format: "X: 12.34"
	lv_label_set_text(x_label, x_text);

	// ==================== Update Y Coordinate ====================
	char y_text[32];
	snprintf(y_text, sizeof(y_text), "Y: %.2f", pose.y);  // Format: "Y: 56.78"
	lv_label_set_text(y_label, y_text);

	// ==================== Update Theta Value ====================
	char theta_text[32];
	snprintf(theta_text, sizeof(theta_text), "%.2f", pose.theta);  // Format: "90.12"
	lv_label_set_text(theta_label, theta_text);
	
	// ==================== Update Tachometer (Throttled) ====================
	// Only redraw tachometer when heading changes by >1° to reduce canvas redraws
	static float last_tachometer_theta = -999.0f;  // Last drawn angle
	if (fabs(pose.theta - last_tachometer_theta) > 1.0f) {  // Significant change
		draw_tachometer(pose.theta);  // Redraw compass needle
		last_tachometer_theta = pose.theta;
	}
	
	// Update controller display when position changes significantly
	if (controller != nullptr) {
		static float last_x = -999, last_y = -999, last_theta = -999;
		static bool first_update = true;
		static bool was_active = false;
		
		// Only update if this is the active view
		bool is_active = (rd_view_get_current() == this->view);
		if (!is_active) {
			was_active = false;
			return;
		}
		
		// Clear screen when view just became active
		if (!was_active) {
			controller->clear();
			pros::delay(50);
			was_active = true;
			first_update = true; // Force update after clearing
		}
		
		// Update if first time or if position changed by more than 0.5 inches or 2 degrees
		if (first_update || fabs(pose.x - last_x) > 0.5 || fabs(pose.y - last_y) > 0.5 || fabs(pose.theta - last_theta) > 2.0) {
			first_update = false;
			last_x = pose.x;
			last_y = pose.y;
			last_theta = pose.theta;
			
			// Line 0: X position
			char line0[32];
			snprintf(line0, sizeof(line0), "X: %.2f\"", pose.x);
			controller->set_text(0, 0, line0);
			pros::delay(50);
			
			// Line 1: Y position
			char line1[32];
			snprintf(line1, sizeof(line1), "Y: %.2f\"", pose.y);
			controller->set_text(1, 0, line1);
			pros::delay(50);
			
			// Line 2: Theta
			char line2[32];
			snprintf(line2, sizeof(line2), "Theta: %.2f deg", pose.theta);
			controller->set_text(2, 0, line2);
			pros::delay(50);
		}
	}
}

void rd::Position::focus() {
	rd_view_focus(this->view);
}
