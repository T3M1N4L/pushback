#include "robodash/views/position.hpp"
#include "robodash/apix.h"
#include "pros/misc.hpp"
#include <cmath>
#include <cstdio>

// ============================= Constants ============================= //

#define PI 3.14159265358979323846

// ============================= Color Definitions ============================= //

#define COLOR_ACCENT lv_color_hex(0x9333ea)    // Purple accent
#define COLOR_TEXT_DIM lv_color_hex(0x444444)
#define COLOR_TEXT_MED lv_color_hex(0x888888)
#define COLOR_TEXT_BRIGHT lv_color_hex(0xffffff)
#define COLOR_POS_X lv_color_hex(0xef4444)     // Red for X
#define COLOR_POS_Y lv_color_hex(0x22c55e)     // Green for Y
#define COLOR_POS_THETA lv_color_hex(0xa78bfa) // Purple for Theta

// ============================= Constructor ============================= //

rd::Position::Position(lemlib::Chassis* chassis, const std::vector<std::string>& field_images, const std::vector<std::string>& field_names, pros::Controller* controller) {
	this->chassis = chassis;
	this->field_paths = field_images;
	this->field_names = field_names;
	this->current_field_index = 0;
	this->controller = controller;

	this->view = rd_view_create("Position");
	lv_obj_set_style_bg_color(view->obj, color_bg, 0);
	lv_obj_clear_flag(view->obj, LV_OBJ_FLAG_SCROLLABLE);

	// Create main container with two panels
	lv_obj_t *main_container = lv_obj_create(view->obj);
	lv_obj_add_style(main_container, &style_transp, 0);
	lv_obj_set_size(main_container, 480, 240);
	lv_obj_align(main_container, LV_ALIGN_TOP_LEFT, 0, 0);
	lv_obj_set_layout(main_container, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_ROW);
	lv_obj_clear_flag(main_container, LV_OBJ_FLAG_SCROLLABLE);

	// Left panel - Field image (240x240)
	lv_obj_t *left_panel = lv_obj_create(main_container);
	lv_obj_add_style(left_panel, &style_transp, 0);
	lv_obj_set_size(left_panel, 240, lv_pct(100));
	lv_obj_set_layout(left_panel, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(left_panel, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(left_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	lv_obj_clear_flag(left_panel, LV_OBJ_FLAG_SCROLLABLE);

	// Right panel - Position display
	lv_obj_t *right_panel = lv_obj_create(main_container);
	lv_obj_add_style(right_panel, &style_transp, 0);
	lv_obj_set_size(right_panel, 230, lv_pct(100));
	lv_obj_set_layout(right_panel, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(right_panel, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(right_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
	lv_obj_clear_flag(right_panel, LV_OBJ_FLAG_SCROLLABLE);

	init_field_display(left_panel);
	init_position_display(right_panel);

	rd_view_set_anims(this->view, RD_ANIM_ON);
}

void rd::Position::init_field_display(lv_obj_t *parent) {
	// Container for field image
	lv_obj_t *field_container = lv_obj_create(parent);
	lv_obj_add_style(field_container, &style_transp, 0);
	lv_obj_set_size(field_container, 240, 240);
	lv_obj_clear_flag(field_container, LV_OBJ_FLAG_SCROLLABLE);

	// Field image
	field_image = lv_img_create(field_container);
	lv_obj_align(field_image, LV_ALIGN_CENTER, 0, 0);
	lv_obj_clear_flag(field_image, LV_OBJ_FLAG_SCROLLABLE);

	// Load initial field image with forced refresh
	if (!field_paths.empty() && pros::usd::is_installed()) {
		std::string full_path = "S:/img/" + field_paths[0];
		lv_img_set_src(field_image, full_path.c_str());
		lv_obj_invalidate(field_image);
		lv_refr_now(NULL);
	}
}

void rd::Position::init_position_display(lv_obj_t *parent) {
	// Field control buttons (above position display)
	lv_obj_t *button_cont = lv_obj_create(parent);
	lv_obj_add_style(button_cont, &style_transp, 0);
	lv_obj_set_size(button_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_set_layout(button_cont, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(button_cont, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(button_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	lv_obj_set_style_pad_column(button_cont, 8, 0);

	// Previous field button
	lv_obj_t *prev_btn = lv_btn_create(button_cont);
	lv_obj_set_size(prev_btn, 32, 32);
	lv_obj_set_style_bg_color(prev_btn, color_bg, 0);
	lv_obj_set_style_border_color(prev_btn, color_border, 0);
	lv_obj_set_style_border_width(prev_btn, 1, 0);
	lv_obj_set_style_radius(prev_btn, 4, 0);
	lv_obj_set_user_data(prev_btn, this);
	lv_obj_add_event_cb(prev_btn, &prev_field_cb, LV_EVENT_CLICKED, nullptr);

	lv_obj_t *prev_img = lv_img_create(prev_btn);
	lv_obj_align(prev_img, LV_ALIGN_CENTER, 0, 0);
	lv_img_set_src(prev_img, LV_SYMBOL_LEFT);
	lv_obj_set_style_text_color(prev_img, COLOR_ACCENT, 0);

	// Field label
	field_label = lv_label_create(button_cont);
	lv_label_set_text(field_label, field_names.empty() ? "Field" : field_names[0].c_str());
	lv_obj_set_style_text_color(field_label, COLOR_ACCENT, 0);
	lv_obj_set_style_text_font(field_label, &lv_font_montserrat_14, 0);

	// Next field button
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
	lv_img_set_src(next_img, LV_SYMBOL_RIGHT);
	lv_obj_set_style_text_color(next_img, COLOR_ACCENT, 0);

	// Position box (X and Y)
	position_box = lv_obj_create(parent);
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
	lv_label_set_text(x_label, "X: 0.00");
	lv_obj_set_style_text_font(x_label, &lv_font_montserrat_14, 0);
	lv_obj_set_style_text_color(x_label, COLOR_POS_X, 0);

	// Y label
	y_label = lv_label_create(position_box);
	lv_label_set_text(y_label, "Y: 0.00");
	lv_obj_set_style_text_font(y_label, &lv_font_montserrat_14, 0);
	lv_obj_set_style_text_color(y_label, COLOR_POS_Y, 0);

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
	lv_obj_set_style_text_color(theta_unit, COLOR_TEXT_MED, 0);

	// Tachometer (100x100 canvas) - right side
	init_tachometer();
	// Add tachometer to the container
	lv_obj_set_parent(tachometer, heading_tacho_container);
}

void rd::Position::init_tachometer() {
	// Tachometer container
	tachometer = lv_canvas_create(view->obj);
	lv_obj_set_size(tachometer, 100, 100);
	
	// Create canvas buffer (100x100)
	static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(100, 100)];
	lv_canvas_set_buffer(tachometer, cbuf, 100, 100, LV_IMG_CF_TRUE_COLOR);
	
	// Fill with black background
	lv_canvas_fill_bg(tachometer, color_bg, LV_OPA_COVER);
	
	// Draw initial tachometer with 0 degrees
	draw_tachometer(0);
}

void rd::Position::draw_tachometer(float theta) {
	// Clear canvas
	lv_canvas_fill_bg(tachometer, color_bg, LV_OPA_COVER);
	
	int center_x = 50;
	int center_y = 50;
	
	// Draw tick marks around the circle
	lv_draw_line_dsc_t tick_dsc;
	lv_draw_line_dsc_init(&tick_dsc);
	tick_dsc.color = color_border;
	tick_dsc.width = 1;
	tick_dsc.opa = LV_OPA_COVER;
	
	for (int angle = 0; angle < 360; angle += 30) {
		float angle_rad = (angle - 90) * PI / 180.0f;
		int outer_len = 45;
		int inner_len = (angle % 90 == 0) ? 35 : 40;
		
		lv_point_t tick_points[2];
		tick_points[0].x = (lv_coord_t)(center_x + outer_len * cosf(angle_rad));
		tick_points[0].y = (lv_coord_t)(center_y + outer_len * sinf(angle_rad));
		tick_points[1].x = (lv_coord_t)(center_x + inner_len * cosf(angle_rad));
		tick_points[1].y = (lv_coord_t)(center_y + inner_len * sinf(angle_rad));
		
		lv_canvas_draw_line(tachometer, tick_points, 2, &tick_dsc);
	}
	
	// Draw heading needle (purple)
	lv_draw_line_dsc_t needle_dsc;
	lv_draw_line_dsc_init(&needle_dsc);
	needle_dsc.color = COLOR_POS_THETA;
	needle_dsc.width = 3;
	needle_dsc.opa = LV_OPA_COVER;
	needle_dsc.round_end = 1;
	needle_dsc.round_start = 1;
	
	float theta_rad = (theta - 90) * PI / 180.0f;
	lv_point_t needle_points[2];
	needle_points[0].x = center_x;
	needle_points[0].y = center_y;
	needle_points[1].x = (lv_coord_t)(center_x + 45 * cosf(theta_rad));
	needle_points[1].y = (lv_coord_t)(center_y + 45 * sinf(theta_rad));
	
	lv_canvas_draw_line(tachometer, needle_points, 2, &needle_dsc);
	
	// Draw center dot
	lv_draw_rect_dsc_t dot_dsc;
	lv_draw_rect_dsc_init(&dot_dsc);
	dot_dsc.bg_color = COLOR_POS_THETA;
	dot_dsc.bg_opa = LV_OPA_COVER;
	dot_dsc.radius = LV_RADIUS_CIRCLE;
	dot_dsc.border_width = 0;
	
	lv_canvas_draw_rect(tachometer, center_x - 4, center_y - 4, 8, 8, &dot_dsc);
}

void rd::Position::prev_field_cb(lv_event_t *event) {
	rd::Position *position = (rd::Position *)lv_obj_get_user_data(lv_event_get_target(event));
	if (!position) return;
	
	position->current_field_index--;
	if (position->current_field_index < 0) {
		position->current_field_index = position->field_paths.size() - 1;
	}
	position->update_field_display();
}

void rd::Position::next_field_cb(lv_event_t *event) {
	rd::Position *position = (rd::Position *)lv_obj_get_user_data(lv_event_get_target(event));
	if (!position) return;
	
	position->current_field_index++;
	if (position->current_field_index >= (int)position->field_paths.size()) {
		position->current_field_index = 0;
	}
	position->update_field_display();
}

void rd::Position::update_field_display() {
	if (current_field_index >= 0 && current_field_index < (int)field_paths.size()) {
		// Update label
		lv_label_set_text(field_label, field_names[current_field_index].c_str());
		
		// Preload image into cache before displaying
		if (pros::usd::is_installed()) {
			std::string full_path = "S:/img/" + field_paths[current_field_index];
			
			// Create temporary hidden image to force cache load
			lv_obj_t *temp_img = lv_img_create(lv_scr_act());
			lv_img_set_src(temp_img, full_path.c_str());
			lv_obj_add_flag(temp_img, LV_OBJ_FLAG_HIDDEN);
			
			// Force complete render cycle to decode image
			lv_refr_now(NULL);
			lv_refr_now(NULL);
			
			// Now set on actual image (should be instant from cache)
			lv_img_set_src(field_image, full_path.c_str());
			lv_obj_invalidate(field_image);
			lv_refr_now(NULL);
			
			// Clean up temp image
			lv_obj_del(temp_img);
		}
	}
}

void rd::Position::update() {
	if (!chassis) return;

	lemlib::Pose pose = chassis->getPose();

	// Update X
	char x_text[32];
	snprintf(x_text, sizeof(x_text), "X: %.2f", pose.x);
	lv_label_set_text(x_label, x_text);

	// Update Y
	char y_text[32];
	snprintf(y_text, sizeof(y_text), "Y: %.2f", pose.y);
	lv_label_set_text(y_label, y_text);

	// Update Theta value
	char theta_text[32];
	snprintf(theta_text, sizeof(theta_text), "%.2f", pose.theta);
	lv_label_set_text(theta_label, theta_text);
	
	// Update tachometer
	draw_tachometer(pose.theta);
	
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
