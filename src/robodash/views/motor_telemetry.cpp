#include "robodash/views/motor_telemetry.hpp"
#include "robodash/apix.h"
#include <cmath>

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
#define COLOR_CARD_BG lv_color_hex(0x080808)
#define COLOR_PROGRESS_TRACK lv_color_hex(0x111111)

// ============================= Metric Data ============================= //

static const char *metric_labels[] = {"VEL", "PWR", "CUR", "TEMP", "TRQ"};
static const char *metric_units[] = {"RPM", "W", "A", "C", "Nm"};
static const float metric_max[] = {600.0f, 11.0f, 2.5f, 65.0f, 2.5f};

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

// ============================= Tab Click Callback ============================= //

void rd::MotorTelemetry::tab_click_cb(lv_event_t *event) {
	lv_obj_t *target = lv_event_get_target(event);
	rd::MotorTelemetry *screen = (rd::MotorTelemetry *)lv_event_get_user_data(event);
	
	// Find which tab was clicked
	for (int i = 0; i < 5; i++) {
		if (screen->tabs[i] == target) {
			screen->active_metric = i;
			
			// Update tab styles
			for (int j = 0; j < 5; j++) {
				if (j == i) {
					// Active tab style
					lv_obj_set_style_bg_color(screen->tabs[j], get_metric_color(i), 0);
					lv_obj_set_style_text_color(screen->tabs[j], lv_color_hex(0x000000), 0);
					lv_obj_set_style_border_width(screen->tabs[j], 0, 0);
				} else {
					// Inactive tab style
					lv_obj_set_style_bg_color(screen->tabs[j], COLOR_CARD_BG, 0);
					lv_obj_set_style_text_color(screen->tabs[j], COLOR_TEXT_DIM, 0);
					lv_obj_set_style_border_width(screen->tabs[j], 1, 0);
					lv_obj_set_style_border_color(screen->tabs[j], color_border, 0);
				}
			}
			break;
		}
	}
}

// ============================= Constructor ============================= //

rd::MotorTelemetry::MotorTelemetry(std::string name, int motor_count) {
	this->view = rd_view_create(name.c_str());
	this->active_metric = 0; // Start with Velocity
	this->motor_count = motor_count > 8 ? 8 : (motor_count < 1 ? 1 : motor_count);

	// Set pure black background and account for 32px top bar
	lv_obj_set_style_bg_color(view->obj, color_bg, 0);
	lv_obj_set_style_pad_top(view->obj, 0, 0);
	lv_obj_set_height(view->obj, 240); // 272 - 32 = 240

	// Initialize UI components
	init_header();
	init_motor_grid(this->motor_count);
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

	// Create tabs container (LEFT-aligned in header)
	lv_obj_t *tabs_container = lv_obj_create(header_bar);
	lv_obj_set_size(tabs_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_set_style_bg_opa(tabs_container, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(tabs_container, 0, 0);
	lv_obj_set_style_pad_all(tabs_container, 0, 0);
	lv_obj_align(tabs_container, LV_ALIGN_LEFT_MID, 8, 0);
	lv_obj_set_flex_flow(tabs_container, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(tabs_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
	lv_obj_set_style_pad_column(tabs_container, 2, 0);
	lv_obj_clear_flag(tabs_container, LV_OBJ_FLAG_SCROLLABLE);

	// Create 5 metric tabs
	for (int i = 0; i < 5; i++) {
		tabs[i] = lv_btn_create(tabs_container);
		lv_obj_set_size(tabs[i], LV_SIZE_CONTENT, LV_SIZE_CONTENT);
		lv_obj_set_style_radius(tabs[i], 2, 0);
		lv_obj_set_style_pad_ver(tabs[i], 2, 0);
		lv_obj_set_style_pad_hor(tabs[i], 6, 0);
		lv_obj_set_style_min_width(tabs[i], 30, 0);
		
		// Initial style (inactive)
		lv_obj_set_style_bg_color(tabs[i], COLOR_CARD_BG, 0);
		lv_obj_set_style_text_color(tabs[i], COLOR_TEXT_DIM, 0);
		lv_obj_set_style_border_width(tabs[i], 1, 0);
		lv_obj_set_style_border_color(tabs[i], color_border, 0);

		// Label
		lv_obj_t *label = lv_label_create(tabs[i]);
		lv_label_set_text(label, metric_labels[i]);
		lv_obj_set_style_text_font(label, &lv_font_montserrat_10, 0);
		lv_obj_center(label);

		// Add click event
		lv_obj_add_event_cb(tabs[i], tab_click_cb, LV_EVENT_CLICKED, this);
	}

	// Set first tab active
	lv_obj_set_style_bg_color(tabs[0], COLOR_VEL, 0);
	lv_obj_set_style_text_color(tabs[0], lv_color_hex(0x000000), 0);
	lv_obj_set_style_border_width(tabs[0], 0, 0);

	// Divider - 1px height
	divider = lv_obj_create(view->obj);
	lv_obj_set_size(divider, LV_PCT(100), 1);
	lv_obj_set_pos(divider, 0, 30);
	lv_obj_set_style_bg_color(divider, color_border, 0);
	lv_obj_set_style_border_width(divider, 0, 0);
	lv_obj_set_style_radius(divider, 0, 0);
}

// ============================= Motor Grid Initialization ============================= //

void rd::MotorTelemetry::init_motor_grid(int count) {
	// Motor grid fills remaining height (240 - 31 = 209px available)
	motor_grid = lv_obj_create(view->obj);
	lv_obj_set_size(motor_grid, LV_PCT(100), 209);
	lv_obj_set_pos(motor_grid, 0, 31);
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
	
	// Calculate cell sizes (209 - 12 padding = 197px available height)
	int grid_width = 480 - 12; // 6px padding on each side
	int grid_height = 209 - 12;
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
	lv_obj_set_style_radius(bar, 0, 0);
	lv_bar_set_range(bar, 0, 100);
	lv_bar_set_value(bar, 0, LV_ANIM_OFF);
	lv_obj_set_style_bg_color(bar, COLOR_VEL, LV_PART_INDICATOR);
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
	float max_val = metric_max[active_metric];
	int percentage = (int)((value / max_val) * 100.0f);
	if (percentage < 0) percentage = 0;
	if (percentage > 100) percentage = 100;
	
	lv_bar_set_value(cards[index].progress_bar, percentage, LV_ANIM_ON);
	lv_obj_set_style_bg_color(cards[index].progress_bar, value_color, LV_PART_INDICATOR);
}

void rd::MotorTelemetry::update_card(int index, const motor_data_t &data, bool is_small) {
	// Update port + name
	char port_text[32];
	snprintf(port_text, sizeof(port_text), "P%d %s", data.port, data.name);
	lv_label_set_text(cards[index].port_label, port_text);

	// Update LED (always based on temperature)
	update_led(cards[index].status_led, data.temp_c);

	// Update metric display
	update_metric_display(index, data, is_small);
}

void rd::MotorTelemetry::update(const std::vector<motor_data_t> &motors) {
	bool is_small = (motor_count >= 5);
	
	int count = motors.size() > motor_count ? motor_count : motors.size();
	for (int i = 0; i < count; i++) {
		update_card(i, motors[i], is_small);
	}
}

void rd::MotorTelemetry::focus() { rd_view_focus(this->view); }
