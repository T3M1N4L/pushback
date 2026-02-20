#include "robodash/apix.h"
#include "pros/misc.hpp"

const int view_menu_width = 240;

// ============================== UI Elements ============================== //

lv_obj_t *screen;
lv_obj_t *view_cont;

lv_obj_t *shade;
lv_obj_t *view_menu;
lv_obj_t *view_list;
lv_obj_t *alert_cont;
lv_obj_t *alert_btn;
lv_obj_t *anim_label;
lv_obj_t *battery_icon;
lv_obj_t *battery_percent;
lv_obj_t *battery_charging;

lv_anim_t anim_sidebar_open;
lv_anim_t anim_sidebar_close;
lv_anim_t anim_shade_hide;
lv_anim_t anim_shade_show;

rd_view_t *current_view;

// ========================== Forward Declarations ========================== //

void update_battery_indicator();

// ============================ Helper Functions============================ //

bool valid_view(rd_view_t *view) {
	if (view == NULL) return false;

	for (int i = 0; i < lv_obj_get_child_cnt(view_list); i++) {
		lv_obj_t *child = lv_obj_get_child(view_list, i);
		rd_view_t *reg_view = (rd_view_t *)lv_obj_get_user_data(child);
		if (reg_view == view) return true;
	}

	return false;
}

// ============================== UI Callbacks ============================== //

void view_focus_cb(lv_event_t *event) {
	rd_view_t *view = (rd_view_t *)lv_event_get_user_data(event);
	rd_view_focus(view);
}

void views_btn_cb(lv_event_t *event) {
	lv_obj_clear_flag(view_menu, LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(shade, LV_OBJ_FLAG_HIDDEN);
	
	// Update battery when view menu opens
	update_battery_indicator();

	if (rd_view_get_anims(current_view) == RD_ANIM_ON) {
		lv_anim_start(&anim_sidebar_open);
		lv_anim_start(&anim_shade_show);
	}
}

void close_cb(lv_event_t *event) {
	if (lv_obj_get_child_cnt(alert_cont) > 0) {
		lv_obj_clear_flag(alert_btn, LV_OBJ_FLAG_HIDDEN);
	}

	lv_obj_add_flag(alert_cont, LV_OBJ_FLAG_HIDDEN);

	if (rd_view_get_anims(current_view) == RD_ANIM_ON) {
		lv_anim_start(&anim_sidebar_close);
		lv_anim_start(&anim_shade_hide);
	} else {
		lv_obj_add_flag(view_menu, LV_OBJ_FLAG_HIDDEN);
		lv_obj_add_flag(shade, LV_OBJ_FLAG_HIDDEN);
	}
}

void alert_btn_cb(lv_event_t *event) {
	if (!lv_obj_has_flag(alert_cont, LV_OBJ_FLAG_HIDDEN)) return;
	lv_obj_add_flag(alert_btn, LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(alert_cont, LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(shade, LV_OBJ_FLAG_HIDDEN);
	if (rd_view_get_anims(current_view) == RD_ANIM_ON) lv_anim_start(&anim_shade_show);
}

void alert_cb(lv_event_t *event) {
	rd_view_t *view = (rd_view_t *)lv_event_get_user_data(event);
	rd_view_focus(view);

	lv_obj_t *alert = lv_event_get_target(event);
	lv_obj_del(alert);

	if (lv_obj_get_child_cnt(alert_cont) == 0) {
		lv_obj_add_flag(alert_cont, LV_OBJ_FLAG_HIDDEN);

		if (rd_view_get_anims(current_view) == RD_ANIM_ON) {
			lv_anim_start(&anim_shade_hide);
		} else {
			lv_obj_add_flag(shade, LV_OBJ_FLAG_HIDDEN);
		}
	}
}

// ============================ Battery Update ============================ //

void update_battery_indicator() {
	if (!battery_icon || !battery_percent) return;
	
	double capacity = pros::battery::get_capacity();
	int32_t current = pros::battery::get_current();
	
	// Update percentage text
	char battery_text[8];
	snprintf(battery_text, sizeof(battery_text), "%.0f%%", capacity);
	lv_label_set_text(battery_percent, battery_text);
	
	// Determine color based on capacity
	lv_color_t battery_color;
	if (capacity > 90.0) {
		battery_color = lv_color_hex(0x22c55e);  // Green (same as motor telemetry)
	} else if (capacity < 10.0) {
		battery_color = lv_color_hex(0xef4444);  // Red (same as motor telemetry)
	} else if (capacity < 30.0) {
		battery_color = lv_color_hex(0xeab308);  // Yellow (same as motor telemetry)
	} else {
		battery_color = lv_color_hex(0xFFFFFF);  // White
	}
	
	// Determine icon based on capacity
	const char *battery_symbol;
	if (capacity > 75.0) {
		battery_symbol = LV_SYMBOL_BATTERY_FULL;
	} else if (capacity > 50.0) {
		battery_symbol = LV_SYMBOL_BATTERY_3;
	} else if (capacity > 25.0) {
		battery_symbol = LV_SYMBOL_BATTERY_2;
	} else if (capacity > 10.0) {
		battery_symbol = LV_SYMBOL_BATTERY_1;
	} else {
		battery_symbol = LV_SYMBOL_BATTERY_EMPTY;
	}
	
	// Apply updates
	lv_img_set_src(battery_icon, battery_symbol);
	lv_obj_set_style_img_recolor(battery_icon, battery_color, 0);
	lv_obj_set_style_text_color(battery_percent, battery_color, 0);
}

// =========================== UI Initialization =========================== //

void create_ui() {
	screen = lv_scr_act();
	lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

	view_cont = lv_obj_create(screen);
	lv_obj_set_size(view_cont, 480, 240);
	lv_obj_add_style(view_cont, &style_bg, 0);
	lv_obj_align(view_cont, LV_ALIGN_TOP_LEFT, 0, 0);

	// ---------------------------- Top Buttons ---------------------------- //

	lv_obj_t *views_open_btn = lv_btn_create(screen);
	lv_obj_set_size(views_open_btn, 32, 32);
	lv_obj_add_style(views_open_btn, &style_core_button, 0);
	lv_obj_add_style(views_open_btn, &style_core_button_pr, LV_STATE_PRESSED);
	lv_obj_align(views_open_btn, LV_ALIGN_TOP_RIGHT, -4, 4);
	lv_obj_add_event_cb(views_open_btn, views_btn_cb, LV_EVENT_PRESSED, NULL);

	lv_obj_t *open_img = lv_img_create(views_open_btn);
	lv_img_set_src(open_img, LV_SYMBOL_BARS);
	lv_obj_set_style_img_recolor(open_img, color_text, 0);
	lv_obj_set_style_img_recolor_opa(open_img, LV_OPA_COVER, 0);
	lv_obj_set_style_radius(open_img, 2, 0);
	lv_obj_align(open_img, LV_ALIGN_CENTER, 0, 0);

	alert_btn = lv_btn_create(screen);
	lv_obj_set_size(alert_btn, 32, 32);
	lv_obj_add_style(alert_btn, &style_core_button, 0);
	lv_obj_add_style(alert_btn, &style_core_button_pr, LV_STATE_PRESSED);
	lv_obj_align(alert_btn, LV_ALIGN_TOP_RIGHT, -42, 4);
	lv_obj_add_event_cb(alert_btn, alert_btn_cb, LV_EVENT_PRESSED, NULL);
	lv_obj_add_flag(alert_btn, LV_OBJ_FLAG_HIDDEN);

	lv_obj_t *alert_img = lv_img_create(alert_btn);
	lv_img_set_src(alert_img, LV_SYMBOL_BELL);
	lv_obj_set_style_img_recolor(alert_img, color_text, 0);
	lv_obj_set_style_img_recolor_opa(alert_img, LV_OPA_COVER, 0);
	lv_obj_align(alert_img, LV_ALIGN_CENTER, 0, 0);

	// ------------------------------- Shade ------------------------------- //

	shade = lv_obj_create(screen);
	lv_obj_set_size(shade, lv_pct(100), lv_pct(100));
	lv_obj_add_style(shade, &style_core_shade, 0);
	lv_obj_add_flag(shade, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_event_cb(shade, close_cb, LV_EVENT_PRESSED, NULL);

	// ----------------------------- View Menu ----------------------------- //

	view_menu = lv_obj_create(screen);
	lv_obj_set_size(view_menu, view_menu_width, 240);
	lv_obj_align(view_menu, LV_ALIGN_TOP_RIGHT, 0, 0);
	lv_obj_add_style(view_menu, &style_core_bg, 0);
	lv_obj_add_flag(view_menu, LV_OBJ_FLAG_HIDDEN);

	lv_obj_t *title = lv_label_create(view_menu);
	lv_label_set_text(title, "Select View");
	lv_obj_add_style(title, &style_text_large, 0);
	lv_obj_align(title, LV_ALIGN_TOP_LEFT, 12, 12);

	lv_obj_t *views_close_btn = lv_btn_create(view_menu);
	lv_obj_set_size(views_close_btn, 32, 32);
	lv_obj_add_style(views_close_btn, &style_transp, 0);
	lv_obj_add_style(views_close_btn, &style_transp, LV_STATE_PRESSED);
	lv_obj_align(views_close_btn, LV_ALIGN_TOP_RIGHT, -4, 4);
	lv_obj_add_event_cb(views_close_btn, close_cb, LV_EVENT_PRESSED, NULL);

	lv_obj_t *close_img = lv_img_create(views_close_btn);
	lv_img_set_src(close_img, LV_SYMBOL_CLOSE);
	lv_obj_align(close_img, LV_ALIGN_CENTER, 0, 0);

	view_list = lv_list_create(view_menu);
	lv_obj_set_size(view_list, 200, lv_pct(100) - 32);
	lv_obj_add_style(view_list, &style_core_list, 0);
	lv_obj_align(view_list, LV_ALIGN_TOP_LEFT, 4, 36);
	lv_obj_add_flag(view_list, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
	lv_obj_set_scroll_dir(view_list, LV_DIR_VER);

	// View list scroll buttons
	lv_obj_t *view_btns = lv_obj_create(view_menu);
	lv_obj_add_style(view_btns, &style_transp, 0);
	lv_obj_set_size(view_btns, 32, lv_pct(100) - 32);
	lv_obj_align(view_btns, LV_ALIGN_TOP_RIGHT, -4, 36);
	lv_obj_clear_flag(view_btns, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_layout(view_btns, LV_LAYOUT_FLEX);
	lv_obj_set_flex_flow(view_btns, LV_FLEX_FLOW_COLUMN);
	lv_obj_set_flex_align(view_btns, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	// Up scroll button
	lv_obj_t *view_up_btn = lv_btn_create(view_btns);
	lv_obj_add_style(view_up_btn, &style_transp, 0);
	lv_obj_set_size(view_up_btn, 32, 32);
	lv_obj_set_style_text_opa(view_up_btn, 128, LV_STATE_PRESSED);
	lv_obj_set_flex_grow(view_up_btn, 1);
	lv_obj_add_event_cb(view_up_btn, [](lv_event_t *e) {
		lv_obj_scroll_by_bounded(view_list, 0, 40, LV_ANIM_ON);
	}, LV_EVENT_CLICKED, nullptr);

	lv_obj_t *view_up_img = lv_img_create(view_up_btn);
	lv_obj_align(view_up_img, LV_ALIGN_CENTER, 0, 0);
	lv_img_set_src(view_up_img, LV_SYMBOL_UP);

	// Down scroll button
	lv_obj_t *view_down_btn = lv_btn_create(view_btns);
	lv_obj_add_style(view_down_btn, &style_transp, 0);
	lv_obj_set_size(view_down_btn, 32, 32);
	lv_obj_set_style_text_opa(view_down_btn, 128, LV_STATE_PRESSED);
	lv_obj_set_flex_grow(view_down_btn, 1);
	lv_obj_add_event_cb(view_down_btn, [](lv_event_t *e) {
		lv_obj_scroll_by_bounded(view_list, 0, -40, LV_ANIM_ON);
	}, LV_EVENT_CLICKED, nullptr);

	lv_obj_t *view_down_img = lv_img_create(view_down_btn);
	lv_obj_align(view_down_img, LV_ALIGN_CENTER, 0, 0);
	lv_img_set_src(view_down_img, LV_SYMBOL_DOWN);

	anim_label = lv_label_create(view_menu);
	lv_obj_add_flag(anim_label, LV_OBJ_FLAG_HIDDEN);

	// Battery indicator (bottom right)
	battery_icon = lv_img_create(view_menu);
	lv_img_set_src(battery_icon, LV_SYMBOL_BATTERY_FULL);
	lv_obj_align(battery_icon, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
	lv_obj_set_style_img_recolor(battery_icon, lv_color_hex(0x22c55e), 0);
	lv_obj_set_style_img_recolor_opa(battery_icon, LV_OPA_COVER, 0);

	battery_percent = lv_label_create(view_menu);
	lv_label_set_text(battery_percent, "100%");
	lv_obj_set_style_text_font(battery_percent, &lv_font_montserrat_10, 0);
	lv_obj_set_style_text_color(battery_percent, lv_color_hex(0x22c55e), 0);
	lv_obj_align_to(battery_percent, battery_icon, LV_ALIGN_OUT_LEFT_MID, -2, 0);

	battery_charging = lv_label_create(view_menu);
	lv_label_set_text(battery_charging, LV_SYMBOL_CHARGE);
	lv_obj_align_to(battery_charging, battery_icon, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_flag(battery_charging, LV_OBJ_FLAG_HIDDEN);  // Hidden by default

	// -------------------------- Alert Container -------------------------- //

	alert_cont = lv_obj_create(screen);
	lv_obj_set_size(alert_cont, 320, lv_pct(100));
	lv_obj_align(alert_cont, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_style(alert_cont, &style_transp, 0);
	lv_obj_clear_flag(alert_cont, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_set_flex_align(
	    alert_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START
	);
	lv_obj_set_flex_flow(alert_cont, LV_FLEX_FLOW_COLUMN);
}

void create_anims() {
	// ------------------------- Sidebar Animations ------------------------- //

	lv_anim_init(&anim_sidebar_open);
	lv_anim_set_var(&anim_sidebar_open, view_menu);
	lv_anim_set_time(&anim_sidebar_open, 200);
	lv_anim_set_exec_cb(&anim_sidebar_open, &anim_x_cb);

	anim_sidebar_close = anim_sidebar_open;

	lv_anim_set_path_cb(&anim_sidebar_open, &lv_anim_path_ease_out);
	lv_anim_set_values(&anim_sidebar_open, view_menu_width, 0);

	lv_anim_set_values(&anim_sidebar_close, 0, view_menu_width);
	lv_anim_set_deleted_cb(&anim_sidebar_close, &anim_del_cb);
	lv_anim_set_path_cb(&anim_sidebar_close, &lv_anim_path_ease_out);

	// -------------------------- Shade Animations -------------------------- //

	lv_anim_init(&anim_shade_hide);
	lv_anim_set_var(&anim_shade_hide, shade);
	lv_anim_set_time(&anim_shade_hide, 200);
	lv_anim_set_exec_cb(&anim_shade_hide, &anim_opa_cb);

	anim_shade_show = anim_shade_hide;

	lv_anim_set_values(&anim_shade_hide, 144, 0);
	lv_anim_set_deleted_cb(&anim_shade_hide, &anim_del_cb);
	lv_anim_set_values(&anim_shade_show, 0, 144);
}

// =============================== Initialize =============================== //

void battery_charging_update_task() {
	while (true) {
		if (battery_charging && battery_icon) {
			int32_t current = pros::battery::get_current();
			double capacity = pros::battery::get_capacity();
			
			if (current < 0) {
				lv_obj_clear_flag(battery_charging, LV_OBJ_FLAG_HIDDEN);
				lv_obj_align_to(battery_charging, battery_icon, LV_ALIGN_CENTER, 0, 0);
				
				// Set color: green when battery is white (30-90%), white when battery is colored
				lv_color_t charging_color;
				if (capacity >= 30.0 && capacity <= 90.0) {
					charging_color = lv_color_hex(0x22c55e);  // Green
				} else {
					charging_color = lv_color_hex(0xFFFFFF);  // White
				}
				lv_obj_set_style_text_color(battery_charging, charging_color, 0);
			} else {
				lv_obj_add_flag(battery_charging, LV_OBJ_FLAG_HIDDEN);
			}
		}
		pros::delay(100);  // Fast update for charging indicator
	}
}

void battery_update_task() {
	while (true) {
		update_battery_indicator();
		pros::delay(1000);  // Update every second
	}
}

bool initialized = false;

void initialize() {
	if (initialized) return;

	_init_fs();
	_init_styles();

	create_ui();
	create_anims();
	
	// Start battery update tasks
	pros::Task battery_task(battery_update_task);
	pros::Task charging_task(battery_charging_update_task);

	initialized = true;
}

// =============================== View Class =============================== //

rd_view_t *rd_view_create(const char *name) {
	initialize();

	rd_view_t *view = (rd_view_t *)malloc(sizeof(rd_view_t));

	view->obj = lv_obj_create(lv_scr_act());
	lv_obj_set_size(view->obj, lv_pct(100), lv_pct(100));
	lv_obj_add_flag(view->obj, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_style(view->obj, &style_bg, 0);
	lv_obj_set_parent(view->obj, view_cont);

	view->_list_btn = lv_list_add_btn(view_list, NULL, name);
	lv_obj_add_style(view->_list_btn, &style_core_list_btn, 0);
	lv_obj_add_style(view->_list_btn, &style_list_btn_pr, LV_STATE_PRESSED);
	lv_obj_set_user_data(view->_list_btn, view);
	lv_obj_add_event_cb(view->_list_btn, view_focus_cb, LV_EVENT_PRESSED, view);
	lv_obj_add_event_cb(view->_list_btn, close_cb, LV_EVENT_PRESSED, NULL);

	view->name = name;
	view->anims = RD_ANIM_ON;

	if (!current_view) rd_view_focus(view);

	return view;
}

void rd_view_del(rd_view_t *view) {
	if (!valid_view(view)) return;

	lv_obj_del(view->_list_btn);
	lv_obj_del(view->obj);
	if (current_view == view) current_view = NULL;

	free(view);
}

void rd_view_set_anims(rd_view_t *view, rd_anim_state_t state) { view->anims = state; }

rd_anim_state_t rd_view_get_anims(rd_view_t *view) { return view->anims; }

lv_obj_t *rd_view_obj(rd_view_t *view) {
	if (!valid_view(view)) return NULL;

	return view->obj;
}

void rd_view_focus(rd_view_t *view) {
	if (!valid_view(view)) return;

	if (current_view != NULL) lv_obj_add_flag(current_view->obj, LV_OBJ_FLAG_HIDDEN);
	current_view = view;
	lv_obj_clear_flag(current_view->obj, LV_OBJ_FLAG_HIDDEN);

	// Always keep anim_label hidden since we removed the text
	lv_obj_add_flag(anim_label, LV_OBJ_FLAG_HIDDEN);
}

void rd_view_alert(rd_view_t *view, const char *msg) {
	if (!valid_view(view)) return;

	if (!lv_obj_has_flag(view_menu, LV_OBJ_FLAG_HIDDEN)) {
		if (rd_view_get_anims(current_view) == RD_ANIM_ON)
			lv_anim_start(&anim_sidebar_close);
		else
			lv_obj_add_flag(view_menu, LV_OBJ_FLAG_HIDDEN);
	}

	if (lv_obj_has_flag(shade, LV_OBJ_FLAG_HIDDEN)) {
		lv_obj_clear_flag(shade, LV_OBJ_FLAG_HIDDEN);
		if (rd_view_get_anims(current_view) == RD_ANIM_ON) lv_anim_start(&anim_shade_show);
	}

	lv_obj_clear_flag(alert_cont, LV_OBJ_FLAG_HIDDEN);

	lv_obj_t *alert = lv_obj_create(alert_cont);
	lv_obj_set_width(alert, lv_pct(100));
	lv_obj_set_height(alert, LV_SIZE_CONTENT);
	lv_obj_add_event_cb(alert, alert_cb, LV_EVENT_CLICKED, view);
	lv_obj_add_style(alert, &style_alert, 0);

	lv_obj_t *origin_label = lv_label_create(alert);
	lv_obj_align(origin_label, LV_ALIGN_TOP_LEFT, 0, 0);
	lv_obj_add_style(origin_label, &style_text_small, 0);
	lv_label_set_text(origin_label, view->name);

	lv_obj_t *alert_msg = lv_label_create(alert);
	lv_obj_align(alert_msg, LV_ALIGN_TOP_LEFT, 0, 18);
	lv_obj_set_width(alert_msg, lv_pct(100));
	lv_obj_add_style(alert_msg, &style_text_medium, 0);
	lv_label_set_long_mode(alert_msg, LV_LABEL_LONG_WRAP);
	lv_label_set_text(alert_msg, msg);
}

rd_view_t *rd_view_get_current() {
	return current_view;
}