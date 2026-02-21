#include "robodash/views/console.hpp"
#include "robodash/apix.h"
#include "pros/misc.hpp"

// ============================= Core Functions ============================= //

rd::Console::Console(std::string name, pros::Controller* controller) 
	: controller(controller), scroll_position(0) {
	this->view = rd_view_create(name.c_str());

	lv_obj_set_style_bg_color(view->obj, color_bg, 0);

	this->output_cont = lv_obj_create(view->obj);
	lv_obj_set_width(output_cont, lv_pct(100));
	lv_obj_set_height(output_cont, lv_pct(100));
	lv_obj_align(output_cont, LV_ALIGN_CENTER, 0, 0);
	lv_obj_add_style(output_cont, &style_transp, 0);

	this->output = lv_label_create(output_cont);
	lv_obj_set_height(output, LV_SIZE_CONTENT);
	lv_obj_add_style(output, &style_transp, 0);
	lv_obj_add_style(output, &style_text_mono, 0);
	lv_label_set_recolor(output, true);
	lv_label_set_long_mode(output, LV_LABEL_LONG_WRAP);
}

// =========================== Console Functions =========================== //

void rd::Console::clear() {
	lv_label_set_text(this->output, "");
	this->stream.str("");
	this->stream.clear();
	this->lines.clear();
}

void rd::Console::print(std::string str) {
	this->stream << str;
	if (this->output) lv_label_set_text(this->output, this->stream.str().c_str());
	lv_obj_scroll_to_y(this->output_cont, LV_COORD_MAX, LV_ANIM_OFF);
}

void rd::Console::println(std::string str) { this->print(str + "\n"); }

void rd::Console::update_line(int line_num, std::string str) {
	// Expand lines vector if needed
	while (lines.size() <= (size_t)line_num) {
		lines.push_back("");
	}
	
	// Update the specific line
	lines[line_num] = str;
	
	// Rebuild the full text
	this->stream.str("");
	this->stream.clear();
	for (size_t i = 0; i < lines.size(); i++) {
		this->stream << lines[i];
		if (i < lines.size() - 1) this->stream << "\n";
	}
	
	if (this->output) lv_label_set_text(this->output, this->stream.str().c_str());
}

void rd::Console::update() {
	if (controller == nullptr) return;
	
	// Display ASCII smiley face on controller LCD
	static bool was_active = false;
	bool is_active = (rd_view_get_current() == this->view);
	
	if (!is_active) {
		was_active = false;
		return;
	}
	
	// Handle scrolling (only when view is active)
	if (controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
		if (scroll_position > 0) scroll_position--;
	}
	if (controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		if (scroll_position < (int)lines.size() - 3) scroll_position++;
	}
	
	// Clear and draw smiley when view becomes active
	if (!was_active) {
		controller->clear();
		pros::delay(50);
		
		// Line 0: Eyes (18 char max)
		controller->set_text(0, 0, "   X             X  ");
		pros::delay(50);
		
		// Line 1: Smile curve
		controller->set_text(1, 0, "");
		pros::delay(50);
		
		// Line 2: Smile bottom
		controller->set_text(2, 0, "      ######    ");
		pros::delay(50);
		
		was_active = true;
	}
}

void rd::Console::focus() { rd_view_focus(this->view); }
