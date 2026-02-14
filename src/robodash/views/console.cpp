#include "robodash/views/console.hpp"
#include "robodash/apix.h"

// ============================= Core Functions ============================= //

rd::Console::Console(std::string name) {
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

void rd::Console::focus() { rd_view_focus(this->view); }
