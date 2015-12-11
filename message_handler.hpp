#ifndef _MESSAGE_HANDLER_HPP_
#define _MESSAGE_HANDLER_HPP_

#include <iostream>
#include <string>
#include "console_color.h"
//#include <ncurses.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>

struct msg_field {
    msg_field();
    ~msg_field();
    std::string name;
    std::string value;
    double* data_f;
    int* data_i;
    std::string* status;
    int n_status;
    int type;  // 0 for empty, 1 for integer, 2 for double, 3 for status
};

class status_toolbar {
   public:
    status_toolbar();
    ~status_toolbar();
    void add(std::string name, int row, int col, int* value);
    void add(std::string name, int row, int col, double* value);
    void add_name(std::string name, int row);
    std::string fetch();
    void print_out();

   private:
    std::string* line_name;
    msg_field** msg;
    int n_rows;
    int n_cols;
    int msg_char_length;

    int cols;
    int lines;
};

namespace msg {
void init();

// const char * sticky;
void clear_line();
void print_sticky();
void begin_text();
void end_text();
void text(const char* str);
// void update_sticky();
void alert(std::string str);
void status(std::string str);
void status2(std::string str);

void set_sticky_width(int w);
void set_sticky_height(int h);
void sticky_refresh();
void sticky_refresh(std::string str);


void panel_refresh_thread(status_toolbar * stat, int refresh_freq);
}

#endif
