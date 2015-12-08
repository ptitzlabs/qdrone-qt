#include "message_handler.hpp"

msg_field::msg_field()
    : name(""),
      value(""),
      data_f(0),
      data_i(0),
      status(0),
      n_status(0),
      type(0) {}

msg_field::~msg_field() {}

status_toolbar::status_toolbar() : n_rows(4), n_cols(16) {
    line_name = new std::string[n_rows];
    msg = new msg_field* [n_rows];
    for (int i = 0; i < n_rows; i++) {
        std::cout << std::endl;
    }
    for (int i = 0; i < n_rows; i++) {
        std::cout << "\x1b[A";
    }
    for (int i = 0; i < n_rows; i++) {
        msg[i] = new msg_field;
        line_name[i] = "line " + i;
    }
#ifdef TIOCGSIZE
    struct ttysize ts;
    ioctl(STDIN_FILENO, TIOCGSIZE, &ts);
    cols = ts.ts_cols;
    lines = ts.ts_lines;
#elif defined(TIOCGWINSZ)
    struct winsize ts;
    ioctl(STDIN_FILENO, TIOCGWINSZ, &ts);
    cols = ts.ws_col;
    lines = ts.ws_row;
#endif /* TIOCGSIZE */
    for (int i = 0; i < lines; i++) std::cout << std::endl;

    msg_char_length = cols / (n_cols + 1);
}

status_toolbar::~status_toolbar() {
    delete[] line_name;
    for (int i = 0; i < n_rows; i++) {
        delete msg[i];
    }
    delete[] msg;
}

void status_toolbar::add(std::string name, int row, int col, int* value) {
    msg[row][col].name = name;
    msg[row][col].data_i = value;
    msg[row][col].type = 1;
    msg[row][col].value = *msg[row][col].data_i;
}
void status_toolbar::add(std::string name, int row, int col, double* value) {
    msg[row][col].name = name;
    msg[row][col].data_f = value;
    msg[row][col].type = 2;
    msg[row][col].value = *msg[row][col].data_f;
    std::cout << msg[row][col].name << " " << *msg[row][col].data_f << " "
              << msg[row][col].type;
}
void status_toolbar::add_name(std::string name, int row) {
    line_name[row] = name;
}

void status_toolbar::print_out() {
    std::cout << "\x1b[H";
    for (int i = 0; i < n_rows; i++) {
        std::cout << "\x1b["<<i+2<<"H"<<CLEARLINE << BOLDGREEN << line_name[i] << " " << RESET;
        for (int j = 0; j < n_cols; j++) {
            // std::cout << msg[i][j].name << ": ";
            switch (msg[i][j].type) {
                case 0:
                    break;
                case 1:
                    std::cout << msg[i][j].name << ": ";
                    std::cout << *msg[i][j].data_i;
                    break;
                case 2:
                    std::cout << "\x1b[" << i +2<< ";" << (j+1)* msg_char_length
                              << "H" << msg[i][j].name << ": "
                              << *msg[i][j].data_f;
                    break;
                case 3:
                    break;
            }
        }
        std::cout << std::endl;
    }
    std::cout << "\x1b[" << lines << ";0H";
    // std::cout<<"ok";
}

namespace msg {
std::string sticky;
int sticky_w;
int sticky_h;
status_toolbar info;
}

void msg::sticky_refresh(std::string str) {
    sticky = str;
    clear_line();
    std::cout << "\x1b[A";
    print_sticky();
}

void msg::init() {
    // getmaxyx(stdscr,y_dim,x_dim);
}
void msg::clear_line() {
    std::cout << "\x1b[A";
    for (int i = 0; i < 100; i++) std::cout << " ";
    std::cout << "\n";
}

void msg::alert(std::string str) {
    std::cout << RED << "Alert: " << RESET << str << std::endl;
}
void msg::status(std::string str) {
    clear_line();
    std::cout << GREEN << "\x1b[AStatus: " << RESET << str << std::endl;
    print_sticky();
}
void msg::status2(std::string str) {
    clear_line();
    std::cout << YELLOW << "\x1b[AStatus: " << RESET << str << std::endl;
    print_sticky();
}
void msg::text(const char* str) {
    clear_line();
    std::cout << "\x1b[A" << str << std::endl;
    print_sticky();
}
void msg::begin_text() {
    clear_line();
    std::cout << "\x1b[A";
}
void msg::end_text() { print_sticky(); }
void msg::print_sticky() { std::cout << sticky << std::endl; }

void msg::panel_refresh_thread(status_toolbar * stat, int refresh_freq){
    stat->print_out();
    usleep(refresh_freq*1000000);
}
