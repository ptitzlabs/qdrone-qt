#ifndef _CONSOLE_COLOR_H_
#define _CONSOLE_COLOR_H_
#pragma once
//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BLACKBG   "\033[40m"      /* Black */
#define REDBG     "\033[41m"      /* Red */
#define GREENBG   "\033[42m"      /* Green */
#define YELLOWBG  "\033[43m"      /* Yellow */
#define BLUEBG    "\033[44m"      /* Blue */
#define MAGENTABG "\033[45m"      /* Magenta */
#define CYANBG    "\033[46m"      /* Cyan */
#define WHITEBG   "\033[47m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
#define CLEARLINE "\x1b[K"

#define BLACKONWHITE "\033[40m\033[37m"
#endif
