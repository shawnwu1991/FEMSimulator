#pragma once

#ifndef _Std_String_h_
#define _Std_String_h_

#include <string>
#include <sstream>
#include <fstream>
#include <tchar.h>
#include <iomanip>

typedef std::basic_string<TCHAR> tstring;
typedef std::basic_ostream<TCHAR> tostream;
typedef std::basic_istream<TCHAR> tistream;
typedef std::basic_fstream<TCHAR> tfstream;
typedef std::basic_ifstream<TCHAR> tifstream;
typedef std::basic_ofstream<TCHAR> tofstream;
typedef std::basic_stringstream<TCHAR> tstringstream;
typedef std::basic_istringstream<TCHAR> tistringstream;
typedef std::basic_ostringstream<TCHAR> tostringstream;

#if UNICODE
#define tcout wcout
#define tcin wcin
#define toStdTString toStdWString
#else
#define tcout cout
#define tcin wcin
#define toStdTString toStdString
#endif

#endif