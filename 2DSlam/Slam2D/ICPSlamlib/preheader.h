#ifndef PREHEADER_H
#define PREHEADER_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <assert.h>

using namespace std;

/** \def THROW_EXCEPTION(msg)
* \param msg This can be a char*, a std::string, or a literal string.
* Defines a unified way of reporting exceptions
* \sa MRPT_TRY_START, MRPT_TRY_END, THROW_EXCEPTION_CUSTOM_MSG1
*/
#define THROW_EXCEPTION(msg)	\
{\
	std::ostringstream auxCompStr;\
	auxCompStr << "\n\n =============== MRPT EXCEPTION =============\n";\
	auxCompStr << ", line " << __LINE__ << ":\n";\
	auxCompStr << msg << std::endl; \
	throw std::logic_error( auxCompStr.str() );\
}\


#endif