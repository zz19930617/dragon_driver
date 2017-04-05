/*
 * utils.h
 *
 *  Created on: Jan 16, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_UTIL_LOG_H_
#define INCLUDE_MIDDLEWARE_UTIL_LOG_H_

#include <glog/logging.h>
#include <glog/log_severity.h>

namespace middleware {

#define LOG_INFO      LOG(INFO)     << "\t"
#define LOG_WARNING   LOG(WARNING)  << "\t"
#define LOG_ERROR     LOG(ERROR)    << "\t"
#define LOG_FATAL     LOG(FATAL)    << "\t"

} /* namespace middleware */



#endif /* INCLUDE_MIDDLEWARE_UTIL_LOG_H_ */
