/*
 * Logger.h
 *
 *  Created on: 27.07.2013
 *      Author: morpheby
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "Log.h"
#include <memory>
#include <exception>
#include <string>
#include <cerrno>
#include <vector>

namespace util {

#ifdef DEBUG
std::string MakeDebugString(const std::string &file, int line, const std::string &funct);
#endif

class posix_error_exception {
	int error_;
	std::string happenedWhile_;
public:
	posix_error_exception(const std::string& happenedWhile);

	int getErrno() const;
	std::string getWhile() const;
};

class Logger : public std::enable_shared_from_this<Logger> {
	std::vector<std::shared_ptr<Log>> logs_;

	static Logger *instance_;

	std::shared_ptr<Logger> getShared();
	void write(LogSeverity severity, const std::string& info);

public:
	Logger();
	virtual ~Logger();

	void addLog(const std::shared_ptr<Log> &log);

	void logException(const std::exception &exc);
	void logException(const std::string &exc);
	void logPosixError(int err, const std::string &comment);
	static std::string getPosixErrorDescription(int err);
	void logWarning(const std::string &warning);
	void log(const std::string &info);
	void trace(const std::string &t);

	static std::shared_ptr<Logger> getInstance();
};

} /* namespace util */

#ifdef DEBUG
#define MAKE_DEBUG_STRING()		\
	util::MakeDebugString(__FILE__, __LINE__, __func__)
#else
#define MAKE_DEBUG_STRING() ""
#endif

#endif /* LOGGER_H_ */
