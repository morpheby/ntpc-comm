/*
 * Log.h
 *
 *  Created on: 27.07.2013
 *      Author: morpheby
 */

#ifndef LOG_H_
#define LOG_H_

#include <fstream>
#include <iostream>
#include <errno.h>
#include <string.h>
#include <string>
#include <memory>

namespace util {

enum class LogSeverity : int {
	TRACE,
	INFO,
	WARNING,
	ERROR,
};

class Log {
	std::string appName_;
	LogSeverity minSev_;

protected:
	std::string getSeverity(LogSeverity s) const;
	void writeReal(LogSeverity severity, const std::string &str, std::ostream &stream);
	virtual void writeImpl(LogSeverity severity, const std::string &str) = 0;

	Log(const std::string &appName, LogSeverity minSeverity);
public:
	virtual ~Log();
	void write(LogSeverity severity, const std::string &str);
};

class LogFile : public Log {
	std::ofstream logStream_;
protected:
	void writeImpl(LogSeverity severity, const std::string &str);
public:
	LogFile(const std::string &appName, LogSeverity minSeverity, const std::string &logFilePath);
	virtual ~LogFile();
};

class LogStream : public Log {
	std::ostream logStream_;
protected:
	void writeImpl(LogSeverity severity, const std::string &str);
public:
	LogStream(const std::string &appName, LogSeverity minSeverity, std::streambuf *stream);
	virtual ~LogStream();

};

} /* namespace util */
#endif /* LOG_H_ */
