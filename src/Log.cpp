/*
 * Log.cpp
 *
 *  Created on: 27.07.2013
 *      Author: morpheby
 */

#include "platform.h"

#include "Log.h"

#include <iomanip>
#include <ctime>

namespace util {

Log::Log(const std::string& appName, LogSeverity minSeverity) :
		appName_(appName),
		minSev_(minSeverity) {
}

std::string Log::getSeverity(LogSeverity s) const {
	switch(s) {
	case LogSeverity::TRACE:
		return ".trace.";
	case LogSeverity::INFO:
		return "(info)";
	case LogSeverity::WARNING:
		return "(WARNING)";
	case LogSeverity::ERROR:
		return "[[ERROR]]";
	}
	return "";
}

Log::~Log() {
}

void Log::writeReal(LogSeverity severity, const std::string& str, std::ostream &stream) {
	if((int) severity < (int) minSev_)
		return;
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
//	logStream_ << "[" << std::put_time(&tm, "%c, %Z") << "] "
    char s[64];
    std::strftime(s, 64, "%c %Z", &tm);
    stream << "[" << s << "] " << appName_ << " "
			<< getSeverity(severity) << ": " << str << std::endl;
}

void Log::write(LogSeverity severity, const std::string& str) {
	writeImpl(severity, str);
}

LogFile::LogFile(const std::string& appName, LogSeverity minSeverity,
		const std::string& logFilePath) :
			Log(appName, minSeverity),
			logStream_(logFilePath, std::ios::out | std::ios::app) {
}

LogFile::~LogFile() {
}

void LogFile::writeImpl(LogSeverity severity, const std::string& str) {
	writeReal(severity, str, logStream_);
}

LogStream::LogStream(const std::string& appName, LogSeverity minSeverity,
		std::streambuf *stream) :
					Log(appName, minSeverity),
					logStream_(stream) {
}

LogStream::~LogStream() {
}

void LogStream::writeImpl(LogSeverity severity, const std::string& str) {
	writeReal(severity, str, logStream_);
}

} /* namespace util */
