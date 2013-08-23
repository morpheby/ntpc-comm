/*
 * Logger.cpp
 *
 *  Created on: 27.07.2013
 *      Author: morpheby
 */

#include "Logger.h"


namespace util {

Logger * Logger::instance_;

posix_error_exception::posix_error_exception(const std::string& happenedWhile) :
		error_(errno), happenedWhile_(happenedWhile) {
}

int posix_error_exception::getErrno() const {
	return error_;
}

std::string posix_error_exception::getWhile() const {
	return happenedWhile_;
}



Logger::Logger() {
	instance_ = this;
}

Logger::~Logger() {
}

void Logger::logException(const std::exception& exc) {
	write(LogSeverity::ERROR, std::string("Exception occured: ") + exc.what());
}

void Logger::logPosixError(int err, const std::string& comment) {
	write(LogSeverity::ERROR, "Error occured - " +
			getPosixErrorDescription(err) + " - while " + comment);
}

#ifdef DEBUG
std::string MakeDebugString(const std::string &file, int line, const std::string &funct) {
	return " // " + file + ":" + std::to_string(line) + ": " + funct;
}
#endif

std::string Logger::getPosixErrorDescription(int err) {
	return strerror(err);
}

std::shared_ptr<Logger> Logger::getShared() {
	return shared_from_this();
}

void Logger::logWarning(const std::string& warning) {
	write(LogSeverity::WARNING, warning);
}

void Logger::logException(const std::string& exc) {
	write(LogSeverity::ERROR, std::string("Exception occured: ") + exc);
}

void Logger::write(LogSeverity severity, const std::string& info) {
	for(auto log : logs_)
		log->write(severity, info);
}

void Logger::log(const std::string& info) {
	write(LogSeverity::INFO, info);
}

void Logger::trace(const std::string& t) {
	write(LogSeverity::TRACE, t);
}

void Logger::addLog(const std::shared_ptr<Log>& log) {
	logs_.push_back(log);
	log->write(LogSeverity::INFO, "Log started");
}

std::shared_ptr<Logger> Logger::getInstance() {
	return instance_->getShared();
}

} /* namespace util */

