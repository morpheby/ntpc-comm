/*
 * Logger.cpp
 *
 *  Created on: 27.07.2013
 *      Author: morpheby
 */

#include "platform.h"

#include "Logger.h"


namespace util {

Logger * Logger::instance_;

info_exception::info_exception(const std::string &happenedWhile, const std::string &happenedIn) :
		happenedWhile_(happenedWhile),
		happenedIn_(happenedIn) {
}

std::string info_exception::getWhile() const {
	return happenedWhile_ + happenedIn_;
}

const char * info_exception::what() const noexcept {
	return ("Exception occured while " + getWhile()).c_str();
}

posix_error_exception::posix_error_exception(const std::string& happenedWhile,
			const std::string& happenedIn) :
		info_exception(happenedWhile, happenedIn),
		error_(errno) {
}

int posix_error_exception::getErrno() const {
	return error_;
}

const char * posix_error_exception::what() const noexcept {
	return ("Error occured - " + Logger::getPosixErrorDescription(getErrno()) +
			" - while " + getWhile()).c_str();
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
	return " // from " + file + ":" + std::to_string(line) + ": " + funct;
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

