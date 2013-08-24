/*
 * main.cpp
 *
 *  Created on: 23.07.2013
 *      Author: morpheby
 */

#include "platform.h"

#include "SerialComm.h"

#include <iostream>
#include <algorithm>
#include <iterator>
#include <memory>

#include "Log.h"
#include "Logger.h"

void logging_terminate_handler() {
	util::Logger::getInstance()->logException("Terminate called on active thread. Aborting...");
	std::abort();
}

int main(int argc, char **argv) {
	if(argc != 2) {
		std::cout << "Usage:" << std::endl
				<< argv[0] << " {port name}" << std::endl;
		return -1;
	}
	std::string port = argv[1];

	auto logger = std::make_shared<util::Logger>();
	logger->addLog(std::make_shared<util::LogFile>("ntpc-comm", util::LogSeverity::TRACE, "ntpc-comm.log"));
	logger->addLog(std::make_shared<util::LogStream>("ntpc-comm", util::LogSeverity::ERROR, std::cerr.rdbuf()));

	try {
		comm::SerialComm serial(port);

		std::vector<uint16_t> rdbuf(20);

		serial.read(rdbuf.begin(), rdbuf.end());

	} catch(const util::posix_error_exception &e) {
		logger->logPosixError(e.getErrno(), e.getWhile());
	} catch(const std::exception &e) {
		logger->logException(e);
	} catch(const char *str) {
		logger->logException(str);
	} catch(const std::string &s) {
		logger->logException(s);
	}

	return 0;
}


