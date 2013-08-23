/*
 * SerialComm_test.cpp
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

int main(int argc, char **argv) {
	std::string port;

	std::cout << "Utility reads raw data from serial 9-bit port. To start, "
			  << "enter tty device path: " << std::endl;
	std::getline(std::cin, port);
	std::cout << "Send some data over serial port (like ping request) from the device";

	auto logger = std::make_shared<util::Logger>();
	logger->addLog(std::make_shared<util::LogFile>("ntpc-comm", util::LogSeverity::TRACE, "ntpc-comm.log"));
	logger->addLog(std::make_shared<util::LogStream>("ntpc-comm", util::LogSeverity::ERROR, std::cerr.rdbuf()));

	try {
		comm::SerialComm serial(port);

		std::vector<uint16_t> rdbuf(20);

		serial.read(rdbuf.begin(), rdbuf.end());

		std::cout << std::hex;
		std::copy(rdbuf.begin(), rdbuf.end(), std::ostream_iterator<uint16_t>(std::cout, " "));
		std::cout << std::endl;
	} catch(const util::posix_error_exception &e) {
		logger->logPosixError(e.getErrno(), e.getWhile());
	} catch(const std::exception &e) {
		logger->logException(e);
	} catch(const char *str) {
		logger->logException(str);
	} catch(const std::string &s) {
		logger->logException(s);
	}

	std::cout << "Please, check the data. If it is correct, press enter; "
			  << "otherwise press Ctrl+C" << std::endl;
	std::getline(std::cin, port);

	return 0;
}
