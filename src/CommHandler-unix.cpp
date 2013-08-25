/*
 * CommHandler.cpp
 *
 *  Created on: 23.07.2013
 *      Author: morpheby
 */

#include "platform.h"

#include "CommHandler.h"
#include "Logger.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

namespace comm {
namespace internal {

CommHandler::CommHandler() : cfg_(std::make_shared<_CommCfg_t>()), commDes_(0), desSet_(false) {
	resetProcessingFlags();
}

CommHandler::~CommHandler() {
}

void CommHandler::setBaud(_CommSpeed_t baud) {
	util::Logger::getInstance()->log("Setting speed for port " + std::to_string(commDes_) +
			" to " + std::to_string(baud));
	if(cfsetispeed(getCommConfigPtr().get(), baud) < 0 ||
	   cfsetospeed(getCommConfigPtr().get(), baud) < 0) {
		throw util::posix_error_exception("setting port speed to" + std::to_string(baud),
				MAKE_DEBUG_STRING());
	}
	writeConfig();
}

_CommSpeed_t CommHandler::getBaud() const {
	return cfgetispeed(getCommConfigPtr().get());
}

void CommHandler::resetProcessingFlags() {
	util::Logger::getInstance()->log("Clearing configuration");
	cfmakeraw(getCommConfigPtr().get());
	getCommConfigPtr()->c_oflag = 0;
	writeConfig();
}

void CommHandler::setPlatformCompatibilityFlags() {
	/* Here I put all those flags that completely differ from platform to platform */
	getCommConfigPtr()->c_cc[VTIME] = 50; // 5 sec timeout
	getCommConfigPtr()->c_cc[VMIN] = 0; // no minimum
	getCommConfigPtr()->c_cflag |= CLOCAL;
	getCommConfigPtr()->c_iflag &= ~IGNBRK;
	writeConfig();
}

void CommHandler::set8BitComm() {
	getCommConfigPtr()->c_cflag &= ~CSIZE;
	getCommConfigPtr()->c_cflag |= CS8;
	writeConfig();
}

void CommHandler::setStopBitsCount(int count) {
	getCommConfigPtr()->c_cflag =
				!(count - 2)
				? (CSTOPB | getCommConfigPtr()->c_cflag)
				: ((~CSTOPB) & getCommConfigPtr()->c_cflag);
	writeConfig();
}

int CommHandler::getStopBitsCount() const {
	return 1 + !!(getCommConfigPtr()->c_cflag & CSTOPB);
}

void CommHandler::setReceiveEnable(bool enable) {
	getCommConfigPtr()->c_cflag =
			enable
			? (CREAD | getCommConfigPtr()->c_cflag)
			: ((~CREAD) & getCommConfigPtr()->c_cflag);
	writeConfig();
}

bool CommHandler::isReceiveEnabled() const {
	return getCommConfigPtr()->c_cflag & CREAD;
}

void CommHandler::setParityMode(ParityMode mode) {
	switch(mode) {
	case ParityMode::NONE:
		getCommConfigPtr()->c_cflag &= ~(PARENB | PARODD
#ifndef NO_SPACEMARK_PARITY
				| CMSPAR
#endif
				);
		break;
	case ParityMode::EVEN:
		getCommConfigPtr()->c_cflag = (getCommConfigPtr()->c_cflag & (~(PARODD
#ifndef NO_SPACEMARK_PARITY
				| CMSPAR
#endif
				))) | PARENB;
		break;
	case ParityMode::ODD:
		getCommConfigPtr()->c_cflag = (getCommConfigPtr()->c_cflag
#ifndef NO_SPACEMARK_PARITY
				& (~CMSPAR)
#endif
				) | PARODD | PARENB;
		break;
#ifndef NO_SPACEMARK_PARITY
	case ParityMode::MARK:
		getCommConfigPtr()->c_cflag |= PARODD | PARENB | CMSPAR;
		break;
	case ParityMode::SPACE:
		getCommConfigPtr()->c_cflag = (getCommConfigPtr()->c_cflag & (~PARODD)) | PARENB | CMSPAR;
		break;
#endif
	default:
		break;
	}
	writeConfig();
}

ParityMode CommHandler::getParityMode() const {
	auto cflag = getCommConfigPtr()->c_cflag;
	if(cflag & PARENB) {
#ifndef NO_SPACEMARK_PARITY
		if(cflag & CMSPAR) {
			if(cflag & PARODD)
				return ParityMode::MARK;
			else
				return ParityMode::SPACE;
		} else
#endif
		{
			if(cflag & PARODD)
				return ParityMode::ODD;
			else
				return ParityMode::EVEN;
		}
	}
	return ParityMode::NONE;
}

void CommHandler::setParityErrorIgnore(bool ignore) {
	getCommConfigPtr()->c_iflag =
			ignore
			? (IGNPAR | getCommConfigPtr()->c_iflag)
			: ((~IGNPAR) & getCommConfigPtr()->c_iflag);
	writeConfig();
}

void CommHandler::setParityErrorMark(bool mark) {
	getCommConfigPtr()->c_iflag =
			mark
			? (PARMRK | getCommConfigPtr()->c_iflag) // XXX
			: ((~PARMRK) & getCommConfigPtr()->c_iflag);
	writeConfig();
}

void CommHandler::setParityErrorCheckEnable(bool enable) {
	getCommConfigPtr()->c_iflag =
			enable
			? (INPCK | (getCommConfigPtr()->c_iflag))
			: ((~INPCK) & getCommConfigPtr()->c_iflag);
	writeConfig();
}

void CommHandler::setCommDescriptor(_CommHandle_t des) {
	commDes_ = des;
	desSet_ = true;
	readConfig();
}

void CommHandler::resetCommDescriptor() {
	desSet_ = false;
	commDes_ = 0;
}

_CommHandle_t CommHandler::getCommDescriptor() const {
	return commDes_;
}

void CommHandler::readConfig() {
	if(isCommDescriptorSet()) {
		util::Logger::getInstance()->log("Reading configuration for port " + std::to_string(commDes_));
		if(tcgetattr((int) getCommDescriptor(), getCommConfigPtr().get()) < 0)
			throw util::posix_error_exception("reading configuration from port", MAKE_DEBUG_STRING());
	}
}

void CommHandler::writeConfig() {
	if(isCommDescriptorSet()) {
		util::Logger::getInstance()->log("Writing configuration for port " + std::to_string(commDes_));
		if(tcsetattr((int) getCommDescriptor(), TCSAFLUSH, getCommConfigPtr().get()) < 0)
			throw util::posix_error_exception("writing configuration to port", MAKE_DEBUG_STRING());
	}
}

std::shared_ptr<_CommCfg_t> CommHandler::getCommConfigPtr() {
	return cfg_;
}

std::shared_ptr<const _CommCfg_t> CommHandler::getCommConfigPtr() const {
	return cfg_;
}

void CommHandler::setMapBreakToInterrupt(bool map) {
	getCommConfigPtr()->c_iflag =
			map
			? (BRKINT | getCommConfigPtr()->c_iflag)
			: ((~BRKINT) & getCommConfigPtr()->c_iflag);
	writeConfig();
}

bool CommHandler::isCommDescriptorSet() const {
	return desSet_;
}

_CommHandle_t port_open(const std::string& port) {
	int commDes = ::open(port.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
	if(commDes == -1)
		throw util::posix_error_exception("opening port", MAKE_DEBUG_STRING());
	if(!::isatty(commDes))
		throw util::info_exception(port + " is not a tty", MAKE_DEBUG_STRING());
	return commDes;
}

int port_close(_CommHandle_t port) {
	return ::close(port);
}

ssize_t port_read(_CommHandle_t port, void * buffer, size_t sz) {
	return ::read(port, buffer, sz);
}

ssize_t port_write(_CommHandle_t port, const void * buffer, size_t sz) {
	return ::write(port, buffer, sz);
}

ssize_t port_get_input_queue_size(_CommHandle_t port) {
	int count;
	::ioctl(port, FIONREAD, &count);
	return count;
}

} /* namespace internal */
} /* namespace comm */
