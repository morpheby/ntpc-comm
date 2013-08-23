/*
 * CommHandler.cpp
 *
 *  Created on: 23.07.2013
 *      Author: morpheby
 */

#include "CommHandler.h"
#include "Logger.h"

namespace comm {
namespace internal {

CommHandler::CommHandler() : cfg_(std::make_shared<termios>()), commDes_(0), desSet_(false) {
	resetProcessingFlags();
}

CommHandler::~CommHandler() {
}

void CommHandler::setBaud(_CommSpeed_t baud) {
	util::Logger::getInstance()->log("Setting speed for port " + std::to_string(commDes_) +
			" to " + std::to_string(baud));
	if(cfsetispeed(getCommConfigPtr().get(), baud) < 0 ||
	   cfsetospeed(getCommConfigPtr().get(), baud) < 0) {
		throw util::posix_error_exception("setting port speed to" + std::to_string(baud) + MAKE_DEBUG_STRING());
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
//	getCommConfigPtr()->c_cflag &= ~CLOCAL;
	getCommConfigPtr()->c_iflag &= ~IGNBRK;
//	getCommConfigPtr()->c_lflag |= ICANON;
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

void CommHandler::setRecieveEnable(bool enable) {
	getCommConfigPtr()->c_cflag =
			enable
			? (CREAD | getCommConfigPtr()->c_cflag)
			: ((~CREAD) & getCommConfigPtr()->c_cflag);
	writeConfig();
}

bool CommHandler::isRecieveEnabled() const {
	return getCommConfigPtr()->c_cflag & CREAD;
}

void CommHandler::setParityMode(ParityMode mode) {
	switch(mode) {
	case ParityMode::NONE:
		getCommConfigPtr()->c_cflag &= ~(PARENB | PARODD | CMSPAR);
		break;
	case ParityMode::EVEN:
		getCommConfigPtr()->c_cflag = (getCommConfigPtr()->c_cflag & (~(PARODD | CMSPAR))) | PARENB;
		break;
	case ParityMode::ODD:
		getCommConfigPtr()->c_cflag = (getCommConfigPtr()->c_cflag & (~CMSPAR)) | PARODD | PARENB;
		break;
	case ParityMode::MARK:
		getCommConfigPtr()->c_cflag |= PARODD | PARENB | CMSPAR;
		break;
	case ParityMode::SPACE:
		getCommConfigPtr()->c_cflag = (getCommConfigPtr()->c_cflag & (~PARODD)) | PARENB | CMSPAR;
		break;
	default:
		break;
	}
	writeConfig();
}

ParityMode CommHandler::getParityMode() const {
	auto cflag = getCommConfigPtr()->c_cflag;
	if(cflag & PARENB) {
		if(cflag & CMSPAR) {
			if(cflag & PARODD)
				return ParityMode::MARK;
			else
				return ParityMode::SPACE;
		} else {
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
			throw util::posix_error_exception("reading configuration from port" + MAKE_DEBUG_STRING());
	}
}

void CommHandler::writeConfig() {
	if(isCommDescriptorSet()) {
		util::Logger::getInstance()->log("Writing configuration for port " + std::to_string(commDes_));
		if(tcsetattr((int) getCommDescriptor(), TCSAFLUSH, getCommConfigPtr().get()) < 0)
			throw util::posix_error_exception("writing configuration to port" + MAKE_DEBUG_STRING());
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

} /* namespace internal */
} /* namespace comm */
