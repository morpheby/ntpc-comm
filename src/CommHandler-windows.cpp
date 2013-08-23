/*
 * CommHandler.cpp
 *
 *  Created on: 23.08.2013
 *      Author: morpheby
 */

#include "platform.h"

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
}

_CommSpeed_t CommHandler::getBaud() const {
}

void CommHandler::resetProcessingFlags() {
}

void CommHandler::setPlatformCompatibilityFlags() {
}

void CommHandler::set8BitComm() {
}

void CommHandler::setStopBitsCount(int count) {
}

int CommHandler::getStopBitsCount() const {
}

void CommHandler::setRecieveEnable(bool enable) {
}

bool CommHandler::isRecieveEnabled() const {
}

void CommHandler::setParityMode(ParityMode mode) {
}

ParityMode CommHandler::getParityMode() const {
}

void CommHandler::setParityErrorIgnore(bool ignore) {
}

void CommHandler::setParityErrorMark(bool mark) {
}

void CommHandler::setParityErrorCheckEnable(bool enable) {
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
	}
}

void CommHandler::writeConfig() {
	if(isCommDescriptorSet()) {
	}
}

std::shared_ptr<_CommCfg_t> CommHandler::getCommConfigPtr() {
	return cfg_;
}

std::shared_ptr<const _CommCfg_t> CommHandler::getCommConfigPtr() const {
	return cfg_;
}

void CommHandler::setMapBreakToInterrupt(bool map) {
}

bool CommHandler::isCommDescriptorSet() const {
	return desSet_;
}

} /* namespace internal */
} /* namespace comm */
