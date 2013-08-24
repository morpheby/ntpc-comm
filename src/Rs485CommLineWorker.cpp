/*
 * Rs485CommLineWorker.cpp
 *
 *  Created on: 23.08.2013
 *      Author: morpheby
 */

#include "platform.h"

#include "Rs485CommLineWorker.h"

#include "Logger.h"

namespace rs485 {

void Rs485CommLineWorker::setReceiveActive(bool receiveActive) {
	std::lock_guard<std::mutex> lock(receiveFlagsMutex_);
	receiveActive_ = receiveActive;
}

Rs485CommLineWorker::Rs485CommLineWorker(_MachineAddress_t localAddress,
		const std::unique_ptr<comm::SerialComm>& connection) :
			localAddress_(localAddress),
			receiveActive_(false),
			promiscuousMode_(false),
			connection_(connection) {
	util::Logger::getInstance()->log("Starting RS-485 communication...");
	worker_ = std::thread(&Rs485CommLineWorker::rs485Worker, this);
}

Rs485CommLineWorker::~Rs485CommLineWorker() {
	util::Logger::getInstance()->log("Shutting down RS-485 communication...");

	// Connection has to be terminated before
}

_MachineAddress_t Rs485CommLineWorker::getLocalAddress() const {
	return localAddress_;
}

const std::unique_ptr<comm::SerialComm>& Rs485CommLineWorker::getConnection() const {
	return connection_;
}

bool Rs485CommLineWorker::isPromiscuousMode() const {
	std::lock_guard<std::mutex> lock(receiveFlagsMutex_);
	return promiscuousMode_;
}

void Rs485CommLineWorker::setPromiscuousMode(bool promiscuousMode) {
	std::lock_guard<std::mutex> lock(receiveFlagsMutex_);
	promiscuousMode_ = promiscuousMode;
}

void Rs485CommLineWorker::rs485Worker() {
	std::vector<uint16_t> tmpBuffer(1);
	bool addressedOnce = false;
	while(!isExiting()) {
		connection_->read(tmpBuffer.begin(), tmpBuffer.end());
		if(tmpBuffer[0] & (1 << 8)) {
			// We received connection marker
			if(!isPromiscuousMode()) {
				// We don't receive everything
				if(tmpBuffer[0] & (0x100 - 1) == getLocalAddress()) {
					// It seems like it is we being addressed
					setReceiveActive(addressedOnce);
					addressedOnce = !addressedOnce;
				} else
					setReceiveActive(addressedOnce = false);
			}
		} else {
			if(addressedOnce)
				addressedOnce = false;
			if(isReceiveActive() || isPromiscuousMode()) {
				// Put data in the queue
				std::lock_guard<std::mutex> lock(receiveDataMutex_);
				dataStream_.push(tmpBuffer[0] & (0x100 - 1));
			}
		}
	}
}

bool Rs485CommLineWorker::isReceiveActive() const {
	std::lock_guard<std::mutex> lock(receiveFlagsMutex_);
	return receiveActive_;
}

bool Rs485CommLineWorker::isExiting() const {
	std::lock_guard<std::mutex> lock(receiveFlagsMutex_);
	return exiting_;
}

void Rs485CommLineWorker::setExiting(bool exiting) {
	std::lock_guard<std::mutex> lock(receiveFlagsMutex_);
	exiting_ = exiting;
}

} /* namespace rs485 */
