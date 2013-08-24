/*
 * SerialComm.cpp
 *
 *  Created on: 23.07.2013
 *      Author: morpheby
 */

#include "platform.h"

#include "SerialComm.h"
#include "CommHandler.h"
#include <assert.h>
#include <chrono>
#include <functional>
#include "Logger.h"

namespace comm {


SerialComm::SerialComm(const std::string &connStr) :
	connStr_(connStr), exiting_(false), commPort_(std::make_shared<internal::CommHandler>()),
	receiveWorker_() {
	util::Logger::getInstance()->log("Opening port " + connStr);
	internal::_CommHandle_t commDes = internal::port_open(connStr);
	getCommHandler()->setCommDescriptor(commDes);
	resetCommConfig();
	receiveWorker_ = std::thread(&SerialComm::dataReader, this);
}

std::shared_ptr<internal::CommHandler> SerialComm::getCommHandler() const {
	return commPort_;
}

void SerialComm::resetCommConfig() {
	std::lock_guard<std::mutex> lock(portConfigMutex_);

	auto handler = getCommHandler();
	handler->resetProcessingFlags();
	handler->setBaud(9600);
	handler->set8BitComm();
	handler->setStopBitsCount(2);
	handler->setReceiveEnable(true);
	handler->setPlatformCompatibilityFlags();
	handler->setMapBreakToInterrupt(true);
	handler->setParityErrorCheckEnable(true);
	handler->setParityErrorMark(true);
	handler->setParityErrorIgnore(false);
#ifdef NO_SPACEMARK_PARITY
	handler->setParityMode(internal::ParityMode::EVEN);
#else
	handler->setParityMode(internal::ParityMode::SPACE);
#endif
}

uint16_t SerialComm::read9BitByte() {
	std::unique_lock<std::mutex> lock(recvMutex_);

	while(receiveBuffer_.empty())
		receiveDataReady_.wait(lock);

	uint16_t byte = receiveBuffer_.front();
	receiveBuffer_.pop();
	return byte;
}

int SerialComm::processRawDataStream() {
	auto parMode  = getCommHandler()->getParityMode();

	uint8_t rcv[3];
	int sz = 0;
	uint16_t byte = 0;

	{
		std::lock_guard<std::mutex> lock(recvMutex_);

		readNoLock(rcv, 1);
		++sz;
		if(rcv[0] == 0xFF) { // parity error marker start
			readNoLock(rcv+1, 1);
			++sz;
			if(rcv[1] == 0x00) { // parity error occured
				readNoLock(rcv+2, 1);
				++sz;
				byte = processParityBit(rcv[2], true, parMode);
				util::Logger::getInstance()->trace("Parity error" + MAKE_DEBUG_STRING());
			} else if(rcv[1] == 0xFF)
				byte = processParityBit(rcv[1], false, parMode);
			else
				assert(rcv[0] == 0xFF && (rcv[1] == 0x00 || rcv[1] == 0xFF));
		} else {
			byte = processParityBit(rcv[0], false, parMode);
		}
		receiveBuffer_.push(byte);
	}

	util::Logger::getInstance()->trace("Received byte " + std::to_string(byte) + MAKE_DEBUG_STRING());

	for(int i = 0; i < sz; ++i)
		receiveDataReady_.notify_one();

	return sz;
}

uint16_t SerialComm::processParityBit(char received, bool isParityError,
		internal::ParityMode parMode) {
	uint16_t rcv = (uint8_t) received;
	switch(parMode) {
#ifndef NO_SPACEMARK_PARITY
	case internal::ParityMode::SPACE:
		if(isParityError)
			rcv |= (1<<8);
		break;
	case internal::ParityMode::MARK:
		if(!isParityError)
			rcv |= (1<<8);
		break;
#endif
	case internal::ParityMode::ODD:
		if(getOddParity(received) == !isParityError)
			rcv |= (1<<8);
		break;
	case internal::ParityMode::EVEN:
		if(getEvenParity(received) == !isParityError)
			rcv |= (1<<8);
		break;
	case internal::ParityMode::NONE:
		break;
	}
	return rcv;
}

bool SerialComm::getEvenParity(char byte) {
//	bool p = false;
//	for(; byte; byte >>= 1)
//		p ^= byte & 1;
//	return p;

	// The parity matrix (0 = even number of 1's)
	static const int parity[256]={
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
		0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};
	return parity[(uint8_t)byte];
}

bool SerialComm::getOddParity(char byte) {
	return getEvenParity(byte) ^ 1;
}

void SerialComm::dataReader() {
	while(!isExiting()) {
		ssize_t count;
		{
			std::lock_guard<std::mutex> lock(portConfigMutex_); // don't allow changes while data is present
			count = internal::port_get_input_queue_size(getCommHandler()->getCommDescriptor());
			do {
				while(count > 0) {
					count -= processRawDataStream();
				}
				count = internal::port_get_input_queue_size(getCommHandler()->getCommDescriptor());
			} while(count > 0 && !isExiting());
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

size_t SerialComm::readNoLock(uint8_t* buf, size_t sz) {
	size_t readSz = 0;
	while(readSz < sz) {
		{
			std::lock_guard<std::mutex> lock(exitMutex_);
			if(exiting_)	 // atomically check for exit condition
				return readSz;
		}
		ssize_t ret = internal::port_read(getCommHandler()->getCommDescriptor(), buf+readSz, sz-readSz);
		if(ret == -1) {
			util::Logger::getInstance()->logWarning("Error while reading from stream - " +
					util::Logger::getPosixErrorDescription(errno) + MAKE_DEBUG_STRING());
			continue;
		}
		readSz += ret;
	}
	return readSz;
}

bool SerialComm::isExiting() {
	std::lock_guard<std::mutex> lock(exitMutex_);
	return exiting_;
}

void SerialComm::setExiting(bool exiting) {
	std::lock_guard<std::mutex> lock(exitMutex_);
	exiting_ = exiting;
}

SerialComm::~SerialComm() {
	util::Logger::getInstance()->log("Stopping communication and closing port...");
	setExiting(true); // atomically set exit flag
	receiveWorker_.join();

	auto handler = getCommHandler();
	internal::_CommHandle_t commDes = handler->getCommDescriptor();
	handler->resetCommDescriptor();
	internal::port_close(commDes);
}

} /* namespace comm */
