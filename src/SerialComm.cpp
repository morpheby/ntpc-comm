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

#define MAX_ERRORS_ON_STREAM 5

namespace comm {


SerialComm::SerialComm(const std::string &connStr) :
	connStr_(connStr), exiting_(false), commPort_(std::make_shared<internal::CommHandler>()),
	receiveWorker_() {
	util::Logger::getInstance()->log("Opening port " + connStr);
	internal::_CommHandle_t commDes = internal::port_open(connStr);
	getCommHandler()->setCommDescriptor(commDes);
	resetCommConfig();
	receiveWorker_ = std::thread(&SerialComm::dataReader, this);
	sendWorker_ = std::thread(&SerialComm::dataWriter, this);
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

bool SerialComm::getEvenParity(uint8_t byte) {
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
	return parity[byte];
}

bool SerialComm::getOddParity(uint8_t byte) {
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
	int errorCount = 0;
	while(readSz < sz && !isExiting()) {
		ssize_t ret = internal::port_read(getCommHandler()->getCommDescriptor(), buf+readSz, sz-readSz);
		if(ret == -1) {
			util::Logger::getInstance()->logWarning("Error while reading from stream - " +
					util::Logger::getPosixErrorDescription(errno) + MAKE_DEBUG_STRING());
			++errorCount;
			if(errorCount == MAX_ERRORS_ON_STREAM)
				throw util::posix_error_exception(" reading from stream, got MAX_ERRORS_ON_STREAM, aborting",
						MAKE_DEBUG_STRING());
			continue;
		}
		readSz += ret;
	}
	return readSz;
}

bool SerialComm::isExiting() const {
	return exiting_;
}

void SerialComm::write9BitByte(uint16_t byte) {
	{
		std::lock_guard<std::mutex> lock (sendMutex_);
		sendBuffer_.push(byte);
	}
	sendDataReady_.notify_one();
}

void SerialComm::dataWriter() {
	while(!isExiting()) {
		std::unique_lock<std::mutex> lock(sendMutex_);

		while(sendBuffer_.empty())
			sendDataReady_.wait(lock);

		while(!sendBuffer_.empty()) {
			processRawOutput(sendBuffer_.front());
			sendBuffer_.pop();
		}
	}
}

internal::ParityMode SerialComm::processParityReverse(uint16_t byteToSend) {
#ifndef NO_SPACEMARK_PARITY
	if(byteToSend & (1 << 8))
		return internal::ParityMode::MARK;
	else
		return internal::ParityMode::SPACE;
#else
	if(getEvenParity(byteToSend & 0xFFU) != (byteToSend >> 8))
		return internal::ParityMode::ODD;
	else
		return internal::ParityMode::EVEN;
#endif
}

void SerialComm::processRawOutput(uint16_t byte) {
	// lock port configuration while we send data
	std::lock_guard<std::mutex> lock (portConfigMutex_);
	internal::ParityMode parity = processParityReverse(byte);
	if(getCommHandler()->getParityMode() != parity) {
#ifdef DEBUG
		std::string parStr;
		switch(parity) {
		case internal::ParityMode::EVEN:
			parStr = "ParityMode::EVEN";
			break;
		case internal::ParityMode::ODD:
			parStr = "ParityMode::ODD";
			break;
#ifndef NO_SPACEMARK_PARITY
		case internal::ParityMode::SPACE:
			parStr = "ParityMode::SPACE";
			break;
		case internal::ParityMode::MARK:
			parStr = "ParityMode::MARK";
			break;
#endif
		default:
			parStr = "--WRONG PARITY--";
			break;
		}
		util::Logger::getInstance()->trace("Switching parity to " + parStr + MAKE_DEBUG_STRING());
#endif
		getCommHandler()->setParityMode(parity);
	}

	ssize_t ret = internal::port_write(getCommHandler()->getCommDescriptor(), &byte, 1);
	if(ret == -1) {
		util::Logger::getInstance()->logWarning("Error while writing to stream [" +
				std::to_string(byte) + "] - " +
				util::Logger::getPosixErrorDescription(errno) + MAKE_DEBUG_STRING());
	} else
		util::Logger::getInstance()->trace("Written byte " + std::to_string(byte) + MAKE_DEBUG_STRING());
}

void SerialComm::setExiting(bool exiting) {
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
