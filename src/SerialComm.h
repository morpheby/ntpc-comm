/*
 * SerialComm.h
 *
 *  Created on: 23.07.2013
 *      Author: morpheby
 */

#ifndef SERIALCOMM_H_
#define SERIALCOMM_H_

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

#include "Atom.h"

/* Add -DNO_SPACEMARK_PARITY, if your model doesn't support space/mark parity */

namespace comm {

namespace internal {
class CommHandler;
enum class ParityMode;
}

class SerialComm {
	const std::string connStr_;
	util::Atom<bool> exiting_;
	const std::shared_ptr<internal::CommHandler> commPort_;
	std::thread receiveWorker_, sendWorker_;
	std::mutex portConfigMutex_, recvMutex_, sendMutex_;
	std::condition_variable receiveDataReady_, sendDataReady_;

	std::queue<uint16_t> receiveBuffer_, sendBuffer_;


	uint16_t read9BitByte();
	int processRawDataStream();

	static uint16_t processParityBit(char received, bool isParityError, internal::ParityMode parMode);
	static internal::ParityMode processParityReverse(uint16_t byteToSend);
	static bool getEvenParity(uint8_t byte);
	static bool getOddParity(uint8_t byte);
	size_t readNoLock(uint8_t *buf, size_t sz);

	void write9BitByte(uint16_t byte);
	void processRawOutput(uint16_t byte);
protected:
	std::shared_ptr<internal::CommHandler> getCommHandler() const;
	void resetCommConfig();
	void dataReader();
	void dataWriter();
public:
	SerialComm(const std::string &connectionString);
	virtual ~SerialComm();
	bool isExiting() const;
	void setExiting(bool exiting);
	template <typename _ForwardIterator>
	void read(_ForwardIterator begin, _ForwardIterator end);
	template <typename _ForwardIterator>
	void write(_ForwardIterator begin, _ForwardIterator end);
};

} /* namespace comm */

#include "SerialComm.hpp"

#endif /* SERIALCOMM_H_ */
