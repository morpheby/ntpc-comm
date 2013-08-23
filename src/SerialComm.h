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

/* Add -DNO_SPACEMARK_PARITY, if your model doesn't support space/mark parity */

namespace comm {

namespace internal {
class CommHandler;
enum class ParityMode;
}

class SerialComm {
	std::string connStr_;
	bool exiting_;
	std::shared_ptr<internal::CommHandler> commPort_;
	std::thread recieveWorker_, sendWorker_;
	std::mutex portConfigMutex_, recvMutex_, sendMMutex_, exitMutex_;
	std::condition_variable recieveDataReady_;

	std::queue<uint16_t> recieveBuffer_;


	uint16_t read9BitByte();
	int processRawDataStream();

	static uint16_t processParityBit(char recieved, bool isParityError, internal::ParityMode parMode);
	static bool getEvenParity(char byte);
	static bool getOddParity(char byte);
	int readNoLock(int filedes, uint8_t *buf, int sz);

	void write9BitByte(uint16_t byte);
protected:
	std::shared_ptr<internal::CommHandler> getCommHandler() const;
	void resetCommConfig();
	void dataReader();
public:
	SerialComm(const std::string &connectionString);
	virtual ~SerialComm();

	template <typename _ForwardIterator>
	void read(_ForwardIterator begin, _ForwardIterator end);
};

} /* namespace comm */

#include "SerialComm.hpp"

#endif /* SERIALCOMM_H_ */
