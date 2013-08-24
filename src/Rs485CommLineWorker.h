/*
 * Rs485CommLineWorker.h
 *
 *  Created on: 23.08.2013
 *      Author: morpheby
 */

#ifndef RS485COMMLINEWORKER_H_
#define RS485COMMLINEWORKER_H_

#include <thread>
#include <queue>
#include <memory>
#include <mutex>

#include "SerialComm.h"

namespace rs485 {

typedef uint8_t _MachineAddress_t;

class Rs485CommLineWorker {
	_MachineAddress_t localAddress_;
	std::thread worker_;
	bool receiveActive_;
	bool promiscuousMode_;
	bool exiting_;
	std::queue<uint8_t> dataStream_;
	std::unique_ptr<comm::SerialComm> connection_;
	std::mutex receiveFlagsMutex_, receiveDataMutex_;

	void rs485Worker();

protected:
	void setReceiveActive(bool receiveActive);

public:
	Rs485CommLineWorker(_MachineAddress_t localAddress,
			const std::unique_ptr<comm::SerialComm>& connection);

	virtual ~Rs485CommLineWorker();

	_MachineAddress_t getLocalAddress() const;

	const std::unique_ptr<comm::SerialComm>& getConnection() const;

	bool isPromiscuousMode() const;

	void setPromiscuousMode(bool promiscuousMode);

	bool isReceiveActive() const;

	bool isExiting() const;

	void setExiting(bool exiting);
};

} /* namespace rs485 */
#endif /* RS485COMMLINEWORKER_H_ */
