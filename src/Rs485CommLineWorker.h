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

#include "Atom.h"

#include "SerialComm.h"

namespace rs485 {

typedef uint8_t _MachineAddress_t;

class Rs485CommLineWorker {
	const _MachineAddress_t localAddress_;
	std::thread worker_;
	util::Atom<bool> receiveActive_;
	util::Atom<bool> promiscuousMode_;
	util::Atom<bool> exiting_;
	std::queue<uint8_t> dataStream_;
	std::unique_ptr<comm::SerialComm> connection_;
	std::mutex receiveDataMutex_;

	void rs485Worker();

protected:
	void setReceiveActive(bool receiveActive);

public:
	Rs485CommLineWorker(_MachineAddress_t localAddress,
			const std::unique_ptr<comm::SerialComm>& connection);

	virtual ~Rs485CommLineWorker();

	_MachineAddress_t getLocalAddress() const;

	const std::unique_ptr<comm::SerialComm>& getConnection() const;

	bool isPromiscuousMode();

	void setPromiscuousMode(bool promiscuousMode);

	bool isReceiveActive();

	bool isExiting();

	void setExiting(bool exiting);
};

} /* namespace rs485 */
#endif /* RS485COMMLINEWORKER_H_ */
