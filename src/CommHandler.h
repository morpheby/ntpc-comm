/*
 * CommHandler.h
 *
 *  Created on: 23.07.2013
 *      Author: morpheby
 */

#ifndef COMMHANDLER_H_
#define COMMHANDLER_H_

#ifdef HAVE_TERMIOS_H
#include <termios.h>
#endif

#include <memory>

namespace comm {
namespace internal {

typedef int _CommFlags_t;

#ifdef COMM_USE_WINDOWS_BACKEND
typedef unsigned long _CommSpeed_t;
#else
typedef speed_t _CommSpeed_t;
#endif

typedef int _CommHandle_t;

#ifdef COMM_USE_WINDOWS_BACKEND
#define unsigned long _CommCfg_t;
#else
typedef termios _CommCfg_t;
#endif

enum class ParityMode {
	NONE,
	ODD,
	EVEN,
#ifndef NO_SPACEMARK_PARITY
	MARK,
	SPACE,
#endif
};

class CommHandler {
	std::shared_ptr<_CommCfg_t> cfg_;
	_CommHandle_t commDes_;
	bool desSet_;
public:
	CommHandler();
	virtual ~CommHandler();

	void setBaud(_CommSpeed_t baud);
	_CommSpeed_t getBaud() const;

	void resetProcessingFlags();

	void setPlatformCompatibilityFlags();
	void set8BitComm();
	void setStopBitsCount(int count);
	int getStopBitsCount() const;

	void setRecieveEnable(bool enable);
	bool isRecieveEnabled() const;

	void setParityMode(ParityMode mode);
	ParityMode getParityMode() const;

	void setParityErrorIgnore(bool ignore);
	void setParityErrorMark(bool mark);
	void setParityErrorCheckEnable(bool enable);

	void setCommDescriptor(_CommHandle_t des);
	void resetCommDescriptor();
	_CommHandle_t getCommDescriptor() const;

	void setMapBreakToInterrupt(bool map);

protected:
	void readConfig();
	void writeConfig();

	std::shared_ptr<_CommCfg_t> getCommConfigPtr();
	std::shared_ptr<const _CommCfg_t> getCommConfigPtr() const;

	bool isCommDescriptorSet() const;
};

} /* namespace internal */
} /* namespace comm */
#endif /* COMMHANDLER_H_ */
