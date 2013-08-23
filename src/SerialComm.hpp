/*
 * SerialComm.hpp
 *
 *  Created on: 26.07.2013
 *      Author: morpheby
 */

#ifndef SERIALCOMM_HPP_
#define SERIALCOMM_HPP_


namespace comm {

template <typename _ForwardIterator>
void SerialComm::read(_ForwardIterator begin, _ForwardIterator end) {
	while(begin != end) {
		*(begin++) = read9BitByte();
	}
}

} /* namespace comm */


#endif /* SERIALCOMM_HPP_ */
