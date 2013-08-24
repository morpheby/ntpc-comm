/*
 * Atom.h
 *
 *  Created on: 24.08.2013
 *      Author: morpheby
 */

#ifndef ATOM_H_
#define ATOM_H_

namespace util {

template <typename T>
class Atom;

template <typename T>
class Atom {
public:
	Atom();
	Atom(const T&);
	virtual ~Atom();

	T operator T();
};


} /* namespace util */

#include "Atom.hpp"

#endif /* ATOM_H_ */
