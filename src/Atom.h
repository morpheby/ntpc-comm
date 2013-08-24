/*
 * Atom.h
 *
 *  Created on: 24.08.2013
 *      Author: morpheby
 */

#ifndef ATOM_H_
#define ATOM_H_

#include <mutex>

namespace util {

template <typename T>
class Atom;

template <typename T>
class Atom {
	std::mutex atomMutex_;
	T valueHeld_;

	T getValue();
	void setValue(const T&);
	void setValue(T&&);
public:
	Atom();
	Atom(const T&);
	Atom(T&&);
	Atom(Atom<T>&);
	Atom(Atom<T>&&);

	~Atom();

	operator T();

	Atom<T>& operator = (Atom<T>&);
	Atom<T>& operator = (Atom<T>&&);
	Atom<T>& operator = (const T&);
	Atom<T>& operator = (T&&);
};


} /* namespace util */

#include "Atom.hpp"

#endif /* ATOM_H_ */
