/*
 * Atom.hpp
 *
 *  Created on: 24.08.2013
 *      Author: morpheby
 */

#ifndef ATOM_HPP_
#define ATOM_HPP_

#include <utility>

namespace util {


template <class T>
Atom<T>::Atom() : valueHeld_() {
}

template <class T>
Atom<T>::Atom(const T& v) : valueHeld_(v) {
}

template <class T>
Atom<T>::Atom(Atom<T>& av) : valueHeld_(av.getValue()) {
}

template <class T>
Atom<T>::Atom(T&& v) : valueHeld_(std::move(v)) {
}

template <class T>
Atom<T>::Atom(Atom<T>&& av) : atomMutex_(std::move(av.atomMutex_)),
		valueHeld_(std::move(av.valueHeld_)) {
}

template <class T>
Atom<T>::~Atom () {
}

template <class T>
T Atom<T>::getValue() {
	std::lock_guard<std::mutex> lock(atomMutex_);
	return valueHeld_;
}

template <class T>
void Atom<T>::setValue(const T& v) {
	std::lock_guard<std::mutex> lock(atomMutex_);
	valueHeld_ = v;
}

template <class T>
void Atom<T>::setValue(T&& v) {
	std::lock_guard<std::mutex> lock(atomMutex_);
	valueHeld_ = std::move(v);
}

template <class T>
Atom<T>::operator T() {
	return getValue();
}

template <class T>
Atom<T>& Atom<T>::operator = (Atom<T> &av) {
	setValue(av.getValue());
	return *this;
}

template <class T>
Atom<T>& Atom<T>::operator = (const T& v) {
	setValue(v);
	return *this;
}

template <class T>
Atom<T>& Atom<T>::operator = (Atom<T>&& av) {
	setValue(std::move(av.valueHeld_));
	return *this;
}

template <class T>
Atom<T>& Atom<T>::operator = (T&& v) {
	setValue(std::move(v));
	return *this;
}

}  // namespace util

#endif /* ATOM_HPP_ */
