/*
 * Atom_test.cpp
 *
 *  Created on: 24.08.2013
 *      Author: morpheby
 */

#include "platform.h"

#include "Atom.h"

#include <thread>
#include <assert.h>
#include <iostream>

struct move_test {
	move_test() {
	}

	move_test(const move_test & a) {
		throw "Called non-moving c-tor";
	}

	move_test(move_test &&a) {
	}

	move_test & operator = (const move_test& a) {
		throw "Called non-moving operator=";
	}
	move_test & operator = (move_test && a) {
		return *this;
	}
};

struct race_test {
	int a, b, c, d, e, f;
};

int main() {
	try {
		std::cout << "Test started" << std::endl;

		std::cout << "Constructors..." << std::endl;

		util::Atom<int> atom1(5), atom2;

		std::cout << "Operators and conversions..." << std::endl;

		assert(atom1 == 5);
		atom1 = 4;
		assert(atom1 == 4);
		atom2 = atom1;
		assert(atom2 == 4);

		int a = atom2;
		assert(a == 4);

		int b (util::Atom<int>(6));
		assert(b == 6);

		std::cout << "Move semantics..." << std::endl;

		util::Atom<move_test> atom3, atom4(move_test()), atom5(util::Atom<move_test>());
		atom3 = move_test();
		atom3 = util::Atom<move_test>();

		std::cout << "Race test..." << std::endl;

		util::Atom<race_test> atom6 (race_test {0, 0, 0, 0, 0, 0});

		std::thread thrd1 ( [&] () {
			std::cout << "1-ing thread started" << std::endl;
			int i = 1000000;
			while(--i) {
				atom6 = race_test {1, 1, 1, 1, 1, 1};
			}
			std::cout << "1-ing thread finished" << std::endl;
		}
				);
		std::thread thrd2 ( [&] () {
			std::cout << "0-ing thread started" << std::endl;
			int i = 1000000;
			while(--i) {
				atom6 = race_test {0, 0, 0, 0, 0, 0};
			}
			std::cout << "0-ing thread finished" << std::endl;
		}
				);
		std::thread thrd3 ( [&] () {
			std::cout << "Comparing thread started" << std::endl;
			int i = 1000000;
			while(--i) {
				race_test a = atom6;
				assert(a.a == a.b);
				assert(a.a == a.c);
				assert(a.a == a.d);
				assert(a.a == a.f);
			}
			std::cout << "Comparing thread finished" << std::endl;
		}
				);
		thrd1.join();
		thrd2.join();
		thrd3.join();

	} catch (const std::string &s) {
		std::cout << s << std::endl;
		return 1;
	} catch (const std::exception &e) {
		std::cout << e.what() << std::endl;
		return 1;
	} catch (...) {
		std::cout << "Exception" << std::endl;
		return 1;
	}

	std::cout << "Tests finished" << std::endl;

	return 0;
}

