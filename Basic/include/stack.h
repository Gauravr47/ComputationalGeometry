#pragma once

#include "list.h"
using namespace cg;
namespace cg {
	template<class T> class Stack {
	private:
		List<T>* s;
	public:
		Stack(void);
		~Stack(void);
		void push(T);
		T pop(void);
		T top(void);
		T bottom(void);
		T nextToTop(void);
		int size(void);
		bool empty(void);
	};

	template<class T>
	Stack<T>::Stack(void) : s(new List<T>) {

	}

	template<class T>
	Stack<T>::~Stack(void) {
		delete s;
	}

	template<class T>
	void Stack<T>::push(T val) {
		s->prepend(val);
	}

	template<class T>
	T Stack<T>::pop(void) {
		s->first();
		return s->remove();
	}

	template<class T>
	T Stack<T>::top(void) {
		return s->first();
	}

	template<class T>
	T Stack<T>::nextToTop(void) {
		s->first();
		return s->next();
	}

	template<class T>
	T Stack<T>::bottom(void) {
		return s->last();
	}

	template<class T>
	int Stack<T>::size(void) {
		return s->length();
	}

	template<class T>
	bool Stack<T>::empty(void) {
		return (s->length() == 0);
	}
}