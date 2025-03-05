#pragma once

#include "basic_data_structures.h"
#include "geometry.h"
using namespace cg;

namespace cg {
	template<class T>
	T leastItem(List<T>& s, int(*cmp)(T, T)) {
		int i;
		if (s.length() == 0)
			return nullptr;
		T v = s.first();
		for (s.next(); !s.isHead(); s.next()) {
			if (cmp(s.val(), v) < 0) {
				v = s.val();
			}
		}
		return v;
	}

	void reverseString(char* [], int);

	template<class T> void heapSort(T s[], int n, int(*cmp)(T, T))
	{
		SearchTree<T> t(cmp);
		for (int i = 0; i < n; i++)
			t.insert(s[i]);
		for (int i = 0; i < n; i++)
			s[i] = t.removexino;
	}

	template<class T>
	void swap(T& a, T& b) {
		T inter = a;
		a = b;
		b = inter;
		return;
	}

	template<class T>
	void insertionSort(T a[], int n, int(*cmp)(T, T)) {
		for (int i = 1; i < n; i++) {
			T v = a[i];
			int j = i;
			while (j > 0 && ((*cmp)(v, a[j - 1]) < 0)) {
				a[j] = a[j - 1];
				j--;
			}
			a[j] = v;
		}
		return;
	}

	template<class T>
	void selectionSort(T a[], int n, int(*cmp)(T, T)) {
		int min;
		for (int i = 0; i < n - 1; i++) {
			min = i;
			for (int j = i + 1; j < n; j++) {
				if ((*cmp)(a[j], a[min]) < 0)
					min = j;
			}
			swap(a[i], a[min]);
		}
		return;
	}

	template<class T>
	void merge(T x[], int l, int m, int r, int(*cmp)(T, T)) {
		T* a = x + l;
		T* b = x + m + 1;
		T* c = new T[r - l + 1];
		int aindx = 0, bindx = 0, cindx = 0;
		int alim = m - l + 1, blim = r - m;
		while ((aindx < alim) && (bindx < blim)) {
			if ((*cmp)(a[aindx], b[bindx]) < 0) {
				c[cindx++] = a[aindx++];
			}
			else {
				c[cindx++] = b[bindx++];
			}
		}
		while (aindx < alim) {
			c[cindx++] = a[aindx++];
		}
		while (bindx < blim) {
			c[cindx++] = b[bindx++];
		}
		for (aindx = cindx = 0; aindx <= r - l; a[aindx++] = c[cindx++]);
		delete c;
	}

	template<class T>
	void mSort(T a[], int l, int r, int (*cmp)(T, T)) {
		if (l < r) {
			int m = (r + l) / 2;
			mSort(a, l, m, cmp);
			mSort(a, m + 1, r, cmp);
			merge(a, l, m, r, cmp);
		}
	}

	template<class T>
	void mergeSort(T a[], int n, int(*cmp)(T, T)) {
		mSort(a, 0, n - 1, cmp);
	}

	int leftToRightCmp(Point*, Point*);
	int rightToLeftCmp(Point*, Point*);
	int bottomToTopCmp(Point*, Point*);
	int leftToRightCmp(Vertex*, Vertex*);
	int rightToLeftCmp(Vertex*, Vertex*);
	void splitPointSet(Point* [], int, Point*, Point* [], Point* [], int(*cmp)(Point*, Point*));
}