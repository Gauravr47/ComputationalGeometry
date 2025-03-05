#ifndef BASIC_DATA_STRUCTURES_H
#define BASIC_DATA_STRUCTURES_H

#include <stdlib.h>

namespace cg {

	class Node {
	protected:
		Node* _next;
		Node* _prev;
	public:
		Node(void);
		virtual ~Node(void);
		Node* next(void);
		Node* prev(void);
		Node* insert(Node*);
		Node* remove(void);
		void splice(Node*);
	};

	Node::Node(void) : _next(this), _prev(this) {

	}

	Node::~Node(void)
	{
	}

	Node* Node::next(void)
	{
		return _next;
	}

	Node* Node::prev(void)
	{
		return _prev;
	}

	Node* Node::insert(Node* b) {
		Node* c = _next;
		b->_next = c;
		b->_prev = this;
		this->_next = b;
		c->_prev = b;
		return b;
	}

	Node* Node::remove(void)
	{
		_prev->_next = _next;
		_next->_prev = _prev;
		_next = _prev = this;
		return this;
	}

	void Node::splice(Node* b)
	{
		Node* a = this;
		Node* an = a->_next;
		Node* bn = b->_next;
		a->_next = bn;
		b->_next = an;
		an->_prev = b;
		bn->_prev = a;
	}

	///////////////////////////////////////////////////////////////LIST////////////////////////////////////////
	template<class T> class List;

	template<class T> class ListNode : public Node {
	public:
		T _val;
		ListNode(T val);
		friend class List<T>;
	};

	template<class T>
	ListNode<T>::ListNode(T val) : _val(val) {

	}

	template<class T> class List {
	private:
		ListNode<T>* header;
		ListNode<T>* win;
		int _length;
	public:
		List(void);
		~List(void);
		T insert(T);
		T append(T);
		List* append(List*);
		T prepend(T);
		T remove(void);
		void val(T);
		T val(void);
		T next(void);
		T prev(void);
		T first(void);
		T last(void);
		int length(void);
		bool isFirst(void);
		bool isLast(void);
		bool isHead(void);
	};

	template<class T>
	List<T>::List(void) : _length(0)
	{
		header = new ListNode<T>(nullptr);
		win = header;
	}


	template<class T>
	List<T>::~List(void) {
		while (length() > 0) {
			first();
			remove();
		}
		delete header;
	}

	template<class T>
	T List<T>::insert(T val) {
		win->insert(new ListNode<T>(val));
		++_length;
		return val;
	}

	template<class T>
	T List<T>::prepend(T val) {
		header->insert(new ListNode<T>(val));
		++_length;
		return val;
	}

	template<class T>
	T List<T>::append(T val) {
		header->prev()->insert(new ListNode<T>(val));
		++_length;
		return val;
	}

	template<class T>
	List<T>* List<T>::append(List<T>* l) {
		ListNode<T>* a = (ListNode<T>*)header->prev();
		((Node*)a)->splice(l->header);
		_length += l->length();
		l->header->remove();
		l->_length = 0;
		l->win = header;
		return this;
	}

	template<class T>
	T List<T>::remove() {
		if (win == header) {
			return nullptr;
		}
		T val = win->_val;
		win = (ListNode<T>*) win->prev();
		delete (ListNode<T>*) win->next()->remove();
		--_length;
		return val;
	}

	template<class T>
	void List<T>::val(T val) {
		if (win != header)
			win->_val = val;
	}

	template<class T>
	T List<T>::val(void) {
		return win->_val;
	}

	template<class T>
	T List<T>::next(void) {
		win = (ListNode<T>*) win->next();
		return win->_val;
	}

	template<class T>
	T List<T>::prev(void) {
		win = (ListNode<T>*) win->prev();
		return win->_val;
	}

	template<class T>
	T List<T>::first(void) {
		win = (ListNode<T>*) header->next();
		return win->_val;
	}

	template<class T>
	T List<T>::last(void) {
		win = (ListNode<T>*) header->prev();
		return win->_val;
	}

	template<class T>
	int List<T>::length(void) {
		return _length;
	}

	template<class T>
	bool List<T>::isFirst(void) {
		return ((win == header->next()) && (_length > 0));
	}

	template<class T>
	bool List<T>::isLast(void) {
		return ((win == header->prev()) && (_length > 0));
	}

	template<class T>
	bool List<T>::isHead(void) {
		return (win == header);
	}

	template<class T> List<T>* arrayToList(T a[], int n) {
		List<T>* s = new List<T>;
		for (int i = 0; i < n; i++) {
			s->append(a[i]);
		}
		return s;
	}
	/////////////////////////////////////////////////////////////STACK////////////////////////////////////////////////
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
	/////////////////////////////////////////////////////// TREES////////////////////////////////////////////////
	template<class T> class SearchTree;
	template<class T>  class BraidedSearchTree;
	template<class T> class RandomizedSearchTree;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Tree node
	template<class T> class TreeNode {
	protected:
		TreeNode* _lchild;
		TreeNode* _rchild;
		T val;
	public:
		TreeNode(T);
		~TreeNode(void);
		friend class SearchTree<T>;
		friend class BraidedSearchTree<T>;
	};

	template<class T> TreeNode<T>::TreeNode(T v) : val(v), _lchild(nullptr), _rchild(nullptr) {

	};

	template<class T> TreeNode<T>::~TreeNode() {
		if (_lchild) delete _lchild;
		if (_rchild) delete _rchild;
	};

	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//SearchTree Class
	template<class T> class SearchTree {
	private:
		TreeNode<T>* root;
		int (*cmp)(T, T);
		TreeNode<T>* _findMin(TreeNode<T>*);
		void _remove(T, TreeNode<T>*&);
		void _inorder(TreeNode<T>*, void(*visit)(T));
		int _height(TreeNode<T>*&);
	public:
		SearchTree(int(*c)(T, T));
		SearchTree(SearchTree<T>&);
		~SearchTree();
		int isEmpty(void);
		T find(T);
		T findMin(void);
		void inorder(void(*visit)(T));
		void insert(T);
		void remove(T);
		T removeMin(void);
		int height(void);
		void clone(TreeNode<T>*);
	};

	template<class T> SearchTree<T>::SearchTree(int(*c)(T, T)) : cmp(c), root(nullptr) {

	};

	template<class T>
	SearchTree<T>::SearchTree(SearchTree<T>& obj)
	{
		if (!obj.isEmpty()) {
			root = nullptr;
			cmp = obj.cmp;
			clone(obj.root);
		}
	}

	template<class T> int SearchTree<T>::isEmpty(void) {
		return (root == nullptr);
	};


	template<class T> SearchTree<T>::~SearchTree(void) {
		if (root) delete root;
	};

	template<class T> T SearchTree<T>::find(T val) {
		TreeNode<T>* n = root;
		while (n) {
			int result = (*cmp)(val, n->val);
			if (result < 0) {
				n = n->_lchild;
			}
			else if (result > 0) {
				n = n->_rchild;
			}
			else {
				return n->_val;
			}
		}
		return nullptr;
	}

	template<class T> T SearchTree<T>::findMin(void) {
		TreeNode<T>* node = _findMin(root);
		return (node ? node->val : nullptr);
	}

	template<class T> TreeNode<T>* SearchTree<T>::_findMin(TreeNode<T>* n) {
		if (n == nullptr)
			return nullptr;
		while (n->_lchild)
			n = n->_lchild;
		return n;
	}

	template<class T> void SearchTree<T>::inorder(void(*visit)(T)) {
		_inorder(root, visit);
		return;
	}

	template<class T> void SearchTree<T>::_inorder(TreeNode<T>* n, void(*visit)(T)) {
		if (n) {
			_inorder(n->_lchild, visit);
			(*visit)(n->val);
			_inorder(n->_rchild, visit);
		}
		return;
	}

	template<class T> void SearchTree<T>::insert(T val) {
		if (root == nullptr) {
			root = new TreeNode<T>(val);
			return;
		}
		int result;
		TreeNode<T>* n = root;
		TreeNode<T>* p = root;
		while (n) {
			p = n;
			result = (*cmp)(val, p->val);
			if (result < 0) {
				if (p->_lchild != nullptr)
					n = p->_lchild;
				else
					break;
			}
			else if (result > 0) {
				if (p->_rchild != nullptr)
					n = p->_rchild;
				else
					break;
			}
			else {
				return;
			}
		}
		if (result < 0) {
			n->_lchild = new TreeNode<T>(val);
		}
		else {
			n->_rchild = new TreeNode<T>(val);
		}
		return;
	}


	template<class T> void SearchTree<T>::remove(T val) {
		_remove(val, root);
	}

	template<class T> void SearchTree<T>::_remove(T val, TreeNode<T>*& n) {
		if (n == nullptr) {
			return;
		}
		int result = (*cmp)(val, n->val);
		if (result < 0) {
			_remove(val, n->left);
		}
		else if (result > 0) {
			_remove(val, n->right);
		}
		else {
			if (n->left == nullptr) {
				TreeNode<T>* old = n;
				n = old->right;
				delete old;
			}
			else if (n->right == nullptr) {
				TreeNode<T>* old = n;
				n = old->left;
				delete old;
			}
			else {
				TreeNode<T>* m = _findMin(n->right);
				n->val = m->val;
				_remove(m->val, n->right);
			}
		}
		return;
	}

	template<class T> T SearchTree<T>::removeMin(void) {
		T val = findMin();
		remove(val);
		return val;
	}

	template<class T> int SearchTree<T>::_height(TreeNode<T>*& n) {
		if (n == nullptr) {
			return 0;
		}
		int left = _height(n->_lchild);
		int right = _height(n->_rchild);
		return max(left, right) + 1;
	}

	template<class T> int SearchTree<T>::height(void) {
		return _height(root);
	}

	template<class T>
	void SearchTree<T>::clone(TreeNode<T>* n)
	{
		if (n != nullptr) {
			insert(n->val);
			clone(n->_lchild);
			clone(n->_rchild);
		}
		return;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//BraidedNode Class

	template<class T> class BraidedNode : public Node, public TreeNode<T> {
	public:
		BraidedNode(T);
		BraidedNode<T>* lchild();
		BraidedNode<T>* rchild();
		BraidedNode<T>* next();
		BraidedNode<T>* prev();
		friend class BraidedSearchTree<T>;
	};

	template<class T> BraidedNode<T>::BraidedNode(T v) : TreeNode<T> (v) {

	};

	template<class T> BraidedNode<T>* BraidedNode<T>::lchild(void) {
		return (BraidedNode<T>*) this->_lchild;
	};

	template<class T> BraidedNode<T>* BraidedNode<T>::rchild(void) {
		return (BraidedNode<T>*) this->_rchild;
	};

	template<class T> BraidedNode<T>* BraidedNode<T>::next(void) {
		return (BraidedNode<T>*) _next;
	};

	template<class T> BraidedNode<T>* BraidedNode<T>::prev(void) {
		return (BraidedNode<T>*) _prev;
	};


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//BraidedSearchtree Class

	template<class T> class BraidedSearchTree {
	private:
		BraidedNode<T>* root;
		BraidedNode<T>* win;
		int (*cmp)(T, T);
		void _remove(T, TreeNode<T>*&);
	public:
		BraidedSearchTree(int(*c)(T, T));
		~BraidedSearchTree();
		T next();
		T prev();
		T val(void);
		T find(T);
		T findMin();
		T insert(T);
		void remove(T);
		T removeMin(void);
		void inorder(void(*visit)(T));
		int isEmpty(void);
		int isHead(void);
		int isFirst(void);
		int isLast(void);
	};

	template<class T>
	void BraidedSearchTree<T>::_remove(T val, TreeNode<T>*& n)
	{
		int result = (*cmp)(val, n->val);
		if (result < 0) {
			_remove(val, n->_lchild);
		}
		else if (result > 0) {
			_remove(val, n->_rchild);
		}
		else {
			if (n->_lchild == nullptr) {
				BraidedNode<T>* old = (BraidedNode<T>*)n;
				if (win == old)
					win = old->prev();
				n = old->rchild();
				old->Node::remove();
				delete old;
			}
			else if (n->_rchild == nullptr) {
				BraidedNode<T>* old = (BraidedNode<T>*)n;
				if (win == old)
					win = old->prev();
				n = old->lchild();
				old->Node::remove();
				delete old;
			}
			else {
				BraidedNode<T>* m = ((BraidedNode<T>*)n)->next();
				n->val = m->val;
				_remove(m->val, n->rchild());

			}
		}
	}

	template<class T>
	BraidedSearchTree<T>::BraidedSearchTree(int(*c)(T, T)) : cmp(c)
	{
		win = root = new BraidedNode<T>(nullptr);
	}

	template<class T>
	BraidedSearchTree<T>::~BraidedSearchTree()
	{
		delete root;
	}

	template<class T>
	T BraidedSearchTree<T>::next()
	{
		win = win->next();
		return win->val;
	}

	template<class T>
	T BraidedSearchTree<T>::prev()
	{
		win = win->prev();
		return win->val;
	}

	template<class T>
	T BraidedSearchTree<T>::val()
	{
		return win->val;
	}

	template<class T>
	T BraidedSearchTree<T>::find(T val)
	{
		BraidedNode<T>* n = root->rchild();
		while (n) {
			int result = (*cmp)(val, n->val);
			if (result < 0) {
				n = n->lchild();
			}
			else if (result > 0) {
				n = n->rchild();
			}
			else {
				win = n;
				return n->val;
			}
		}
		return nullptr;
	}

	template<class T>
	T BraidedSearchTree<T>::findMin()
	{
		win = root->next();
		return win->val;
	}

	template<class T>
	T BraidedSearchTree<T>::insert(T val)
	{
		BraidedNode<T>* p = root;
		BraidedNode<T>* n = root->rchild();
		int result = 1;
		while (n) {
			p = n;
			result = (*cmp)(val, n->val);
			if (result < 0) {
				n = p->lchild();
			}
			else if (result > 0) {
				n = p->rchild();
			}
			else {
				return nullptr;
			}
		}
		win = new BraidedNode<T>(val);
		if (result < 0) {
			p->_lchild = win;
			p->prev()->Node::insert(win);
		}
		else {
			p->_rchild = win;
			p->Node::insert(win);
		}
		return nullptr;
	}

	template<class T>
	void BraidedSearchTree<T>::remove(T val)
	{
		if (win != root)
			_remove(val, root->rchild());
	}

	template<class T>
	T BraidedSearchTree<T>::removeMin(void)
	{
		T val = root->next()->val;
		if (root != root->next())
			_remove(val, root->rchild());
		return val;
	}

	template<class T>
	void BraidedSearchTree<T>::inorder(void(*visit)(T))
	{
		BraidedNode<T>* n = root->next();
		while (n != root) {
			(*visit)(n->val);
			n = n->next();
		}
		return;
	}

	template<class T>
	int BraidedSearchTree<T>::isEmpty(void)
	{
		return (root == root->next());
	}

	template<class T>
	int BraidedSearchTree<T>::isHead(void)
	{
		return (win == root);
	}

	template<class T>
	int BraidedSearchTree<T>::isFirst(void)
	{
		return (win == root->next()) && (root != root->next());
	}

	template<class T>
	int BraidedSearchTree<T>::isLast(void)
	{
		return (win == root->prev()) && (root != root->next());
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//RandomizedNode Class

	template<class T> class RandomizedNode : public BraidedNode<T> {
	private:
		double _priority;
		RandomizedNode* _parent;
		void rotateRight(void);
		void rotateLeft(void);
		void bubbleUp(void);
		void bubbleDown(void);
	public:
		RandomizedNode(T val, int seed = -1);
		~RandomizedNode(void);
		RandomizedNode<T>* lchild(void);
		RandomizedNode<T>* rchild(void);
		RandomizedNode<T>* next(void);
		RandomizedNode<T>* prev(void);
		RandomizedNode<T>* parent(void);
		double priority(void);
		friend class RandomizedSearchTree<T>;
	};

	template<class T>
	void RandomizedNode<T>::rotateRight(void)
	{
		RandomizedNode* y = this;
		RandomizedNode* x = y->lchild();
		RandomizedNode* parent = y->parent();
		y->_lchild = x->rchild();
		if (y->lchild() != nullptr) {
			y->lchild()->_parent = y;
		}
		if (parent->rchild() == y) {
			parent->_rchild = x;
		}
		else {
			parent->_lchild = x;
		}
		x->_rchild = y;
		x->_parent = parent;
		y->_parent = x;
	}

	template<class T>
	void RandomizedNode<T>::rotateLeft(void)
	{
		RandomizedNode* x = this;
		RandomizedNode* y = x->rchild();
		RandomizedNode* parent = x->parent();
		x->_rchild = y->lchild();
		if (x->rchild() != nullptr) {
			x->rchild()->_parent = x;
		}
		if (parent->rchild() == x) {
			parent->_rchild = y;
		}
		else {
			parent->_lchild = y;
		}
		y->_lchild = x;
		y->_parent = parent;
		x->_parent = y;

	}

	template<class T>
	void RandomizedNode<T>::bubbleUp(void)
	{
		RandomizedNode<T>* parent = this->parent();
		if (priority() < parent->priority()) {
			if (parent->lchild() == this) {
				parent->rotateRight();
			}
			else {
				parent->rotateLeft();
			}
			bubbleUp();
		}
	}

	template<class T>
	void RandomizedNode<T>::bubbleDown(void)
	{
		double lPriority = lchild() ? lchild()->priority() : 2.0;
		double rPriority = rchild() ? rchild()->priority() : 2.0;
		double minPriority = (lPriority < rPriority) ? lPriority : rPriority;
		if (priority() <= minPriority)
			return;
		if (lPriority < rPriority)
		{
			rotateRight();
		}
		else {
			rotateLeft();
		}
		bubbleDown();
	}

	template<class T>
	RandomizedNode<T>::RandomizedNode(T v, int seed) : BraidedNode<T> (v)
	{

		if (seed != -1) srand(seed);
		_priority = (rand() % 32767) / 32767.0;
		_parent = nullptr;
	}

	template<class T>
	RandomizedNode<T>::~RandomizedNode(void)
	{

	}

	template<class T>
	RandomizedNode<T>* RandomizedNode<T>::lchild(void)
	{
		return (RandomizedNode<T>*) this->_lchild;
	}

	template<class T>
	RandomizedNode<T>* RandomizedNode<T>::rchild(void)
	{
		return (RandomizedNode<T>*) BraidedNode<T>::rchild();
	}

	template<class T>
	RandomizedNode<T>* RandomizedNode<T>::next(void)
	{
		return (RandomizedNode<T>*) this->_next;
	}

	template<class T>
	RandomizedNode<T>* RandomizedNode<T>::prev(void)
	{
		return (RandomizedNode<T>*) _prev;
	}

	template<class T>
	RandomizedNode<T>* RandomizedNode<T>::parent(void)
	{
		return (RandomizedNode<T>*) _parent;
	}

	template<class T>
	double RandomizedNode<T>::priority(void)
	{
		return _priority;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //RandomizedSearchTree Class

	template<class T> class RandomizedSearchTree {
	private:
		RandomizedNode<T>* root;
		RandomizedNode<T>* win;
		int (*cmp)(T, T);
		void _remove(RandomizedNode<T>*);
	public:
		RandomizedSearchTree(int(*c)(T, T), int = -1);
		~RandomizedSearchTree(void);
		T next();
		T prev();
		T val(void);
		T find(T);
		T findMin();
		T locate(T);
		T insert(T);
		void remove(void);
		T remove(T);
		T removeMin(void);
		void inorder(void(*visit)(T));
		int isEmpty(void);
		int isHead(void);
		int isFirst(void);
		int isLast(void);
	};

	template<class T>
	RandomizedSearchTree<T>::RandomizedSearchTree(int(*c)(T, T), int seed) :cmp(c) {
		root = win = new RandomizedNode<T>(nullptr, seed);
		root->_priority = -1.0;
	};

	template<class T>
	RandomizedSearchTree<T>::~RandomizedSearchTree(void) {
		delete root;
	}

	template<class T>
	void RandomizedSearchTree<T>::_remove(RandomizedNode<T>* n)
	{
		n->_priority = 1.5;
		n->bubbleDown();
		RandomizedNode<T>* parent = n->parent();
		if (parent->_lchild == n) {
			parent->_lchild = nullptr;
		}
		else {
			parent->_rchild = nullptr;
		}
		if (win == n) {
			win = n->prev();
		}
		n->Node::remove();
		delete n;
	}

	template<class T>
	void RandomizedSearchTree<T>::remove(void)
	{
		if (win != root)
			_remove(win);
	}

	template<class T>
	T RandomizedSearchTree<T>::removeMin(void)
	{
		T v = root->next()->val;
		if (root != root->next()) {
			_remove(root->next());
		}
		return v;
	}

	template<class T>
	T RandomizedSearchTree<T>::remove(T val)
	{
		T v = find(val);
		if (v) {
			remove();
			return v;
		}
		return nullptr;
	}

	template<class T>
	T RandomizedSearchTree<T>::find(T val)
	{
		RandomizedNode<T>* n = root->rchild();
		while (n) {
			int result = (*cmp)(val, n->val);
			if (result < 0) {
				n = n->lchild();
			}
			else if (result > 0) {
				n = n->rchild();
			}
			else {
				win = n;
				return val;
			}
		}
		return nullptr;
	}

	template<class T>
	T RandomizedSearchTree<T>::findMin()
	{
		win = root->next();
		return win->val();
	}

	template<class T>
	T RandomizedSearchTree<T>::locate(T val)
	{
		RandomizedNode<T>* b = root;
		RandomizedNode<T>* n = root->rchild();
		while (n) {
			int result = (*cmp)(val, n->val);
			if (result < 0) {
				n = n->lchild();
			}
			else if (result > 0) {
				b = n;
				n = n->rchild();
			}
			else {
				win = n;
				return win->val;
			}
		}
		win = b;
		return win->val;
	}

	template<class T>
	T RandomizedSearchTree<T>::insert(T val)
	{
		int result = 1;
		RandomizedNode<T>* p = root;
		RandomizedNode<T>* n = root->rchild();
		while (n) {
			p = n;
			result = (*cmp)(val, n->val);
			if (result < 0) {
				n = n->lchild();
			}
			else if (result > 0) {
				n = n->rchild();
			}
			else {
				return nullptr;
			}
		}
		win = new RandomizedNode<T>(val);
		win->_parent = p;
		if (result < 0) {
			p->_lchild = win;
			p->prev()->Node::insert(win);
		}
		else {
			p->_rchild = win;
			p->Node::insert(win);
		}
		win->bubbleUp();
		return win->val;
	}

	template<class T>
	T RandomizedSearchTree<T>::next()
	{
		win = win->next();
		return win->val;
	}

	template<class T>
	T RandomizedSearchTree<T>::prev()
	{
		win = win->prev();
		return win->val;
	}

	template<class T>
	T RandomizedSearchTree<T>::val(void)
	{
		return win->val;
	}

	template<class T>
	int RandomizedSearchTree<T>::isEmpty(void)
	{
		return (root == root->next());
	}

	template<class T>
	int RandomizedSearchTree<T>::isHead(void)
	{
		return (win == root);
	}

	template<class T>
	int RandomizedSearchTree<T>::isFirst(void)
	{
		return (root != root->next()) && (win == root->next());
	}

	template<class T>
	int RandomizedSearchTree<T>::isLast(void)
	{
		return (root != root->prev()) && (win == root->prev());
	}

	template<class T>
	void RandomizedSearchTree<T>::inorder(void(*visit)(T))
	{
		RandomizedNode<T>* n = root->next();
		while (n != root) {
			(*visit)(n->val);
			n = n->next();
		}
		return;
	}

}

#define Dictionary RandomizedSearchTree



#endif // !BASIC_DATA_STRUCTURES_H