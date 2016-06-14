#pragma once

template <typename T>
class unique_ptr_d
{

	T* _ptr;

public:

	// Make an empty smart pointer
	unique_ptr_d() :
		_ptr(0)
	{}

	// Make a smart pointer and give it a pointer to look after
	unique_ptr_d(T* ptr) :
		_ptr(ptr)
	{}

	// Destroy the smart pointer. This will also destroy the managed object if there is one
	~unique_ptr_d() {
		if (_ptr) {
			delete _ptr;
		}
	}

	// Checks if a pointer is assigned
	operator bool() const {
		if (_ptr)
			return true;

		return false;
	}

	// Gets the pointer
	T* get() {
		return _ptr;
	}

	// Dereferences to the held object
	T& operator * () {
		return *_ptr;
	}

	// Dereferences to the held object - const
	const T& operator * () const {
		return *_ptr;
	}

	// Sets the pointer to the given, releasing the current one if non-null
	void reset(T* newPtr) {

		if (_ptr) {
			delete(_ptr);
		}

		_ptr = newPtr;

	}
	// Returns a pointer to the managed object and releases the ownership 
	T* release() {
		T* ptr = _ptr;

		_ptr = 0;

		return ptr;
	}

	// Don't allow copying: this pointer is unique
	unique_ptr_d(const unique_ptr_d&) = delete;
};