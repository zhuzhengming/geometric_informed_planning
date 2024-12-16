#pragma once 

#include <Python.h>
#include <string>
#include <vector>
#include <iostream>

class CPyInstance
{
public:
	CPyInstance(){
		Py_Initialize();
	}

	~CPyInstance(){
		Py_Finalize();
	}
};

/**
 * This class is a wrapper for the PyObject. 
 * It automatically decreases the object's reference count by one when the class instance is destroyed. 
 */
class CPyObject
{

private:

	PyObject* m_p; // PyObject pointer 

public:
	
	CPyObject() : m_p(NULL) {}

	CPyObject(PyObject* p) : m_p(p) {}

	
	~CPyObject(){
		release();
	}

	PyObject* getObject(){
		return m_p;
	}

	PyObject* operator ->(){
		return m_p;
	}

	operator PyObject*(){
		return m_p;
	}

	PyObject* setObject(PyObject* p){
		return (m_p=p);
	}
	
	PyObject* operator= (PyObject* p){
		m_p = p;
		return m_p;
	}

	PyObject* addRef(){
		if(m_p){
			Py_INCREF(m_p);
		}
		return m_p;
	}

	void release(){
		if(m_p){
			Py_DECREF(m_p);
		}
		m_p= NULL;
	}
	
	bool is(){
		return m_p ? true : false;
	}
	
	operator bool(){
		return m_p ? true : false;
	}

	/**
	 * Set the value of an attribute of the class instance 
	 * @param attr attribute name
	 * @param value value to set the attribute to 
	 */
	template<typename T>
	int setAttr(const char* attr, T value){
		if constexpr (std::is_same<T,double>::value || std::is_same<T,float>::value){
			PyObject_SetAttrString(m_p, attr, PyFloat_FromDouble(value));
			return 0; 
		}
		else if constexpr (std::is_same<T,std::string>::value){
			PyObject_SetAttrString(m_p, attr, Py_BuildValue("s",value.c_str())); 
			return 0; 
		}
		else if constexpr (std::is_same<T,char*>::value || std::is_same<T,const char*>::value){
			PyObject_SetAttrString(m_p, attr, Py_BuildValue("s",value));
			return 0; 
		}
		else{
			return -1; 
		}
	}

	/**
	 * Set the value of an attribute of the class instance 
	 * @param attr attribute name
	 * @return value of specified type T 
	 */
	template<typename T>
	T getAttr(const char* attr){
		if constexpr (std::is_same<T,double>::value || std::is_same<T,float>::value){
			return PyFloat_AsDouble(PyObject_GetAttrString(m_p, attr));
		}
		else if constexpr (std::is_same<T,int>::value){
			return PyLong_AsLong(PyObject_GetAttrString(m_p, attr));
		}
		else if constexpr (std::is_same<T,PyObject*>::value){
			return PyObject_GetAttrString(m_p, attr);
		}
		return NULL; 
	}
};

// PyObject -> Vector (modified from https://gist.github.com/rjzak/5681680) 
/**
 * Convert a python list or tuple to std::vector
 * Warning: does not work with numpy array 
 */
template<class T> 
std::vector<T> listTupleToVector(PyObject* incoming) {
	std::vector<T> data;
	if (PyTuple_Check(incoming)) {
		for(Py_ssize_t i = 0; i < PyTuple_Size(incoming); i++) {
			PyObject *value = PyTuple_GetItem(incoming, i);
			data.push_back( PyFloat_AsDouble(value) );
		}
	} else {
		if (PyList_Check(incoming)) {
			for(Py_ssize_t i = 0; i < PyList_Size(incoming); i++) {
				PyObject *value = PyList_GetItem(incoming, i);
				data.push_back( PyFloat_AsDouble(value) );
			}
		} else {
			throw std::logic_error("Passed PyObject pointer was not a list or tuple!");
		}
	}
	return data;
}
