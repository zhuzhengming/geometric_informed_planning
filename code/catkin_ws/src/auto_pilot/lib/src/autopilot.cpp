// Python C API reference manual: https://docs.python.org/3.6/c-api/index.html#c-api-index
// Building values reference: https://docs.python.org/3.6/c-api/arg.html?highlight=py_buildvalue#c.Py_BuildValue 

#include "pythonutils.hpp"
#include "autopilot.hpp"

#include <stdio.h>
#include <iostream>

static CPyInstance pInstance; 	 // Generic python initialization 
static CPyObject pModule = NULL; // Module pointer, release at the end of program 
static CPyObject pAutoPilot = NULL; // AutoPilot instance

bool ap::init(){

	// Import the module 
	pModule = PyImport_ImportModule(MODULE_NAME);  
	if(!pModule.is()) {
		PyErr_Print();
		std::cerr << "Cannot load module.\n";
		return false; 
	}

	// Get items belonging to the module 
	CPyObject pDict = PyModule_GetDict(pModule);
	if (pDict.getObject() == nullptr) {
		PyErr_Print();
		std::cerr << "Cannot get the dictionary.\n";
		return false; 
	}

	// Get the class instance 
	pAutoPilot = PyDict_GetItemString(pDict, "autopilot");
	if(pAutoPilot.getObject() == nullptr){
		std::cerr << "Cannot get the Python class instance.\n";
		return false; 
	}

	return true;
}

void ap::init_autopilot(std::string autopilot_type, std::string path){
	PyObject_CallMethod(pAutoPilot, "init", "ss", autopilot_type.c_str(), path.c_str());
}

void ap::set_autopilot_type(std::string type){
	pAutoPilot.setAttr<std::string>("autopilot_control_type",type);
}

void ap::load_flight_parameters(std::string file_path){
	PyObject_CallMethod(pAutoPilot, "load_flight_parameters", "s", file_path.c_str());
}

void ap::arm(bool cmd){
	PyObject_CallMethod(pAutoPilot, "arm", "b", cmd);
}

void ap::update_state_with_pose(double x, double y, double z, double roll, double pitch, double yaw, double dt){
	CPyObject pState = pAutoPilot.getAttr<PyObject*>("state");
	PyObject_CallMethod(pState, "update_with_pose_exp", "ddddddd", x,y,z,roll,pitch,yaw,dt);
}

std::vector<double> ap::get_u(const std::vector<double>& target, bool superimpose){
	CPyObject pValue = PyObject_CallMethod(pAutoPilot, "get_u", "[dddd]b", target[0],target[1],target[2],target[3],superimpose); 
	return listTupleToVector<double>(pValue);   
}

void ap::clean(){
	pModule.release();
	pAutoPilot.release();
	pInstance.~CPyInstance();
}





////////////
// Legacy //
////////////

/**
 * This function creates an instance of the autopilot class. 
 * It uses the auto_pilot module directly. 
 * 
 * To test the interface, an excessive attitude is set to trigger a disarm action. 
 */
void autopilot_interaction_demo(){

	CPyInstance pInst;

	// Make sure we own the GIL
	PyGILState_STATE state = PyGILState_Ensure();

	// Load the module 
	CPyObject pMod = PyImport_ImportModule("auto_pilot");
	printf("Imported module\n");

	// Get items belonging to the module 
	CPyObject pDict = PyModule_GetDict(pMod);
	if (pDict.getObject() == nullptr) {
		PyErr_Print();
		std::cerr << "Fails to get the dictionary.\n";
	}

	// Builds the name of a callable class
	CPyObject pClass = PyDict_GetItemString(pDict, "AutoPilot");
	if (pClass.getObject() == nullptr) {
		PyErr_Print();
		std::cerr << "Fails to get the Python class.\n";
	}

	// Creates an instance of the class
	CPyObject pObject = PyObject_CallObject(pClass, nullptr);
	if(pObject.getObject() == nullptr){
		std::cout << "Cannot instantiate the Python class" << std::endl;
	}

	// Call the init method
	PyObject_CallMethod(pObject, "init", "ss", "Position", PARAMETER_FILE); 
	// alternatively 
	//CPyObject args(Py_BuildValue("ss", "Position",PARAMETER_FILE)); 
	//PyObject_Call(PyObject_GetAttrString(pObject, "init"), args, NULL);

	// Arm the autopilot
	CPyObject pValue = PyObject_CallMethod(pObject, "arm", "b", true);

	// Update the state
	//CPyObject pState = PyObject_GetAttrString(pObject,"state");
	// alternatively
	CPyObject pState = pObject.getAttr<PyObject*>("state");
	PyObject_CallMethod(pState, "update_with_pose_exp", "ddddddd", 1.1,1.2,1.3,2.1,2.2,2.3, 0.016);

	// Call the get_u method
	pValue = PyObject_CallMethod(pObject, "get_u", "[dddd]", 1.1,1.2,1.3,-1.4); 
	std::cout << "Returned value has type: " << pValue.getObject()->ob_type->tp_name << std::endl; // list
	std::vector<double> motor_commands = listTupleToVector<double>(pValue);  
	std::cout << "Motor commands: " << motor_commands[0] << "," << motor_commands[1] << "," << motor_commands[2] << "," << motor_commands[3] << std::endl; 

	// Clean up code 
	pMod.release(); 
	PyGILState_Release(state);
	pInst.~CPyInstance();
	printf("Instantiation test done\n");
	exit(EXIT_SUCCESS);
}