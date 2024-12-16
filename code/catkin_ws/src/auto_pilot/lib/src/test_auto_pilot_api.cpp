#include "autopilot.hpp"

#include <stdio.h>
#include <iostream> 

#define UNIT		std::string("\033[94m[unit]\033[0m")
#define SUCCESS 	std::string("\033[92mSUCCESS\033[0m")
#define FAIL		std::string("\033[91mFAIL\033[0m")
#define RESULT(p) 	p ? SUCCESS.c_str() : FAIL.c_str()
// Usage: printf("%s test performed: %s\n", UNIT.c_str(), RESULT(true/false));

/**
 * Test the libautopilot.so shared object 
 */
int main(){

	bool passed = true; // test status 

	// Test module import
	passed = true;  
	if(!ap::init())
		passed = false; 
	printf("%s '%s' module import: %s\n", UNIT.c_str(), MODULE_NAME, RESULT(passed));
	if(!passed) exit(EXIT_FAILURE); 

	// Test autopilot initialization 
	passed = true; 
	try{
		ap::init_autopilot("Position");
	}catch(const std::exception& e){
		passed = false; 
	}
	printf("%s initialization: %s\n", UNIT.c_str(), RESULT(passed));

	// Test autopilot set type  
	passed = true; 
	try{
		ap::set_autopilot_type("Position");
	}catch(const std::exception& e){
		passed = false; 
	}
	printf("%s autopilot set type: %s\n", UNIT.c_str(), RESULT(passed));

	// Test load flight parameters   
	passed = true; 
	try{
		ap::load_flight_parameters(PARAMETER_FILE);
	}catch(const std::exception& e){
		passed = false; 
	}
	printf("%s load_flight_parameters: %s\n", UNIT.c_str(), RESULT(passed));
	
	// Test arm 
	passed = true; 
	try{
		ap::arm(true);
	}catch(const std::exception& e){
		passed = false; 
	}
	printf("%s arm: %s\n", UNIT.c_str(), RESULT(passed));
	
	// Test update_state_with_pose  
	passed = true; 
	try{
		ap::update_state_with_pose(1.1,1.2,1.3,0.1,3.2,0.3, 0.016);
	}catch(const std::exception& e){
		passed = false; 
	}
	printf("%s update_state_with_pose: %s\n", UNIT.c_str(), RESULT(passed));
	
	// Test get_u
	passed = true; 
	try{
		std::vector<double> target = {1.1,1.2,1.3,-1.4};
		ap::get_u(target, true);
	}catch(const std::exception& e){
		passed = false; 
	}
	printf("%s get_u (with triggered disarm): %s\n", UNIT.c_str(), RESULT(passed));

	ap::clean();

	printf("------\n");
	printf("%s test suite status: %s\n", UNIT.c_str(), RESULT(passed));

	return 0;
}
