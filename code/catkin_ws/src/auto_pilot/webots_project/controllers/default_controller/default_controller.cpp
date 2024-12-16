// File:          default_controller.cpp
// Date:          20.01.2021 
// Description:   Example program showing how to control the rotors of the drone 
// Author:        Lucas Waelti
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

void set_rotor_speed(Robot *robot, std::string rotor, double vel){
  webots::Motor *motor = robot->getMotor(rotor); 
  motor->setPosition(INFINITY); 
  motor->setVelocity(vel); 
}

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // Let all rotor spin at the desired speed 
  const double vel = 5850.0;

  set_rotor_speed(robot, "rotor1", vel);
  set_rotor_speed(robot, "rotor2", -vel);
  set_rotor_speed(robot, "rotor3", vel);
  set_rotor_speed(robot, "rotor4", -vel);

  while (robot->step(timeStep) != -1) { };

  delete robot;
  return 0;
}
