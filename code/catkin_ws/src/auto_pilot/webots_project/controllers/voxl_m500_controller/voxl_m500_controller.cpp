// File:          auto_pilot.cpp
// Date:          28.01.2021 
// Description:   Example program showing how to make the drone hover (open-loop)
// Author:        Lucas Waelti
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Supervisor.hpp>

#include <string.h>

/**
 * Make the led blink *count* times  
 */
void blink(webots::Robot *robot, webots::LED *led, int count){

  int timeStep = (int)robot->getBasicTimeStep();
  int l = 1, c = 0; 

  while (robot->step(timeStep) != -1){
    led->set(l); 
    if(l == 1) l=2; else l=1; 
    for(int i=0; i<10 && robot->step(timeStep) != -1; i++);
    c++;
    if(c >= 2*count){
      led->set(0); 
      return;
    } 
  }
}

/**
 * DOES NOT WORK
 * Check if fast spinning propeller hit something
 * @param h any of ["1","2","3","4"]
 * @return (0) if no contact, (1) otherwise
 */
bool detect_propeller_hit(webots::Supervisor *robot, std::string h){
  webots::Node *prop = robot->getFromDef("PROP"+h)->getField("fastHelix")->getSFNode(); 
  int count = prop->getNumberOfContactPoints();
  printf("Seeing %d hits\n",prop->getNumberOfContactPoints());
  return count>0; 
}

void set_rotor_speed(webots::Robot *robot, std::string rotor, double vel){
  webots::Motor *motor = robot->getMotor(rotor); 
  motor->setPosition(INFINITY); 
  motor->setVelocity(vel); 
}

int main(int argc, char **argv) {

  // create the Robot instance.
  webots::Supervisor *robot = new webots::Supervisor();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // Make the led blink a few times 
  webots::LED *led = robot->getLED("led"); 
  led->set(0);
  blink(robot,led,4); 
  
  // Switch on a range finder 
  //webots::DistanceSensor *dist = robot->getDistanceSensor("vl53l1x_front");
  //dist->enable(16); 

  // Let all rotor spin at the desired speed 
  const double m=1.37,g=9.81,t1=0.0000103; // t1 is the thrustConstant (c.f. https://cyberbotics.com/doc/reference/propeller)
  double base_vel = sqrt(m*g/(4*t1)); // compute motor velocity to hover 
  double vel = base_vel; 
  if(argc == 2 && strcmp(argv[1],"")){
    vel = std::stof(argv[1]);
    printf("Selected speed: %.3lf rad/s\n",vel);
    while (robot->step(timeStep) != -1){
      set_rotor_speed(robot, "rotor1", vel);
      set_rotor_speed(robot, "rotor2", vel);
      set_rotor_speed(robot, "rotor3", -vel);
      set_rotor_speed(robot, "rotor4", -vel);
    }
  }else{
    printf("Speed for hovering: %.3lf rad/s\n",vel);
 
    int i=0; 
    double speed_offset = 100; 
    while (robot->step(timeStep) != -1){
      if(i<200){
        // idle
        vel = 10; 
        i++; 
      }
      else if(i>=200 && i<300){
        // takeoff
        vel = base_vel*1.01; 
        i++;
      }else if(i>=300 && i<400){
        // turn right
        vel = base_vel; 
        speed_offset = 50; 
        set_rotor_speed(robot, "rotor1", vel+speed_offset);
        set_rotor_speed(robot, "rotor2", vel+speed_offset);
        set_rotor_speed(robot, "rotor3", -vel+speed_offset);
        set_rotor_speed(robot, "rotor4", -vel+speed_offset);
        i++;
        continue; 
      }else if(i>=400 && i<500){
        // turn left
        vel = base_vel; 
        speed_offset = -50; 
        set_rotor_speed(robot, "rotor1", vel+speed_offset);
        set_rotor_speed(robot, "rotor2", vel+speed_offset);
        set_rotor_speed(robot, "rotor3", -vel+speed_offset);
        set_rotor_speed(robot, "rotor4", -vel+speed_offset);
        i++;
        continue;
      }else{
        // hover 
        vel = base_vel; 
      }
      set_rotor_speed(robot, "rotor1", vel);
      set_rotor_speed(robot, "rotor2", vel);
      set_rotor_speed(robot, "rotor3", -vel);
      set_rotor_speed(robot, "rotor4", -vel);
    }
  }
  delete robot;
  return 0;
}
