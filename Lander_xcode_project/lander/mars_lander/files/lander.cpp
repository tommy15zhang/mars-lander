// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <iostream>

using namespace std;

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  // INSERT YOUR CODE HERE
    // Magic numbers let's go🤣
    double e, k_h, latitude, p_out, k_p, Delta;

    latitude = position.abs() - MARS_RADIUS;
    k_h = 0.01342;
    k_p = 1;
    Delta = 0.9*GRAVITY*MARS_MASS*(UNLOADED_LANDER_MASS+fuel*FUEL_CAPACITY*FUEL_DENSITY)/(MAX_THRUST*position.abs2());
    attitude_stabilization();
    e = -1*(0.5 + k_h*latitude - velocity*orientation.norm());
    p_out = k_p*e;

    if (p_out <= -Delta){
        throttle = 0;
    }else if (-Delta<p_out<1-Delta){
        throttle = Delta + p_out;
    }else{
        throttle = 1;
    }
    
    
    ofstream fout("/Users/jiahaozhang/Desktop/h_and_ve_data.txt", std::ios_base::app);
    if (fout) { // file opened successfully
        fout << latitude << ' ' << velocity*orientation.norm() << endl;
      }
     else { // file did not open successfully
      cout << "Could not open trajectory file for writing" << endl;
    }
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
    static vector3d previous_position;
    vector3d thr, force, drag_parachute, drag_lander, weight, accele, new_position;
    double d, lander_mass, h;
    
    

    thr = thrust_wrt_world();
    d = atmospheric_density(position);
  //force = weight + thrust + drag
    drag_lander = -0.5*d*DRAG_COEF_LANDER*LANDER_SIZE*LANDER_SIZE*M_PI*velocity.abs2()*velocity.norm();
    drag_parachute = -0.5*d*DRAG_COEF_CHUTE*LANDER_SIZE*LANDER_SIZE*20*velocity.abs2()*velocity.norm();
    //Never write (1/2) in c++, 🤡
    lander_mass = UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY;
    weight = -position.norm()*GRAVITY*MARS_MASS*lander_mass/(position.abs2());
    
    if (parachute_status == DEPLOYED){
        accele = (drag_lander + drag_parachute + weight + thr)/lander_mass;
    }else{
        accele = (drag_lander + weight + thr)/lander_mass;
    }
    
    if (simulation_time == 0.0){
        new_position = position + delta_t*velocity;
        velocity = velocity + delta_t*accele;
    }else{
        new_position = 2*position - previous_position + delta_t*delta_t*accele;
        velocity = (new_position-position)/delta_t;
    }
    previous_position = position;
    position = new_position;
    
    
    

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    // Hyperbolic escape
    position = vector3d(0, 0, MARS_RADIUS*1.9);
    velocity = vector3d(sqrt(GRAVITY*MARS_MASS/(MARS_RADIUS)), 0, 0);
    orientation = vector3d(0,0,90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;
          

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}



