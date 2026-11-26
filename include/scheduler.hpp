#pragma once
#include "typedefs.hpp"
#include "uav.hpp"

// Scheduler header to manage main calls
void sim_init(UAV& uav);
void sim_exec(TimingType& timing,
              UAV& uav);
void sim_print(TimingType& timing, 
               UAV& uav);
