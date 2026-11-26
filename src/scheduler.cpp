#include "scheduler.hpp"

void sim_init(UAV& uav)
{
   // Plant init
   uav.init();
}

void sim_exec(TimingType& timing, 
              UAV& uav)
{
   // Plant integration
   uav.integrate(timing);

   // Navigation

   // Control

}

void sim_print(TimingType& timing, 
               UAV& uav)
{
   uav.print(timing);
}

