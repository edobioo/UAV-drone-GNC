#include <iostream>
#include <iomanip>

#include "utils.hpp"
#include "typedefs.hpp"
#include "scheduler.hpp"

int main(void)
{

   // Declarations
   TimingType timing;
   UAV uav;

   try
   {
      // Load timing config
      load_timing("inputs/timing.txt", &timing);   

      // Init scheduler
      sim_init(uav);

      // Simulation loop
      while (timing.cycle < timing.cycle_end)
      {
         // Exec
         sim_exec(timing, uav);

         // Print
         sim_print(timing, uav);

         // Update time
         timing.cycle++;
         timing.time += timing.sim_step;
      }

      std::cout << std::fixed << std::setprecision(2);
      std::cout << "Simulation ended after " << timing.time << " s!\n";
   }
   catch (const std::exception& error)
   {
      std::cerr << "Error: " << error.what() << "\n";
      return EXIT_FAILURE;
   }
   
   return EXIT_SUCCESS;
}
