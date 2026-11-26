#pragma once

// Timing
struct TimingType
{
   double time;
   int cycle;
   double time_end;
   int cycle_end;
   double sim_step; 
   double ctrl_step;

   // Constructor
   TimingType()
   {
      time = 0.0;
      cycle = 0;
   }
};
