#include "utils.hpp"
#include <fstream>
#include <iostream>
#include <limits>
#include <cstdlib>
#include <stdexcept>
#include <cmath>

// Read timing file
void load_timing(const std::string& filename, TimingType* timing)
{
   std::ifstream file(filename);
   
   if (!file.is_open())
      throw std::runtime_error("Can't open file: " + filename);

   // Skip header
   file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

   if (!(file >> timing->time_end >> timing->sim_step >> timing->ctrl_step))
      throw std::runtime_error("Can't read file: " + filename);

   timing->cycle_end = static_cast<int>(std::round(timing->time_end / 
                                                   timing->sim_step));
}
