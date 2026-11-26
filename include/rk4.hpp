#pragma once
#include <Eigen/Core>

template <int Ns, int Nu>
class RungeKutta4
{
   public:
      using State = Eigen::Vector<double, Ns>;
      using Input = Eigen::Vector<double, Nu>;
      
      template <typename Derivative>
      State step(const double dt,
                 const State& state,
                 const Input& input,
                 const Derivative& f)
      {
         m_k1 = f(state, input);
         m_k2 = f(state + dt / 2.0 * m_k1, input);
         m_k3 = f(state + dt / 2.0 * m_k2, input);
         m_k4 = f(state + dt * m_k3, input);

         return state + dt / 6.0 * (m_k1 + 2.0 * m_k2 + 2.0 * m_k3 + m_k4);
      };

   private:
      State m_k1;
      State m_k2;
      State m_k3; 
      State m_k4;
};
