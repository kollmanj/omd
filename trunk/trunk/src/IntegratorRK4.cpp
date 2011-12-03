#include "IntegratorRK4.h"

namespace OMD
{
IntegratorRK4::IntegratorRK4()
{
}

IntegratorRK4::~IntegratorRK4(void)
{
}

std::vector<double> IntegratorRK4::integrate(Model *m, double t0, double t1, bool storeAccels )
{
   h = t1 - t0;
   k1.clear();
   k2.clear();
   k3.clear();
   k4.clear();
   y0.clear();
   y1.clear();

   y0 = m->getState();
   // need to run solve becuase the dots are from before the last setState command
   // if the accelerations are to be stored do it now at t0
   k1 = m->solve(t0, storeAccels);

   std::vector< double > temp_state;

   std::vector< double >::iterator it;
   int i = 0;
	for (it = k1.begin(); it < k1.end(); ++it )
	{
      temp_state.push_back(y0[i]+0.5*h*(*it));
      i++;
   }

   m->setState(temp_state);
   // need to run solve becuase the dots are from before the last setState command
   k2 = m->solve(t0+0.5*h);

   temp_state.clear();
   i = 0;
	for (it = k2.begin(); it < k2.end(); ++it )
	{
      temp_state.push_back(y0[i]+0.5*h*(*it));
      i++;
   }

   m->setState(temp_state);
   // need to run solve becuase the dots are from before the last setState command
   k3 = m->solve(t0+0.5*h);

   temp_state.clear();
   i = 0;
	for (it = k3.begin(); it < k3.end(); ++it )
	{
      temp_state.push_back(y0[i]+h*(*it));
      i++;
   }

   m->setState(temp_state);    
   // need to run solve becuase the dots are from before the last setState command
   k4 = m->solve(t0+h);

   double s;
   i = 0;
	for (it = k4.begin(); it < k4.end(); ++it )
	{
      s = (1./6.)* (k1[i] + 2.*k2[i] + 2.*k3[i] + (*it));
      y1.push_back(y0[i]+h*s);
      i++;
   }
 
   m->setState(y1);
   return y1;
}

}