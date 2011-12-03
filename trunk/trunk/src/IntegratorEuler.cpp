#include "IntegratorEuler.h"

namespace OMD
{
IntegratorEuler::IntegratorEuler(void)
{
}

IntegratorEuler::~IntegratorEuler(void)
{
}

std::vector<double> IntegratorEuler::integrate(Model *m, double t0, double t1, bool storeAccels )
{
   h = t1 - t0;
   k1.clear();
   y0.clear();
   y1.clear();

   y0 = m->getState();
   // need to run solve becuase the dots are from before the last setState command
   // if the accelerations are to be stored do it now at t0
   k1 = m->solve(t0,storeAccels);

   double s;
   int i = 0;
   std::vector<double>::iterator it;
   for (it = k1.begin(); it < k1.end(); ++it )
   {
      s =  (*it);
      y1.push_back(y0[i]+h*s);
      i++;
   }

   m->setState(y1);
   return y1;

}

}