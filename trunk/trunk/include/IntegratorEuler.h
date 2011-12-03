#ifndef OMDINTEGEULER_H
#define OMDINTEGEULER_H

#include "Integrator.h"
#include "Model.h"

namespace OMD
{
class IntegratorEuler :
	public Integrator
{
public:
	IntegratorEuler(void);
	~IntegratorEuler(void);

	virtual std::vector<double> integrate(Model *m, double t0, double t1, bool storeAccels=false );

private:
   double h;
   std::vector<double> k1;
   std::vector<double> y0;
   std::vector<double> y1;
};
};
#endif