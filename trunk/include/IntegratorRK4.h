#ifndef OMDINTEGRATORRK4_H
#define OMDINTEGRATORRK4_H

#include "Integrator.h"
#include "Model.h"

namespace OMD
{
class IntegratorRK4 :
	public Integrator
{
public:
	IntegratorRK4();
	~IntegratorRK4(void);

	virtual std::vector<double> integrate(Model *m, double t0, double t1, bool storeAccels = false );

private:
   double h;
   std::vector<double> k1;
   std::vector<double> k2;
   std::vector<double> k3;
   std::vector<double> k4;
   std::vector<double> y0;
   std::vector<double> y1;

};
};
#endif