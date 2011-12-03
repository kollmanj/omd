#ifndef OMDINTEGRATOR_H
#define OMDINTEGRATOR_H
#include <vector>

namespace OMD
{

	class Model;


class Integrator
{
public:

	Integrator()
	{
	}

	virtual ~Integrator(void)
	{
	}

	virtual std::vector<double> integrate(Model *m, double t0, double t1, bool storeAccels=false )=0;

};
};
#endif