#include "Force1Body.h"

namespace OMD
{
	Force1Body::Force1Body(std::string const &name, BodyRigid * body, Vect3 const &f, Vect3 const &forceLocation,
							bool const &forceIsLocal, Vect3 const &t, bool const &torqueIsLocal): 
								Force(name), m_body(body), m_force(f), m_forceCoord(forceLocation), m_forceIsLocal(forceIsLocal),
								m_torque(t), m_torqueIsLocal(torqueIsLocal)
	{

	}

	Force1Body::Force1Body(std::string const &name, BodyRigid * body, vector<double> const &f, vector<double> const &forceLocation,
							bool const &forceIsLocal, vector<double> const &t, bool const &torqueIsLocal): 
							m_forceIsLocal(forceIsLocal), m_body(body), m_torqueIsLocal(torqueIsLocal), Force(name)
	{
		m_force << f[0],f[1],f[2];
		m_forceCoord << forceLocation[0], forceLocation[1], forceLocation[2];
		m_torque << t[0], t[1], t[2];
	}


	Force1Body::~Force1Body(void)
	{
	}

	void Force1Body::apply(double t)
	{
		m_body->forceAndTorqueAccum(m_force,m_forceIsLocal,m_forceCoord,m_torque,m_torqueIsLocal);
	}


}
