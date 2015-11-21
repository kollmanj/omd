#ifndef OMD_FORCE1BODY_H
#define OMD_FORCE1BODY_H

#include "Force.h"
#include "OMD.h"
#include "BodyRigid.h"

namespace OMD
{
#include "Force.h"
/// \brief
/// A Force and Torque to be applied to a body that can be in local or global coordinates
///
///	@author John Kollman
///
///
class Force1Body :
	public Force
{
public:
	Force1Body(std::string const &name, BodyRigid * body, Vect3 const &f, Vect3 const &forceLocation=Vect3(0,0,0),
							bool const &forceIsLocal=true, Vect3 const &t=Vect3(0,0,0), bool const &torqueIsLocal=true);
	Force1Body(std::string const &name, BodyRigid * body, std::vector<double> const &f, std::vector<double> const &forceLocation,
							bool const &forceIsLocal, std::vector<double> const &t, bool const &torqueIsLocal=true);
	~Force1Body(void);

	/// set the location on the body, in local body coordinates, at which the force acts
	/// @param[in] body coordinate of the foce at which the force acts
	///
	/// @return Nothing
	///
	void setFoceLocation(Vect3 const &l){m_forceCoord = l;};

	/// set the force to be in local body coordinates
	void setForceLocal(){m_forceIsLocal=true;};
	/// set the torque to be in local body coordinates
	void setTorqueLocal(){m_torqueIsLocal=true;};
	/// set force to be in global coordinates
	void setForceGlobal(){m_forceIsLocal=false;};
	/// set torque to be in global coordinates
	void setTorqueGlobal(){m_torqueIsLocal=false;};

	void setTorque(double x, double y, double z){m_torque=Vect3(x,y,z);};
	void setForce(double x, double y, double z){m_force=Vect3(x,y,z);};

	/// vector of bodies on which to apply force and or torque
	BodyRigid * m_body;
	/// if true the force is local otherwise global
	bool m_forceIsLocal;
	/// if true the torque is local otherwise global
	bool m_torqueIsLocal;
	/// force to be applied to body, m_foceLocal determines whether it is in local body coordinates or global
	Vect3 m_force;
	/// torque to be applied to body, m_torqueLocal determines whether is in local body coordinates or global
	Vect3 m_torque;
	/// body local coordinates of the point at which to apply force
	Vect3 m_forceCoord;

protected:
	void apply(double t);
};
};
#endif
