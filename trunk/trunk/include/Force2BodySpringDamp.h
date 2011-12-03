#ifndef OMD_FORCE2BODYSPRINGDAMP_H
#define OMD_FORCE2BODYSPRINGDAMP_H

#include "force.h"
#include "OMD.h"
#include "BodyRigid.h"

namespace OMD
{
	class Force2BodySpringDamp :
		public Force
	{
	public:
	/// 
	/// Add a force and a torque vector to be summed with other forces and torques and applied to the body
	/// 
	/// @param[in] name: 
	/// 
	/// @param[in] body1: body 1 connected by spring damper
	/// @param[in] body2: body 2 connected by spring damper
	/// @param[in] k: spring constant
	/// @param[in] c: damping constant
	/// @param[in] fl: free length
	/// @param[in] body1Offset:  connection point of spring on 1st body in 1st body coordinates
	/// @param[in] body2Offset:  connection point of spring on 2nd body in 2nd body coordinates
	///
		Force2BodySpringDamp( string const& name,
			BodyRigid *body1,
			BodyRigid *body2,
			double k,
			double c,
			double fl = 0,
			Vect3 body1Offset =Vect3(0.0,0.0,0.0),
			Vect3 body2Offset =Vect3(0.0,0.0,0.0));
	/// 
	/// Add a force and a torque vector to be summed with other forces and torques and applied to the body
	/// 
	/// @param[in] name: 
	/// 
	/// @param[in] body1: body 1 connected by spring damper
	/// @param[in] body2: body 2 connected by spring damper
	/// @param[in] k: spring constant
	/// @param[in] c: damping constant
	/// @param[in] fl: free length
	/// @param[in] body1Offset:  connection point of spring on 1st body in 1st body coordinates
	/// @param[in] body2Offset:  connection point of spring on 2nd body in 2nd body coordinates
	///
		Force2BodySpringDamp( string const& name,
			BodyRigid *body1,
			BodyRigid *body2,
			double k,
			double c,
			double fl,
			vector<double> body1Offset,
			vector<double> body2Offset);
		

		~Force2BodySpringDamp(void);

		virtual void apply(double t);

		double m_k;
		double m_c;
		double m_fl;
		BodyRigid * m_body1;
		BodyRigid * m_body2;
		Vect3 m_body1Offset;
		Vect3 m_body2Offset;
	};
};
#endif