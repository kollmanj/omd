#include "Force2BodySpringDamp.h"

namespace OMD
{
	Force2BodySpringDamp::Force2BodySpringDamp( string const& name,
			BodyRigid *body1,
			BodyRigid *body2,
			double k,
			double c,
			double fl,
			vector<double> body1Offset,
			vector<double> body2Offset): Force(name), m_k(k), m_c(c), m_fl(fl), m_body1(body1), m_body2(body2)
	{
		m_body1Offset << body1Offset[0], body1Offset[1], body1Offset[2];
		m_body2Offset << body2Offset[0], body2Offset[1], body2Offset[2];
	}

	Force2BodySpringDamp::Force2BodySpringDamp(string const& name,
		BodyRigid *body1,
		BodyRigid *body2,
		double k,
		double c,
		double fl,
		Vect3 body1Offset,
		Vect3 body2Offset): Force(name), m_k(k), m_c(c), m_fl(fl), m_body1(body1), m_body2(body2),
		m_body1Offset(body1Offset), m_body2Offset(body2Offset)
	{
	}

	Force2BodySpringDamp::~Force2BodySpringDamp(void)
	{
	}

	void Force2BodySpringDamp::apply(double t)
	{
		// pt1 on body body 1 & pt2 on body 2
		Vect3 pt1Pos = m_body1->getPosition(m_body1Offset);
		Vect3 pt1Vel = m_body1->getVelocityGlobal(m_body1Offset);
		Vect3 pt2Pos = m_body2->getPosition(m_body2Offset);
		Vect3 pt2Vel = m_body2->getVelocityGlobal(m_body2Offset);

		// distance between 2 points in global
		Vect3 dist = pt2Pos-pt1Pos;
		// Relative Velocity
		Vect3 revVel = pt2Vel - pt1Vel;

		// just to have meaningfull names
		Vect3 normalForceVect = dist;
		//normailze normalForceVector in place and get magnitude
		double dist_mag = magnitude(normalForceVect);
		normalForceVect.normalize();

		double springForceMag = (dist_mag-m_fl)*m_k;

		double velAlongSpring = normalForceVect.dot(revVel);
		double dampForceMag = velAlongSpring * m_c;

		double totalForceMag = springForceMag + dampForceMag;

		Vect3 totalForceVect = normalForceVect * totalForceMag;

		m_body1->forceAccum(totalForceVect,false,m_body1Offset);
		m_body2->forceAccum(-totalForceVect,false,m_body2Offset);

	}

}