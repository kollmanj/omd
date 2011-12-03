#include "BodyRigid.h"


namespace OMD
{

	//BodyRigid::BodyRigid(	std::string const &name, 
	//			double const &mass,
	//			vector<vector<double>> const &inertia,
	//			vector<double> const &pos, 
	//			vector<double> const &q, 
	//			vector<double> const &vel,
	//			vector<double> const &wl,
	//			bool const &fixed ): Body(name), m_mass(mass), m_fixed(fixed)
	//{
	//	Mat3x3 inertia_;
	//	inertia_ <<	inertia[0][0],	inertia[0][1],	inertia[0][2],
	//				inertia[1][0],	inertia[1][1],	inertia[1][2],
	//				inertia[2][0],	inertia[2][1],	inertia[2][2];
	//	Vect3 pos_;
	//	pos_	<<	pos[0],	pos[1],	pos[2];

	//	Quat q_(q[0],	q[1],	q[2],	q[3]);

	//	Vect3 vel_;
	//	vel_	<<	vel[0],	vel[1],	vel[2];

	//	Vect3 wl_;
	//	wl_		<<	wl[0],	wl[1],	wl[2];

	//	m_pos = pos_;
	//	m_q = q_;
	//	m_vel =vel_;
	//	m_wl = wl_;

	//	m_wldot = Vect3(0,0,0);
	//	m_accel = Vect3(0,0,0);

	//	m_appliedForce = Vect3(0,0,0);
	//	m_appliedTorque = Vect3(0,0,0);

	//	/// initialize kanes method .... TODO site the page number this this shows up on
	//	m_sAhat << 0.,0.,0.,0.,0.,0.;
	//	m_sFhat <<  0.,0.,0.,0.,0.,0.;
	//	m_sIhat <<  0,0,0,0,0,0,
	//				0,0,0,0,0,0,
	//				0,0,0,0,0,0,
	//				0,0,0,0,0,0,
	//				0,0,0,0,0,0,
	//				0,0,0,0,0,0;

	//	m_n_alpha_t_k << 0,0,0;
	//	m_n_a_t_k << 0,0,0;
	//}


	BodyRigid::BodyRigid(	std::string const &name, 
				double const &mass,
				vector<double> const &inertia,
				vector<double> const &pos, 
				vector<double> const &q, 
				vector<double> const &vel,
				vector<double> const &wl,
				bool const &fixed ): Body(name), m_mass(mass), m_fixed(fixed)
	{
		m_inertia <<inertia[0],	inertia[1],	inertia[2],
					inertia[3],	inertia[4],	inertia[5],
					inertia[6],	inertia[7],	inertia[8];
		Vect3 pos_;
		pos_	<<	pos[0],	pos[1],	pos[2];

		Quat q_(q[0],	q[1],	q[2],	q[3]);

		Vect3 vel_;
		vel_	<<	vel[0],	vel[1],	vel[2];

		Vect3 wl_;
		wl_		<<	wl[0],	wl[1],	wl[2];

		m_pos = pos_;
		m_q = q_;
		m_vel =vel_;
		m_wl = wl_;

		m_wldot = Vect3(0,0,0);
		m_accel = Vect3(0,0,0);

		m_appliedForce = Vect3(0,0,0);
		m_appliedTorque = Vect3(0,0,0);

		/// initialize kanes method .... TODO site the page number this this shows up on
		m_sAhat << 0.,0.,0.,0.,0.,0.;
		m_sFhat <<  0.,0.,0.,0.,0.,0.;
		m_sIhat <<  0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0;

		m_n_alpha_t_k << 0,0,0;
		m_n_a_t_k << 0,0,0;
	}

	BodyRigid::BodyRigid(std::string const &name, double const &mass, Mat3x3 const &inertia, 
		Vect3 const &pos, Quat const &q, Vect3 const &vel, Vect3 const &wl, bool const &fixed) : 
	Body(name,pos,q,vel,wl), m_mass(mass), m_inertia(inertia), m_fixed(fixed)
	{
		m_appliedForce = Vect3(0,0,0);
		m_appliedTorque = Vect3(0,0,0);

		/// initialize kanes method .... TODO site the page number this this shows up on
		m_sAhat << 0.,0.,0.,0.,0.,0.;
		m_sFhat <<  0.,0.,0.,0.,0.,0.;
		m_sIhat <<  0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0,
					0,0,0,0,0,0;

		m_n_alpha_t_k << 0,0,0;
		m_n_a_t_k << 0,0,0;
	}


	BodyRigid::~BodyRigid(void)
	{
	}

	void BodyRigid::forceAccumReset()
	{
        m_appliedForce = Vect3(0,0,0);
		m_appliedTorque = Vect3(0,0,0);
	}

	/// solve for translational Acceleration with No Constraints
	Vect3 BodyRigid::solveTransAccel(bool storeAccels)
	{
		/// this may be an intermediate integrator step so we may not want to save accels.
		Vect3 accel;
		accel(0) = m_appliedForce(0)/m_mass;
		accel(1) = m_appliedForce(1)/m_mass;
		accel(2) = m_appliedForce(2)/m_mass;

		if (storeAccels)
		{
			m_accel = accel;;
		}

		return accel;
	}
	/// solve for rotational Acceleration with No Constraints
	Vect3 BodyRigid::solveRotAccel(bool storeAccels)
	{
		/// page 292 Nikravesh equation 11.18
	    Mat3x3 wskew = skew(m_wl);
		/// TODO, see if line below is faster, or more accurate
	    //m_wldot = m_inertia.colPivHouseholderQr().solve(m_appliedTorque-wskew*m_inertia*m_wl);
		//m_wldot = m_inertia.ldlt().solve(m_appliedTorque-wskew*m_inertia*m_wl);

		Mat3x3 Iinv = getInverse(m_inertia);
		Vect3 wldot = Iinv * (m_appliedTorque-wskew*m_inertia*m_wl);

		if (storeAccels)
		{
			m_wldot = wldot;
		}

		return wldot;
	}

	Vect3 BodyRigid::getVelocityGlobal(Vect3 const &localCoordinates)
	{
		return m_vel + m_q * m_wl.cross(localCoordinates);
	}

	void BodyRigid::forceAccum(Vect3 const &f, bool isInLocal)
	{
		if(isInLocal)
		{
			m_appliedForce += m_q*f;
		}
		else
		{
			m_appliedForce += f;
		}
	}

	void BodyRigid::torqueAccum(Vect3 const &t, bool isInLocal)
	{
		if(isInLocal)
		{
			m_appliedTorque += t;
		}
		else
		{
			/// TODO, make sure this is how to convert from global to local
			m_appliedTorque += m_q.inverse() * t;
		}
	}

	void BodyRigid::forceAccum(Vect3 const &f, bool isInLocal, Vect3 const &forceAppLocal)
	{
		Vect3 force; Vect3 torque;
		if(isInLocal)
		{
			force = m_q*f;
			torque = forceAppLocal.cross(f);
		}
		else
		{
			force = f;
			torque = forceAppLocal.cross(m_q.inverse()*f);
		}

		m_appliedForce += force;
		m_appliedTorque += torque;
	}

	void BodyRigid::forceAccumGlobal(Vect3 const &globalForce, Vect3 const &globalPnt)
	{
	    Vect3 fLocal = m_q.inverse() * globalForce;
	    Vect3 pntLocal = m_q.inverse() * (globalPnt-m_pos);
	    Vect3 tLocal = pntLocal.cross(fLocal);
		//std::cout << "dfadsfads" << std::endl;
	    m_appliedForce +=  globalForce;
		m_appliedTorque += tLocal;
	}

	void BodyRigid::forceAndTorqueAccum(Vect3 const &f, bool forceInLocal, Vect3 const &forceAppLocal, Vect3 const &t, bool torqueInLocal)
	{
		Vect3 force; Vect3 torque;
		if(forceInLocal)
		{
			force = m_q*f;
			torque = forceAppLocal.cross(f);
		}
		else
		{
			force = f;
			torque = forceAppLocal.cross(m_q.inverse()*f);
		}

		if(torqueInLocal)
		{
			torque += t;
		}
		else
		{
			torque += m_q.inverse()*t;
		}

		m_appliedForce += force;
		m_appliedTorque += torque;

	}

	void BodyRigid::dynamicAccumReset()
	{
//	    std::cout << "before: " << m_sIhat << std::endl;
//	    std::cout << "inertia: " << m_INERTIA << std::endl;
		Mat3x3 sIhat_m00 = m_inertia;
		Mat3x3 sIhat_m01;
		sIhat_m01 <<	0.,0.,0.,
						0.,0.,0.,
						0.,0.,0.;

		Mat3x3 sIhat_m10;
		sIhat_m10 <<	0.,0.,0.,
						0.,0.,0.,
						0.,0.,0.;

		Mat3x3 sIhat_m11;

		sIhat_m11 <<	m_mass,	0.,		0.,
						0.,		m_mass,	0.,
						0.,		0.,		m_mass;

		m_sIhat << sIhat_m00,sIhat_m01,sIhat_m10,sIhat_m11;
//		std::cout << "after: " << m_sIhat << std::endl;
//		std::cout << " lr after " << sIhat_m11 << std::endl;
		//inertia force and torque

			    /// equation 21 page 47 Anderson
		Vect3 ang(-m_inertia*m_n_alpha_t_k-m_wl.cross(m_inertia*m_wl));
		/// equation 19 page 47 Anderson
		Vect3 trans(m_n_a_t_k*(-m_mass));

		m_sFhat << ang+m_appliedTorque, trans+m_q.inverse()*m_appliedForce;
	}


	Vect3 BodyRigid::getPosition(Vect3 const &offset) const
	{
		return m_pos + m_q*offset;
	}

	Vect3 BodyRigid::getPointInLocal(Vect3 const &pntGlobal) const
	{
	    return m_q.inverse()*(pntGlobal-m_pos);
	}
}