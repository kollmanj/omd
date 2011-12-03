#ifndef OMD_BODY_H
#define OMD_BODY_H

#include <string>
#include "OMD.h"
#include <vector>

namespace OMD
{
/// \brief
/// Virtual Base class representing a Body
///
///	@author John Kollman
///
///
class Body
{
public:

	Body(std::string const &name )
	{
		m_name = name;
		m_pos << 0,0,0;
		m_q = Quat(1,0,0,0);
		m_vel << 0,0,0;
		m_wl << 0,0,0;

		m_wldot = Vect3(0,0,0);
		m_accel = Vect3(0,0,0);

	}

	Body(std::string const &name, Vect3 const &pos, Quat const &q, Vect3 const &vel = Vect3(0,0,0), Vect3 const &wl = Vect3(0,0,0) )
	{
		m_name = name;
		m_pos = pos;
		m_q = q;
		m_vel =vel;
		m_wl = wl;

		m_wldot = Vect3(0,0,0);
		m_accel = Vect3(0,0,0);
	}
	//Body(std::string const &name, Vect3 const &pos, Mat3x3 const &rot = Mat3x3(Quat(1,0,0,0)), Vect3 const &vel = Vect3(0,0,0), Vect3 const &wl = Vect3(0,0,0) )
	Body(std::string const &name, Vect3 const &pos, Mat3x3 const &rot, Vect3 const &vel = Vect3(0,0,0), Vect3 const &wl = Vect3(0,0,0) )
	{
		m_name = name;
		m_pos = pos;
		m_q = Quat(rot);
		m_vel =vel;
		m_wl = wl;

		m_wldot = Vect3(0,0,0);
		m_accel = Vect3(0,0,0);
	}

	void setAcceleration(Vect3 const &a)
	{
		m_accel = a;
	}

	void setLocalAngularAccel(Vect3 const &wldot)
	{
		m_wldot = wldot;
	}

	virtual Vect3 getVelocityGlobal(Vect3 const &localCoordinates)=0;
	Vect3 getAngularAccelerationGlobal(){return m_q*m_wldot;};
	///
	/// get the velocity of the body in global coordinates at the center of mass
	///
	/// @return vector (x,y,z) describing velocity
	///
	Vect3 getVelocityGlobal(){return m_vel;};
	Vect3 getVelocityLocal(){return m_q.inverse()*m_vel;};
	Vect3 getAngularVelocityLocal(){return m_wl;};
	Vect3 getAngularVelocityGlobal(){return m_q*m_wl;};


	Quat getQuatDot() const
	{
		return OMD::getQDot(m_q,m_wl);
	}

	///
	/// Set rotation matrix
	///
	/// @return Nothing
	///
	void setRot( Mat3x3 const &rot ){m_q=Quat(rot);};
	/////
	///// Get rotation matrix
	/////
	///// @return rot matrix
	/////
	Mat3x3 getRot(  ){return m_q.toRotationMatrix();};
	/////
	///// Get derivative of the rotation matrix wrt time
	/////
	///// @return derivative of rot matrix wrt time
	/////
	Mat3x3 getRotDot() const
	{
		// equation 6.56 page 169 of Nikravesh
		Mat3x4 Lmat = L(m_q);
        Quat qDot = getQuatDot();
        Mat3x4 Gdot = G(qDot);
        ////// equation 6.56 page 169
        return ( 2.0 * Gdot ) * Lmat.transpose();
	};
	
	///
	/// 
	Vect3 getVectorDot(Vect3 const &v) const {return getRotDot()*v;};

	Vect3 localCoordinates(Vect3 const &globalCoordinates){return m_q.inverse()*(globalCoordinates - m_pos);};
	Vect3 globalCoordinates(Vect3 const &localCoordinates){return m_q*localCoordinates + m_pos;};

	void setVelLocal(Vect3 const &a){m_vel=m_q * a;};
	void setAngularVelocity(Vect3 const &a){m_wl=a;};

	/// body name
	std::string m_name;
	/// position of the body
	Vect3 m_pos;
	/// orientation 
	Quat m_q;
	/// velocity in global
	Vect3 m_vel;
	/// angular velocity in body coordinates, wl as in omega l for local
	Vect3 m_wl; 
	/// angular acceleration, may or may not be used
	Vect3 m_wldot;
	/// acceleration in global, may or may not be used
	Vect3 m_accel;

	std::vector<double> getXAxis() const
	{
		Vect3 xa = m_q*Vect3(1,0,0);
		std::vector<double> out;
		out.push_back(xa.x());
		out.push_back(xa.y());
		out.push_back(xa.z());
		return out;
	};

	std::vector<double> getYAxis() const
	{
		Vect3 ya = m_q*Vect3(0,1,0);
		std::vector<double> out;
		out.push_back(ya.x());
		out.push_back(ya.y());
		out.push_back(ya.z());
		return out;
	};

	std::vector<double> getZAxis() const
	{
		Vect3 za = m_q*Vect3(0,0,1);
		std::vector<double> out;
		out.push_back(za.x());
		out.push_back(za.y());
		out.push_back(za.z());
		return out;
	};


	double getX() const {return m_pos.x();};
	double getY() const {return m_pos.y();};
	double getZ() const {return m_pos.z();};
	double getXd() const {return m_vel.x();};
	double getYd() const {return m_vel.y();};
	double getZd() const {return m_vel.z();};
	double getE0() const {return m_q.w();};
	double getE1() const {return m_q.x();};
	double getE2() const {return m_q.y();};
	double getE3() const {return m_q.z();};
	double getWlx() const {return m_wl.x();};
	double getWly() const {return m_wl.y();};
	double getWlz() const {return m_wl.z();};
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	virtual ~Body(void)
	{
	}
};
};
#endif