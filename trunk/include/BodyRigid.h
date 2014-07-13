#ifndef OMD_BODYRIGID_H
#define OMD_BODYRIGID_H 

#include "Body.h"
#include <vector>


namespace OMD
{
/// \brief
/// Represents a Rigid Body
///
///	@author John Kollman
///
///
class BodyRigid :
	public Body
{
public:
	///
	/// Constructor without eigen entities
	///
	BodyRigid(	std::string const &name, 
				double const &mass,
				vector<double> const &inertia,
				vector<double> const &pos, 
				vector<double> const &q, 
				vector<double> const &vel,
				vector<double> const &wl,
				bool const &fixed = false);
	/////
	///// Constructor without eigen entities
	/////
	//BodyRigid(	std::string const &name, 
	//			double const &mass,
	//			vector<vector<double>> const &inertia,
	//			vector<double> const &pos, 
	//			vector<double> const &q, 
	//			vector<double> const &vel,
	//			vector<double> const &wl,
	//			bool const &fixed = false);
	///
	/// Constructor with eigen entities
	///
	BodyRigid(	std::string const &name, 
				double const &mass,
				Mat3x3 const &inertia,
				Vect3 const &pos = Vect3(0,0,0), 
				Quat const &q = Quat(1,0,0,0), 
				Vect3 const &vel = Vect3(0,0,0),
				Vect3 const &wl = Vect3(0,0,0),
				bool const &fixed = false);
	~BodyRigid(void);

	///
	/// Set if body is fixed, i.e. immovable
	///
	/// @param[in] b fixed or not
	/// @return Nothing
	///
	void setFixed(bool const &b=true){m_fixed = b;};
	///
	/// See if body is fixed, i.e. immovable
	///
	/// @return true or fasle
	///
	bool isFixed() const {return m_fixed;};
	///
	/// get the velocity of the body in global coordinates at the center of mass
	///
	/// @param[in] localCoordinates, the location on the body, in local body coordinates, at which the velocity is measured
	///
	/// @return vector (x,y,z) describing velocity
	///
	virtual Vect3 getVelocityGlobal(Vect3 const &localCoordinates);
	Vect3 getVelocityGlobal(){return m_vel;};
	void setPosition(Vect3 const &v){m_pos = v;};

	

	/// solve for translational Acceleration with No Constraints
	Vect3 solveTransAccel(bool storeAccels=false);
	/// solve for rotational Acceleration with No Constraints
	Vect3 solveRotAccel(bool storeAccels=false);

	/// 
	/// Add a force vector to be summed with other forces and applied to the body
	/// 
	/// @param[in] f: the force to be applied to the body at the center of mass, i.e. no resulting torque
	/// @param[in] isInLocal: true if force is expressed in local body coordinates false if in global
	///
	void forceAccum(Vect3 const &f, bool isInLocal = true);
	/// 
	/// Add a force vector to be summed with other forces and applied to the body, 
	/// if forceAppLocal is not at the center of mass there will be a resulting torque that will be computed
	/// and added to the body
	/// 
	/// @param[in] f: the force to be applied to the body at the center of mass, i.e. no resulting torque
	/// @param[in] isInLocal: true if force is expressed in local body coordinates false if in global
	/// @param[in] forceAppLocal: location in body coordintes of the force
	///
	void forceAccum(Vect3 const &f, bool isInLocal, Vect3 const &forceAppLocal);
	void forceAccumGlobal(Vect3 const &globalForce, Vect3 const &globalPnt);
	/// 
	/// Add a torque vector to be summed with other forces and applied to the body
	/// 
	/// @param[in] t: the torque to be applied to the body
	/// @param[in] isInLocal: true if torque is expressed in local body coordinates false if in global
	///
	void torqueAccum(Vect3 const &t, bool isInLocal = true);
	/// 
	/// Add a force and a torque vector to be summed with other forces and torques and applied to the body
	/// 
	/// @param[in] f: force applied to body
	/// 
	/// @param[in] t: the torque to be applied to the body
	/// @param[in] forceInLocal: true if force is expressed in local body coordinates false if in global
	/// @param[in] forceAppLocal: force application point in body coordinates
	/// @param[in] t: torque applied to body in body local coordinates
	/// @param[in] torqueInLocal: true if torque is expressed in local body coordines false if in global
	///
	void forceAndTorqueAccum(Vect3 const &f, bool forceInLocal, Vect3 const &forceAppLocal, Vect3 const &t, bool torqueInLocal);

	void forceAccumReset();

	Vect3 getPosition(Vect3 const &offset) const;
	Vect3 getPointInLocal(Vect3 const &pntGlobal) const;
	Vect3 getAppliedForce(){return m_appliedForce;};
	Vect3 getAppliedTorque(){return m_appliedTorque;};

	void dynamicAccumReset();

	/// defines mass
	double m_mass;
	/// Matrix 3x3 defining inertia in local coordinates
	Mat3x3 m_inertia;
	/// set to true of body is fixed to ground
	bool m_fixed;

	/// Vector to sum the forces applied in global coordinates
	Vect3 m_appliedForce; // global
	
	/// Vector to sum the torque applied in local coordinates
	Vect3 m_appliedTorque; //local

	void defineContact(double stiff, double damp, double frict, double thresh);
	// data for contact
	double m_stiff;
	double m_damp;
	double m_frict;
	double m_thresh;

protected:
	friend class Joint1DOF;
	friend class JointRevolute;
	friend class JointTranslational;
	friend class Branch;
	/// initialize kanes method .... TODO site the page number this this shows up on
	Vect6 m_sAhat;  /// TODO: this should probably not be a member variable of a Body
	// Defined in equation 83 An Order n Formulation nfor the Motion Simulation of General
	// ... K.S. Anderson
	// Inertia Mass Matrix + effects of children
	Mat6x6 m_sIhat; /// TODO: this should probably not be a member variable of a Body
	Vect6 m_sFhat; /// TODO: this should probably not be a member variable of a Body

	/// Found in equation 21 [Anderson  p47] for revolute joint \n
    /// equation notation: \f$ ^{N} \alpha _t ^k \f$
	Vect3 m_n_alpha_t_k;   ///// TODO: get this to acceleration
	/// Found in equation 19 [Anderson  p47] for revolute joint \n
    /// equation notation: \f$ ^{N} a _t ^k \f$
	Vect3 m_n_a_t_k; ///// TODO: get this to acceleration
};

};
#endif