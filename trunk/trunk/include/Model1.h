#ifndef OMD_MODEL1_H
#define OMD_MODEL1_H

#include "model.h"
#include "BodyRigid.h"
#include "Force1Body.h"
#include "ForceGravity.h"
#include "IntegratorRK4.h"
#include "IntegratorEuler.h"
#include <vector>
#include "Force2BodySpringDamp.h"
#include "ForceRevJnt.h"
#include "OMDConfig.h"
#ifdef USE_BULLET  // only include this if using bullet
#include "ForceContact.h"
#endif
#ifdef USE_OGRE	// include for ogre
#include "ForceCollisionDynWorld.h"
#endif

namespace OMD
{
	/// \brief
	/// Class representing a Model containing only bodies and forces, but no joints
	///
	///	@author John Kollman
	///
	///
	class Model1 :
		public Model
	{
	public:
		Model1(void);
		~Model1(void);
		///
		/// Solve the system for the derivates of the states
		///
		/// @param[in]	t time
		/// @return derivates of the states
		///
		virtual std::vector<double> solve( double t, bool storeAccels=false);
		///
		/// Get the states
		///
		/// @param[in]	nothing
		/// @return the states
		///
		virtual std::vector<double> getState();
		/////
		///// Get the derivative of the states wrt time
		///// returns same thing as solve but does not resolve
		/////
		///// @param[in]	nothing
		///// @return the states
		/////
		//   virtual std::vector<double> getDot()= 0;
		/////
		///// Get the size of the state vector i.e. the number of states
		/////
		///// @param[in]	nothing
		///// @return the size of the state vector
		/////
		//   virtual unsigned int getStateSize()= 0;
		///
		/// Set the state vector 
		///
		/// @param[in] std vector of the state
		/// @return nothing
		///
		virtual void setState(std::vector<double> state_vector);
		/////
		///// apply Forces to Body
		/////
		///// @param[in]	t time
		///// @return Nothing
		/////
		void applyForces(double t );
		/// 
		/// add a Ridid Body to the model using no eigen, for use in swig
		/// 
		/// @param[in] name : name of the body
		/// @param[in] mass : mass of the body
		/// @param[in] inertia : inertia of the body
		/// @param[in] pos:  position of the center of the mass of the body
		/// @param[in] q: orientaion of the body in the form of a quaternion
		/// @param[in] vel: initial velocity of the body at the center of mass of the body
		/// @param[in] wl: initial angular velocity of the body
		/// @param[in] fixed:  true if the body is fixed to ground and does not move, otherwise false
		///
		/// @return pointer to the body added to the model
		///
		BodyRigid* addBodyRigid(std::string const &name, double const &mass, std::vector<double> const &inertia, std::vector<double> const &pos, std::vector<double> q, std::vector<double> const &vel, std::vector<double> const &wl, bool const &fixed=false);
		/// 
		/// add a Ridid Body to the model
		/// 
		/// @param[in] name : name of the body
		/// @param[in] mass : mass of the body
		/// @param[in] inertia : inertia of the body
		/// @param[in] pos:  position of the center of the mass of the body
		/// @param[in] q: orientaion of the body in the form of a quaternion
		/// @param[in] vel: initial velocity of the body at the center of mass of the body
		/// @param[in] wl: initial angular velocity of the body
		/// @param[in] fixed:  true if the body is fixed to ground and does not move, otherwise false
		///
		/// @return pointer to the body added to the model
		///
		BodyRigid* addBodyRigid(std::string const &name, double const &mass, Mat3x3 const &inertia, Vect3 const &pos=Vect3(0,0,0), Quat const &q=Quat(1,0,0,0), Vect3 const &vel=Vect3(0,0,0), Vect3 const &wl=Vect3(0,0,0), bool const &fixed=false);
		///
		/// add a Force of type Force1Body to the model without Eigen, for use with swig
		/// 
		/// @param[in] name : name of the force
		/// @param[in] *body : pointer to the body on which to apply the force or torque
		/// @param[in] t : torque vector which defines the force applied to the body
		/// @param[in] torqueLocal: true of the vector f is in local body coordinates, false indicates f is in global coordinates
		/// 
		/// @return pointer force of type Force1Body added to the model
		///
		Force1Body * addTorqueOnBody(std::string const &name, BodyRigid * body, std::vector<double> const &t, bool const &torqueLocal=true);	
		///
		/// add a Force of type Force1Body to the model without Eigen, for use with swig
		/// 
		/// @param[in] name : name of the force
		/// @param[in] *body : pointer to the body on which to apply the force or torque
		/// @param[in] f : force vector which defines the force applied to the body
		/// @param[in] forceLocation : location in body coordinates at which to apply the force
		/// @param[in] forceLocal: true of the vector f is in local body coordinates, false indicates f is in global coordinates
		/// 
		/// @return pointer force of type Force1Body added to the model
		///
		Force1Body * addForceOnBody(std::string const &name, BodyRigid * body, std::vector<double> const &f, std::vector<double> const &forceLocation, bool const &forceLocal=true);
		///
		/// add a Force of type Force1Body to the model
		/// 
		/// @param[in] name : name of the force
		/// @param[in] *body : pointer to the body on which to apply the force or torque
		/// @param[in] f : force vector which defines the force applied to the body
		/// @param[in] forceLocation : location in body coordinates at which to apply the force
		/// @param[in] forceLocal: true of the vector f is in local body coordinates, false indicates f is in global coordinates
		/// @param[in] t: torque vector which defines th torque applied to the body
		/// @param[in] torqueLocal: true if the torque, t, is to be applied in the local body coordinates, false indicates t is in global coordinates
		/// 
		/// @return pointer force of type Force1Body added to the model
		///
		Force1Body * addForce1Body(std::string const &name, BodyRigid * body, Vect3 const &f, Vect3 const &forceLocation=Vect3(0,0,0),
			bool const &forceLocal=true, Vect3 const &t=Vect3(0,0,0), bool const &torqueLocal=true);


		Force1Body * addTorqueOnBody(std::string const &name, BodyRigid * body, Vect3 const &torque, bool const &torqueLocal=true);
		///
		/// add a Force which applies the defined magnitude and direction of gravity to all bodies in the model, No eigen elements for use in swig
		///
		/// @param[in] name : name of the force
		/// @param[in] g: magnitude of gravity
		/// @param[in] direction: direction of gravity
		///
		/// @return pointer to ForceGravity
		///
		ForceGravity * addForceGravity(std::string const &name, double g, vector<double> const &direction);
		///
		/// add a Force which applies the defined magnitude and direction of gravity to all bodies in the model
		///
		/// @param[in] name : name of the force
		/// @param[in] g: magnitude of gravity
		/// @param[in] direction: direction of gravity
		///
		/// @return pointer to ForceGravity
		///
		ForceGravity * addForceGravity(std::string const &name, double g=9.81, Vect3 const &direction=Vect3(0,0,-1));
		///
		/// add a Force which applies the defined magnitude and direction of gravity to all bodies in the model
		///
		/// @param[in] name : name of the force
		/// @param[in] direction: vector of gravity
		///
		/// @return pointer to ForceGravity
		///
		ForceGravity * addForceGravity(std::string const &name, Vect3 const &v=Vect3(0,0,-9.81));
		///
		/// add a Force which representing a spring damper to connecting 2 bodies
		///
		/// @param[in] name : name of the force
		/// @param[in] body1: First Body
		/// @param[in] body2: Second Body
		/// @param[in] k: spring stiffness
		/// @param[in] c: spring damper
		/// @param[in] fl: free length
		/// @param[in] body1Offset: location on body1 in body1 coordinates at which the spring is connected
		/// @param[in] body2Offset: location on body2 in body2 coordinates at which the spring is connected
		///
		/// @return pointer to Force2BodySpringDamp
		///
		Force2BodySpringDamp * addForce2BodySpringDamp ( string const& name,
			BodyRigid *body1,
			BodyRigid *body2,
			double k,
			double c,
			double fl = 0,
			Vect3 body1Offset =Vect3(0.,0.,0.),
			Vect3 body2Offset =Vect3(0.,0.,0.));
		///
		/// add a Force which representing a spring damper to connecting 2 bodies
		///
		/// @param[in] name : name of the force
		/// @param[in] body1: First Body
		/// @param[in] body2: Second Body
		/// @param[in] k: spring stiffness
		/// @param[in] c: spring damper
		/// @param[in] fl: free length
		/// @param[in] body1Offset: location on body1 in body1 coordinates at which the spring is connected
		/// @param[in] body2Offset: location on body2 in body2 coordinates at which the spring is connected
		///
		/// @return pointer to Force2BodySpringDamp
		///
		Force2BodySpringDamp * addForce2BodySpringDamp ( string const& name,
			BodyRigid *body1,
			BodyRigid *body2,
			double k,
			double c,
			double fl,
			std::vector<double> body1Offset,
			std::vector<double> body2Offset);


		#ifdef USE_BULLET
		ForceContact * addForceContact ( string const& name, btCollisionObjectArray objects, double stiff, double damp, double frict, double thresh  );
		ForceContact * addForceContact ( string const& name, btCollisionWorld *collisionWorld, double stiff, double damp, double frict, double thresh);
		ForceContact * addForceContact ( string const& name, double stiff, double damp, double frict, double thresh);
		#endif
		#ifdef USE_OGRE	// don't include in SWIG
		ForceCollisionDynWorld * addForceCollisionDynWorld ( string const& name, double stiff, double damp, double frict, double thresh );
		#endif


		///
		/// get a body in the model
		///
		/// Once you have the body then you call the method to access position, orientation ...
		/// @param[in] bodyname Name of the body to get
		/// @return the body
		///
		BodyRigid *getBody( string bodyname );

		///
		/// set the integration scheme to RK4 which it is by default, so you would only need this if integration scheme was changed
		///
		IntegratorRK4 * setIntegratorRK4();
		///
		/// set the integration scheme to Euler
		///
		IntegratorEuler * setIntegratorEuler();

		virtual void integrate( double t0, double t1, bool storeBodyAccels=false);
	protected:
		//friend class Model2;
		std::vector<BodyRigid *> m_rigidBodies;
		std::vector<Force *> m_forces;
		bool searchForcesForName(std::string const &name);
	private:
		
	};
};
#endif