#ifndef OMD_MODEL_H
#define OMD_MODEL_H

#include <vector>
#include "Force.h"
#include "Body.h"
#include "Integrator.h"

namespace OMD
{
/// \brief
/// Virtual Base class representing a formulation of the multi-body dynamics system
///
///	@author John Kollman
///
///
class Model
{
public:

	Model(void)
	{
	}

	~Model(void)
	{
		delete m_integrator;
	}
	///
	/// Solve the system for the derivates of the states
	///
	/// @param[in]	t time
	/// @return derivates of the states
	///
	virtual std::vector<double> solve( double t, bool storeAccels=false)= 0;
	///
	/// Get the states
	///
	/// @param[in]	nothing
	/// @return the states
	///
	virtual std::vector<double> getState()=0;
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
	virtual void setState(std::vector<double> state_vector)=0;
	virtual void integrate( double t0, double t1, bool storeBodyAccels=false)=0;
	/////
	///// apply Forces to Body
	/////
	///// @param[in]	t time
	///// @return Nothing
	/////
 //   void applyForces(double t );
	/////
	///// get the vector of bodies
	/////
	///// @param[in] nothing
	///// @return std vector of bodies
	/////
 //   std::vector<Body *>* getBodies(){return &m_bodies;};
 //   ///
 //   /// get a body in the model
 //   ///
 //   /// Once you have the body then you call the method to access position, orientation ...
 //   /// @param[in] bodyname Name of the body to get
 //   /// @return pointer to body or NULL if body is not found
 //   ///
	//Body *getBody( 	string bodyname )
	//{
	//	std::vector< Body *>::reverse_iterator rit;
	//	for (rit = m_bodies.rbegin(); rit < m_bodies.rend(); ++rit )
	//	{
	//		if (bodyname == (*rit)->m_name)
	//		{
	//			return (*rit);
	//		}
	//	}
	//	return NULL;
	//}

	Integrator *m_integrator;

protected:
    //std::map<string, Force *> m_forces;
	//std::vector<double> m_state_vector;
	//std::vector<double> m_state_vector_dot;
	
};

};
#endif // OMD_MODEL_H