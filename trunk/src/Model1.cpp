#include "Model1.h"

namespace OMD
{
	Model1::Model1(void)
	{
		m_integrator = new IntegratorRK4(); // use RK4 as default
	}


	Model1::~Model1(void)
	{
		// delete forces
		std::vector<Force *>::iterator itf;
		for (itf = m_forces.begin(); itf < m_forces.end(); itf++)
		{
			delete *itf;
		}

		// delete Rigid Bodies
		std::vector<BodyRigid *>::iterator itb;
		for (itb = m_rigidBodies.begin(); itb < m_rigidBodies.end(); itb++)
		{
			delete *itb;
		}
	}

	std::vector<double> Model1::solve( double t, bool storeAccels)
	{
		applyForces(t);

		std::vector<double> state_vector_dot;
		// no constraints so bodies can solve themselves
		std::vector<BodyRigid *>::iterator itb;
		for (itb = m_rigidBodies.begin(); itb < m_rigidBodies.end(); itb++)
		{
			BodyRigid * rigidBody = (*itb);
			if (!rigidBody->isFixed())
			{
				// translational acceleration in global
				Vect3 transAccel = rigidBody->solveTransAccel();
				// rotational acceleration in body coordinates
				Vect3 rotAccel = rigidBody->solveRotAccel();

				//m_state_vector_dot.clear();

				state_vector_dot.push_back(transAccel(0));
				state_vector_dot.push_back(transAccel(1));
				state_vector_dot.push_back(transAccel(2));
				state_vector_dot.push_back(rotAccel(0));
				state_vector_dot.push_back(rotAccel(1));
				state_vector_dot.push_back(rotAccel(2));

				Vect3 transVel = rigidBody->getVelocityGlobal();

				Quat quatDot = rigidBody->getQuatDot();

				state_vector_dot.push_back(transVel.x());
				state_vector_dot.push_back(transVel.y());
				state_vector_dot.push_back(transVel.z());
				state_vector_dot.push_back(quatDot.w());
				state_vector_dot.push_back(quatDot.x());
				state_vector_dot.push_back(quatDot.y());
				state_vector_dot.push_back(quatDot.z());
			}
		}
		return state_vector_dot;
	}

	BodyRigid* Model1::addBodyRigid(std::string const &name, double const &mass, std::vector<double> const &inertia, std::vector<double> const &pos, std::vector<double> q, std::vector<double> const &vel, std::vector<double> const &wl, bool const &fixed)
	{
		BodyRigid *b = new BodyRigid(name,mass,inertia,pos,q,vel,wl,fixed);
		m_rigidBodies.push_back ( b );
		if (fixed)
		{
			b->setAcceleration(Vect3(0,0,0));
			b->setLocalAngularAccel(Vect3(0,0,0));
		}
		return b;
	}

	BodyRigid* Model1::addBodyRigid(std::string const &name, double const &mass, Mat3x3 const &inertia, Vect3 const &pos, Quat const &q, Vect3 const &vel, Vect3 const &wl, bool const &fixed)
	{
		BodyRigid *b = new BodyRigid(name,mass,inertia,pos,q,vel,wl,fixed);
		m_rigidBodies.push_back ( b );
		if (fixed)
		{
			b->setAcceleration(Vect3(0,0,0));
			b->setLocalAngularAccel(Vect3(0,0,0));
		}
		return b;
	}

	Force1Body * Model1::addTorqueOnBody(std::string const &name, BodyRigid * body, std::vector<double> const &t, bool const &torqueLocal)
	{
		Vect3 t_(t[0],t[1],t[2]);
		return addForce1Body(name,body,Vect3(0,0,0),Vect3(0,0,0),true,t_,torqueLocal);
	}

	Force1Body * Model1::addForceOnBody(std::string const &name, BodyRigid * body, std::vector<double> const &f, std::vector<double> const &forceLocation, bool const &forceLocal)
	{
		Vect3 f_(f[0],f[1],f[2]);
		Vect3 l_(forceLocation[0], forceLocation[1], forceLocation[2]);
		return addForce1Body(name,body,f_,l_,forceLocal);
	}

	Force1Body * Model1::addForce1Body(std::string const &name, BodyRigid * body, Vect3 const &f, Vect3 const &forceLocation,
		bool const &forceLocal, Vect3 const &t, bool const &torqueLocal)
	{
		bool nameAlreadyUsed = searchForcesForName(name);
		if (nameAlreadyUsed)  /// TODO: do something more informative for the user then pass back NULL
		{
			return NULL;
		}
		else
		{
			Force1Body *force = new Force1Body(name, body, f, forceLocation, forceLocal, t, torqueLocal);
			m_forces.push_back(force);	// add force to the vector of forces
			return force;
		}
	}

	Force1Body * Model1::addTorqueOnBody(std::string const &name, BodyRigid * body, Vect3 const &torque, bool const &torqueLocal)
	{
		bool nameAlreadyUsed = searchForcesForName(name);
		if (nameAlreadyUsed)  /// TODO: do something more informative for the user then pass back NULL
		{
			return NULL;
		}
		else
		{
			Force1Body *force = new Force1Body(name, body, Vect3(0,0,0), Vect3(0,0,0), false, torque, torqueLocal);
			m_forces.push_back(force);	// add force to the vector of forces
			return force;
		}
	}

	ForceGravity * Model1::addForceGravity(std::string const &name, double g, vector<double> const &direction)
	{
		Vect3 d_(direction[0],direction[1],direction[2]);
		return addForceGravity(name,g,d_);
	}

	ForceGravity * Model1::addForceGravity(std::string const &name, double g, Vect3 const &direction)
	{
		bool nameAlreadyUsed = searchForcesForName(name);
		if (nameAlreadyUsed)  /// TODO: do something more informative for the user then pass back NULL
		{
			return NULL;
		}
		else
		{
			ForceGravity *force = new ForceGravity(name, g, direction);
			force->addRigidBodies(m_rigidBodies);
			m_forces.push_back(force);	// add force to the vector of forces
			return force;
		}
	}

	ForceGravity * Model1::addForceGravity(std::string const &name, Vect3 const &v)
	{
		bool nameAlreadyUsed = searchForcesForName(name);
		if (nameAlreadyUsed)  /// TODO: do something more informative for the user then pass back NULL
		{
			return NULL;
		}
		else
		{
			ForceGravity *force = new ForceGravity(name, v);
			force->addRigidBodies(m_rigidBodies);
			m_forces.push_back(force);	// add force to the vector of forces
			return force;
		}
	}

	Force2BodySpringDamp * Model1::addForce2BodySpringDamp ( string const& name,
			BodyRigid *body1,
			BodyRigid *body2,
			double k,
			double c,
			double fl,
			vector<double> body1Offset,
			vector<double> body2Offset)
	{
		bool nameAlreadyUsed = searchForcesForName(name);
		if (nameAlreadyUsed)  /// TODO: do something more informative for the user then pass back NULL
		{
			return NULL;
		}
		else
		{
			Force2BodySpringDamp *force = new Force2BodySpringDamp(name, body1, body2, k, c, fl, body1Offset, body2Offset);
			m_forces.push_back(force);	// add force to the vector of forces
			return force;
		}
	}

	Force2BodySpringDamp * Model1::addForce2BodySpringDamp ( string const& name,
			BodyRigid *body1,
			BodyRigid *body2,
			double k,
			double c,
			double fl,
			Vect3 body1Offset,
			Vect3 body2Offset)
	{
		bool nameAlreadyUsed = searchForcesForName(name);
		if (nameAlreadyUsed)  /// TODO: do something more informative for the user then pass back NULL
		{
			return NULL;
		}
		else
		{
			Force2BodySpringDamp *force = new Force2BodySpringDamp(name, body1, body2, k, c, fl, body1Offset, body2Offset);
			m_forces.push_back(force);	// add force to the vector of forces
			return force;
		}
	}

	void Model1::applyForces(double t)
	{
		std::vector<BodyRigid *>::iterator it;

		// zero out forces and torques
		for ( it=m_rigidBodies.begin() ; it < m_rigidBodies.end(); it++ )
		{
			(*it)->forceAccumReset();
		}

		// apply new forces and torques
		std::vector<Force *>::iterator it2;

		for (it2=m_forces.begin(); it2 < m_forces.end(); it2++)
		{
			(*it2)->apply(t);
		}
	}

	IntegratorRK4 * Model1::setIntegratorRK4()
	{
		delete m_integrator;
		m_integrator = new IntegratorRK4();
		return (IntegratorRK4*) m_integrator;
	}

	IntegratorEuler * Model1::setIntegratorEuler()
	{
		delete m_integrator;
		m_integrator = new IntegratorEuler();
		return (IntegratorEuler*) m_integrator;
	}

	void Model1::integrate( double t0, double t1, bool storeBodyAccels)
	{
		m_integrator->integrate(this,t0,t1,storeBodyAccels);
	}

	std::vector<double> Model1::getState()
	{
		std::vector<double> state;

		std::vector< BodyRigid *>::iterator it;
		for (it = m_rigidBodies.begin(); it < m_rigidBodies.end(); ++it)
		{
			BodyRigid *body = (*it);
			if (body->isFixed()==false)
			{
				//Vect3 vel = body->getVelLocal();
				Vect3 vel = body->m_vel;
				state.push_back(vel.x());
				state.push_back(vel.y());
				state.push_back(vel.z());

				Vect3 wl = body->m_wl;
				state.push_back(wl.x());
				state.push_back(wl.y());
				state.push_back(wl.z());

				Vect3 p = body->m_pos;
				state.push_back(p.x());
				state.push_back(p.y());
				state.push_back(p.z());

				Quat q = body->m_q;
				state.push_back(q.w());
				state.push_back(q.x());
				state.push_back(q.y());
				state.push_back(q.z());
			}

		}
		return state;
	}

	void Model1::setState(std::vector<double> state_vector)
	{
		 std::vector< BodyRigid *>::reverse_iterator rit;
		for (rit = m_rigidBodies.rbegin(); rit < m_rigidBodies.rend(); ++rit )
		{
		    BodyRigid *body = (*rit);
		    double e3 = state_vector.back();
		    state_vector.pop_back();
		    double e2 = state_vector.back();
		    state_vector.pop_back();
		    double e1 = state_vector.back();
		    state_vector.pop_back();
		    double e0 = state_vector.back();
		    state_vector.pop_back();

            Quat q(e0,e1,e2,e3);
			// page 317 Nikravesh
			q.normalize();
            body->m_q = q;

            double z = state_vector.back();
		    state_vector.pop_back();
		    double y = state_vector.back();
		    state_vector.pop_back();
		    double x = state_vector.back();
		    state_vector.pop_back();

            body->m_pos = Vect3(x,y,z);

            double wz = state_vector.back();
            state_vector.pop_back();
            double wy = state_vector.back();
            state_vector.pop_back();
            double wx = state_vector.back();
            state_vector.pop_back();

            body->m_wl = Vect3(wx,wy,wz);

            double zdot = state_vector.back();
            state_vector.pop_back();
            double ydot = state_vector.back();
            state_vector.pop_back();
            double xdot = state_vector.back();
            state_vector.pop_back();

            body->m_vel = Vect3(xdot,ydot,zdot);
		}
	}

	bool Model1::searchForcesForName(std::string const &name)
	{
		// first see if a force of the same name already exists
		bool nameAlreadyUsed = false;
		std::vector<Force *>::iterator it;
		for (it = m_forces.begin(); it < m_forces.end(); it++)
		{
			std::string existingName = (*it)->getName();
			if (existingName.compare(name) == 0)
			{
				return true;
			}
		}
		return false;
	}

	BodyRigid* Model1::getBody( string bodyname )
	{
		string name;
		std::vector< BodyRigid *>::reverse_iterator rit;
		for (rit = m_rigidBodies.rbegin(); rit < m_rigidBodies.rend(); ++rit )
		{
			if (bodyname == (*rit)->m_name)
			{
				return (*rit);
			}
		}
		return NULL;
	}


#ifdef USE_OGRE	// use with ogre
	ForceCollisionDynWorld * Model1::addForceCollisionDynWorld ( string const& name, double stiff, double damp, double frict, double thresh )
	{
		bool nameAlreadyUsed = searchForcesForName(name);
		if (!nameAlreadyUsed)
		//if (m_forces.find(name) == m_forces.end())
		{
			ForceCollisionDynWorld *f = new ForceCollisionDynWorld ( name, stiff, damp, frict, thresh );
			m_forces.push_back(f);
			//m_forces[f->getName()] = f;
			return f;
		}
		else
			return NULL;
	}
#endif

#ifdef USE_BULLET
	ForceContact * Model1::addForceContact ( string const& name, btCollisionObjectArray objects,double stiff, double damp, double frict, double thresh )
	{
		bool nameAlreadyUsed = searchForcesForName(name);
		if (!nameAlreadyUsed)
		//if (m_forces.find(name) == m_forces.end())
		{
			ForceContact *f = new ForceContact ( name, objects, stiff, damp, frict, thresh );
			//m_forces[f->getName()] = f;
			m_forces.push_back(f);
			return f;
		}
		else
			return NULL;
	}
	ForceContact * Model1::addForceContact ( string const& name, btCollisionWorld *collisionWorld, double stiff, double damp, double frict, double thresh)
	{
		bool nameAlreadyUsed = searchForcesForName(name);
		if (!nameAlreadyUsed)
		//if (m_forces.find(name) == m_forces.end())
		{
			ForceContact *f = new ForceContact ( name, collisionWorld, stiff, damp, frict, thresh );
			//m_forces[f->getName()] = f;
			m_forces.push_back(f);
			return f;
		}
		else
			return NULL;
	}

	ForceContact * Model1::addForceContact ( string const& name, double stiff, double damp, double frict, double thresh)
	{
		bool nameAlreadyUsed = searchForcesForName(name);
		if (!nameAlreadyUsed)
		//if (m_forces.find(name) == m_forces.end())
		{
			ForceContact *f = new ForceContact ( name, stiff, damp, frict, thresh );
			//m_forces[f->getName()] = f;
			m_forces.push_back(f);
			return f;
		}
		else
			return NULL;
	}
#endif

}