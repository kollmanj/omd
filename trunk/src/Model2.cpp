#include "Model2.h"
#include <list>

namespace OMD
{

	Model2::Model2()
	{
		Mat3x3 eye;
		eye <<	1,0,0,
				0,1,0,
				0,0,1;

		addBodyRigid("DefaultIRF",0,eye,eye,true);
	}
	Model2::~Model2()
	{
		// delete Joints
		for (map<Joint1DOF *, BodyRigid *>::iterator i=m_child_map.begin(); i!=m_child_map.end(); ++i)
		{
			Joint1DOF *j = i->first;
			delete j;
		}

		//		// delete bodies
		//		for (unsigned int i=0; i < m_bodies.size(); ++i)
		//		{
		//			Body *body = m_bodies[i];
		//			delete body;
		//		}


		//TODO: put this back in, but in Model1
		//// delete curves
		//for (unsigned int i=0; i < m_curve2Ds.size(); ++i)
		//{
		//	Curve2D *curve = m_curve2Ds[i];
		//	delete curve;
		//}


	}

	void Model2::kinematics( double t )
	{
		m_tree.kinematics( t );
	}


	void Model2::Dynamics( double t )
	{
		for (unsigned int i=0; i < m_rigidBodies.size(); ++i)
		{
			BodyRigid *body = m_rigidBodies[i];
			body->dynamicAccumReset();
		}
		m_tree.triangularize(t);

		m_tree.ForwardSubstitute(t);
	}

	std::vector<double> Model2::solve( double t, bool storeAccels )
	{
		kinematics(t);
		applyForces(t);
		Dynamics(t);
		std::vector<double> dots = getDot();

		if (storeAccels)
		{
			m_tree.setBodyAccels();
		}

		return dots;
	}

	JointTranslational* Model2::addJointTranslational(string const &name, BodyRigid * parent,
			vector<double> const &parent2joint,
			BodyRigid *child,
			vector<double> const &joint2child,
			vector<double> const & axis,
			double q0, double u0 )
	{
		JointTranslational *j = new JointTranslational(name,parent,parent2joint,child,joint2child,axis,q0,u0);
		BodyRigid * p = j->getParent();
		BodyRigid * c = j->getChild();
		m_parent_map.insert ( pair<BodyRigid*, Joint1DOF*> ( p, j ) );
		m_child_map.insert ( pair<Joint1DOF*, BodyRigid*> (j, c) );
		return j;
	}

	JointTranslational* Model2::addJointTranslational(	string const &name,
		BodyRigid * parent,
		Vect3 parent2joint,
		BodyRigid *child,
		Vect3 joint2child,
		Vect3 axis,
		double q0,
		double u0)
	{
		JointTranslational *j = new JointTranslational(name, parent, parent2joint, child, joint2child, axis, q0, u0);
		BodyRigid * p = j->getParent();
		BodyRigid * c = j->getChild();
		m_parent_map.insert ( pair<BodyRigid*, Joint1DOF*> ( p, j ) );
		m_child_map.insert ( pair<Joint1DOF*, BodyRigid*> (j, c) );
		return j;
	}

	JointTranslational* Model2::addJointTranslational(string const &name, std::string parentname,
			vector<double> parent2joint,
			std::string childname,
			vector<double> joint2child,
			vector<double> axis,
			double q0, double u0 )
	{
		BodyRigid *parent = getBody(parentname);
		BodyRigid *child = getBody(childname);
		return addJointTranslational(name, parent, parent2joint, child, joint2child, axis, q0, u0);
	}

	JointTranslational* Model2::addJointTranslational(	string const &name,
		std::string parentname,
		Vect3 parent2joint,
		std::string childname,
		Vect3 joint2child,
		Vect3 axis,
		double q0,
		double u0)
	{
		// TODO: if Body does not exist fail gracefully
		BodyRigid *parent = getBody(parentname);
		BodyRigid *child = getBody(childname);
		JointTranslational *j = new JointTranslational(name, parent, parent2joint, child, joint2child, axis, q0, u0);
		BodyRigid * p = j->getParent();
		BodyRigid * c = j->getChild();
		m_parent_map.insert ( pair<BodyRigid*, Joint1DOF*> ( p, j ) );
		m_child_map.insert ( pair<Joint1DOF*, BodyRigid*> (j, c) );
		return j;
	}
	//	bool Model2::addJoint ( JointRevolute j )
	//	{
	//		/// TODO: check to see if name exists
	//		JointRevolute *j_copy = new JointRevolute(j);
	//		Body * parent = j_copy->getParent();
	//		Body * child = j_copy->getChild();
	//		m_parent_map.insert ( pair<Body*, Joint1DOF*> ( parent, j_copy ) );
	//		m_child_map.insert ( pair<Joint1DOF*, Body*> (j_copy, child) );
	//		return true;
	//	}

	JointRevolute* Model2::addJointRevolute(	string const& name,
		std::string parentname,
		Vect3 parent2joint,
		std::string childname,
		Vect3 joint2child,
		Vect3 axis,
		double q0,
		double u0)
	{
		// TODO: if Body does not exist fail gracefully
		BodyRigid *parent = getBody(parentname);
		BodyRigid *child = getBody(childname);
		JointRevolute *j = new JointRevolute( name, parent, parent2joint, child, joint2child, axis, q0, u0);
		BodyRigid * p = j->getParent();
		BodyRigid * c = j->getChild();
		m_parent_map.insert ( pair<BodyRigid*, Joint1DOF*> ( p, j ) );
		m_child_map.insert ( pair<Joint1DOF*, BodyRigid*> (j, c) );
		return j;
	}

	JointRevolute* Model2::addJointRevolute(	string const& name,
			BodyRigid * parent,
			vector<double> parent2joint,
			BodyRigid* child,
			vector<double> joint2child,
			vector<double> axis,
			double q0,
			double u0)
	{
		JointRevolute *j = new JointRevolute( name, parent, parent2joint, child, joint2child, axis, q0, u0);
		BodyRigid * p = j->getParent();
		BodyRigid * c = j->getChild();
		m_parent_map.insert ( pair<BodyRigid*, Joint1DOF*> ( p, j ) );
		m_child_map.insert ( pair<Joint1DOF*, BodyRigid*> (j, c) );
		return j;
	}

	JointRevolute* Model2::addJointRevolute(	string const& name,
			std::string parentname,
			vector<double> parent2joint,
			std::string childname,
			vector<double> joint2child,
			vector<double> axis,
			double q0,
			double u0)
	{
		BodyRigid *parent = getBody(parentname);
		BodyRigid *child = getBody(childname);
		return addJointRevolute(name, parent, parent2joint, child, joint2child, axis, q0, u0);
	}

	JointRevolute* Model2::addJointRevolute(	string const& name,
		BodyRigid* parent,
		Vect3 parent2joint,
		BodyRigid* child,
		Vect3 joint2child,
		Vect3 axis,
		double q0,
		double u0)
	{
		JointRevolute *j = new JointRevolute( name, parent, parent2joint, child, joint2child, axis, q0, u0);
		BodyRigid * p = j->getParent();
		BodyRigid * c = j->getChild();
		m_parent_map.insert ( pair<BodyRigid*, Joint1DOF*> ( p, j ) );
		m_child_map.insert ( pair<Joint1DOF*, BodyRigid*> (j, c) );
		return j;
	}

	//	bool Model2::addJoint ( JointTrans j )
	//	{
	//		/// TODO: check to see if name exists
	//		JointTrans *j_copy = new JointTrans(j);
	//		Body * parent = j_copy->getParent();
	//		Body * child = j_copy->getChild();
	//		m_parent_map.insert ( pair<Body*, Joint1DOF*> ( parent, j_copy ) );
	//		m_child_map.insert ( pair<Joint1DOF*, Body*> (j_copy, child) );
	//		return true;
	//	}

	//	Body* Model2::addBody ( Body bodyin )
	//	{
	//		/// TODO: check to see if name exists
	//		Body *bodycopy = new Body(bodyin);
	//		m_bodies.push_back ( bodycopy );
	//		return bodycopy;
	//	}
	BodyRigid* Model2::addBodyRigid (  string name,
			double mass,
			vector<double> inertia,
			vector<double> orientation,
			bool fixed)
	{
		Mat3x3 i;
		i <<	inertia[0], inertia[1], inertia[2],
				inertia[3],	inertia[4],	inertia[5],
				inertia[6], inertia[7], inertia[8];
		Mat3x3 o;
		o <<	orientation[0], orientation[1], orientation[2],
				orientation[3],	orientation[4],	orientation[5],
				orientation[6], orientation[7], orientation[8];
		Quat q(o);
		BodyRigid *b = new BodyRigid(name, mass, i, Vect3(0,0,0), q, Vect3(0,0,0),Vect3(0,0,0),fixed);
		m_rigidBodies.push_back ( b );
		return b;
	}

	BodyRigid* Model2::addBodyRigid (	string name,
		double mass,
		Mat3x3 inertia,
		Mat3x3 orientation,
		bool fixed)
	{
		Quat q(orientation);

		// does not matter what the position is set to
		//BodyRigid *b = new BodyRigid(name, mass, inertia, Vect3(0,0,0), q);
		BodyRigid *b = new BodyRigid(name, mass, inertia, Vect3(0,0,0), q, Vect3(0,0,0),Vect3(0,0,0),fixed);

		m_rigidBodies.push_back ( b );
		return b;
	}

	BodyRigid* Model2::addBodyRigid (	string name,
		double mass,
		Mat3x3 inertia,
		Quat orientation,
		bool fixed)
	{
		Quat q=orientation;

		// does not matter what the position is set to
		//BodyRigid *b = new BodyRigid(name, mass, inertia, Vect3(0,0,0), q);
		BodyRigid *b = new BodyRigid(name, mass, inertia, Vect3(0,0,0), q, Vect3(0,0,0),Vect3(0,0,0),fixed);

		m_rigidBodies.push_back ( b );
		return b;
    }
	//	bool Model2::addForce ( Force1Body f )
	//	{
	//		if (m_forces.find(f.getName()) == m_forces.end())
	//		{
	//			Force1Body *f_copy = new Force1Body(f);
	//			m_forces[f_copy->getName()] = f_copy;
	//			return true;
	//		}
	//		else
	//			return false; // name exists already
	//	}


	//	bool Model2::addForce ( ForceTransJnt f )
	//	{
	//		if (m_forces.find(f.getName()) == m_forces.end())
	//		{
	//			ForceTransJnt *f_copy = new ForceTransJnt(f);
	//			m_forces[f_copy->getName()] = f_copy;
	//			return true;
	//		}
	//		else
	//			return false; // name exists already
	//	}

	//	bool Model2::addForce ( ForceTransJntSpringDamp f )
	//	{
	//		if (m_forces.find(f.getName()) == m_forces.end())
	//		{
	//			ForceTransJntSpringDamp *f_copy = new ForceTransJntSpringDamp(f);
	//			m_forces[f_copy->getName()] = f_copy;
	//			return true;
	//		}
	//		else
	//			return false; // name exists already
	//	}

	//Force1Body * Model2::addForce1Body ( string const& name,
	//	Body *body,
	//	Vect3 force,
	//	Vect3 torque,
	//	Vect3 coord )
	//{
	//	if (m_forces.find(name) == m_forces.end())
	//	{
	//		Force1Body *f = new Force1Body(name,body,force,torque,coord);
	//		m_forces[f->getName()] = f;
	//		return f;
	//	}
	//	else
	//		return NULL;
	//}

	//ForceGravity * Model2::addForceGravity ( string const& name, Vect3 g)
	//{
	//	if (m_forces.find(name) == m_forces.end())
	//	{
	//		ForceGravity *f = new ForceGravity(name,g,getBodies());
	//		m_forces[f->getName()] = f;
	//		return f;
	//	}
	//	else
	//		return NULL;

	//}
//#ifdef USE_OGRE	// use with ogre
//	ForceCollisionDynWorld * Model2::addForceCollisionDynWorld ( string const& name, double stiff, double damp, double frict, double thresh )
//	{
//		if (m_forces.find(name) == m_forces.end())
//		{
//			ForceCollisionDynWorld *f = new ForceCollisionDynWorld ( name, stiff, damp, frict, thresh );
//			m_forces[f->getName()] = f;
//			return f;
//		}
//		else
//			return NULL;
//	}
//#endif

//#ifdef USE_BULLET
//	ForceContact * Model2::addForceContact ( string const& name, btCollisionObjectArray objects,double stiff, double damp, double frict, double thresh )
//	{
//		if (m_forces.find(name) == m_forces.end())
//		{
//			ForceContact *f = new ForceContact ( name, objects, stiff, damp, frict, thresh );
//			m_forces[f->getName()] = f;
//			return f;
//		}
//		else
//			return NULL;
//	}
//	ForceContact * Model2::addForceContact ( string const& name, btCollisionWorld *collisionWorld, double stiff, double damp, double frict, double thresh)
//	{
//		if (m_forces.find(name) == m_forces.end())
//		{
//			ForceContact *f = new ForceContact ( name, collisionWorld, stiff, damp, frict, thresh );
//			m_forces[f->getName()] = f;
//			return f;
//		}
//		else
//			return NULL;
//	}
//
	ForceContact * Model2::addForceContact ( string const& name, double stiff, double damp, double frict, double thresh)
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
//#endif
	/// TODO put this back in
	//Curve2DSineWave * Model2::addCurve2DSineWave(string const& name, double amp, double offset, double freq, double shift)
	//{
	//	Curve2DSineWave *c = new Curve2DSineWave(name,amp,offset,freq,shift);
	//	m_curve2Ds.push_back(c);
	//	return c;
	//}

	ForceRevJnt * Model2::addForceRevJnt(string const& name,
		JointRevolute *jnt,
		double trq)
	{
		ForceRevJnt * f = new ForceRevJnt(name,jnt,trq);
		m_forces.push_back(f);
		return f;
	}

	ForceRevJntPIDCurve2D * Model2::addForceRevJntPIDCurve2D(string const& name,
		JointRevolute * jnt,
		double p,
		double i,
		double d,
		Curve2DSine *curve)
	{
		// copy incomming curve
		// curve was already added
		//m_curve2Ds.push_back(curve); // add curve so we can keep it around and delete it in the end
		ForceRevJntPIDCurve2D *f = new ForceRevJntPIDCurve2D(name,jnt,p,i,d,curve);
		m_forces.push_back(f);
		return f;
	}
	//ForceRevJntPIDCurve2D * Model2::addForceRevJntPIDCurve2D(string const& name,
	//	JointRevolute * jnt,
	//	double p,
	//	double i,
	//	double d,
	//	std::vector<Curve2D *> curves)
	//{
	//	// copy incomming curve
	//	// curve was already added
	//	//m_curve2Ds.push_back(curve); // add curve so we can keep it around and delete it in the end
	//	ForceRevJntPIDCurve2D *f = new ForceRevJntPIDCurve2D(name,jnt,p,i,d,curves);
	//	m_forces[f->getName()] = f;
	//	return f;
	//}
	ForceRevJntSpringDamp * Model2:: addForceRevJntSpringDamp(string const& name,
		JointRevolute *jnt,
		double k,
		double c,
		double fl)
	{
		ForceRevJntSpringDamp * f = new ForceRevJntSpringDamp(name,jnt,k,c,fl);
		m_forces.push_back(f);
		return f;
	}

	//Force2BodySpringDamp * Model2::addForce2BodySpringDamp ( string const& name,
	//	Body *body1,
	//	Body *body2,
	//	double k,
	//	double c,
	//	double fl,
	//	Vect3 body1Offset,
	//	Vect3 body2Offset)
	//{
	//	Force2BodySpringDamp * f = new Force2BodySpringDamp(name,body1,body2,k,c,fl,body1Offset,body2Offset);
	//	m_forces[f->getName()] = f;
	//	return f;
	//}

	//Force2BodySpringDamp * Model2::addForce2BodySpringDamp ( string const& name,
	//	string const& body1Name,
	//	string const& body2Name,
	//	double k,
	//	double c,
	//	double fl,
	//	Vect3 body1Offset,
	//	Vect3 body2Offset)
	//{
	//	Body* body1 = getBody(body1Name);
	//	Body* body2 = getBody(body2Name);
	//	Force2BodySpringDamp * f = new Force2BodySpringDamp(name,body1,body2,k,c,fl,body1Offset,body2Offset);
	//	m_forces[f->getName()] = f;
	//	return f;
	//}


	bool Model2::buildTree()
	{
		// make a list of joints, put all in, take out as used, once empty then done
		list <Joint1DOF * > unusedjoints;
		// map to collect the child bodies of joints added to tree with the joints that where their parents
		map <BodyRigid *, Joint* > chld_bods_with_pjnts;

		// find all fixed bodies and create the level 0 branches in the tree



		multimap<BodyRigid *,Joint1DOF *>::iterator it;
		for ( it = m_parent_map.begin(); it != m_parent_map.end(); ++it )
		{
			BodyRigid * body = ( *it ).first;
			Joint1DOF * joint   = ( *it ).second;

			unusedjoints.push_back( joint );

			// if the body is fixed it needs to be  at base of tree
			if ( body->isFixed() )
			{
				m_tree.setRoot ( joint ) ;
				// remove from list of "all"
				unusedjoints.pop_back();
				// insert the body joint pair
				//			chld_bods_with_pjnts.insert(pair<Body*,Joint*>(joint->getChild(),joint));
				chld_bods_with_pjnts[joint->getChild()]=joint;
			}
		}


		bool loop = true;
		int i=0;
		while (loop)
		{
			list<Joint1DOF *>::iterator it2;
			for ( it2 = unusedjoints.begin(); it2 != unusedjoints.end();)
			{
				if (m_tree.addJoint(*it2))	// if true the tree found a place for the joint
				{
					unusedjoints.remove(*it2);
					it2 = unusedjoints.begin();
				}
				else
				{
					++it2;
				}

			}

			if (unusedjoints.empty() || i > 2000)
			{
				loop = false;
				if ( i > 2000) cout << "Error: Can't place a joint" << endl;
			}
			else
			{
				i++;
			}

		}

		// gets the inital location of the bodies correct
		kinematics(0);
			m_tree.print();
		return true;
	}

	std::vector<double> Model2::getState( )
	{
		// go through tree to get the state_vector
		vector<double> state_vector = m_tree.getState( );;

		// pass the vector of doubles out
		return state_vector;
	}

	std::vector<double> Model2::getDot( )
	{
		return m_tree.getDot();
	}

	Tree *Model2::getTree()
	{
		return &m_tree;
	}

	void Model2::setState(std::vector<double> state_vector )
	{
//	    cout << "set state" <<  std::endl;


//        for (std::vector<double>::const_iterator i = state_vector.begin(); i != state_vector.end(); ++i)
//            std::cout << *i << ' ';

		m_tree.setState( state_vector );

	}

	void Model2::addTree( Tree tree )
	{
		m_tree = tree;
	}

	Vect3 Model2::getBodyPosition( string bodyname )
	{
		Vect3 position(0,0,0);
		string name;
		std::vector< BodyRigid *>::reverse_iterator rit;
		for (rit = m_rigidBodies.rbegin(); rit < m_rigidBodies.rend(); ++rit )
		{
			if (bodyname == (*rit)->m_name)
			{
				position = (*rit)->m_pos;
			}
		}
		return position;
	}

	Mat3x3 Model2::getBodyRot( string bodyname )
	{
		Mat3x3 rot;
		rot <<	0,0,0,
				0,0,0,
				0,0,0;

		string name;
		std::vector< BodyRigid *>::reverse_iterator rit;
		for (rit = m_rigidBodies.rbegin(); rit < m_rigidBodies.rend(); ++rit )
		{
			if (bodyname == (*rit)->m_name)
			{
				rot = (*rit)->getRot();
			}
		}
		return rot;
	}

    // TODO: check make sure it is actually a translational Joint
    JointTranslational * Model2::getJointTranslational( string jointname )
    {
        JointTranslational * jnt = dynamic_cast<JointTranslational*> (getJoint(jointname));
        return jnt;
    }

    // TODO: check make sure it is actually a Revolute Joint
    JointRevolute * Model2::getJointRevolute( string jointname )
    {
        JointRevolute * jnt = dynamic_cast<JointRevolute*> (getJoint(jointname));
        return jnt;
    }

	Joint* Model2::getJoint( string jointname )
	{
		for (map<Joint1DOF *, BodyRigid *>::iterator i=m_child_map.begin(); i!=m_child_map.end(); ++i)
		{
			if (jointname == (i->first)->getName())
			{
				return (i->first);
			}
		}
		return NULL;
	}

    // TODO: check make sure it is actually a Force1Body
    Force1Body* Model2::getForce1Body( string forcename )
    {
        Force1Body * frc = dynamic_cast<Force1Body*> (getForce(forcename));
        return frc;
    }

    ForceRevJnt* Model2::getForceRevJnt( string forcename )
    {
        ForceRevJnt * frc = dynamic_cast<ForceRevJnt*> (getForce(forcename));
        return frc;
    }

	Force* Model2::getForce( string forcename )
	{

		string name;
		std::vector< Force *>::reverse_iterator rit;
		for (rit = m_forces.rbegin(); rit < m_forces.rend(); ++rit )
		{
			if (forcename == (*rit)->getName())
			{
				return (*rit);
			}
		}
		return NULL;
	}

	unsigned int Model2::getStateSize()
	{
		return m_tree.getStateSize();
	}




}
