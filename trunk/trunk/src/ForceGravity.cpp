#include "ForceGravity.h"

namespace OMD
{
ForceGravity::ForceGravity(std::string const &name, double g, Vect3 const &direction): m_g(g), Force(name), m_direction(direction)
{
}

ForceGravity::ForceGravity(std::string const &name, Vect3 const &v ): Force(name)
{
	m_g = v.norm();
	m_direction = v.normalized();
}

ForceGravity::ForceGravity(std::string const &name, double g, vector<double> const &direction): m_g(g), Force(name)
{
	m_direction << direction[0], direction[1], direction[2];
}

ForceGravity::~ForceGravity(void)
{
}

void ForceGravity::addBodyRigid(BodyRigid *rigidBody)
{
		std::vector<BodyRigid *>::iterator itb;

			bool thereAlready = false;
			for (std::vector<BodyRigid *>::iterator itb2=m_rigidBodies.begin(); itb2 < m_rigidBodies.end(); itb2++)
			{
				if (itb == itb2)
				{
					thereAlready = true;
					break;
				}
			}
			if (!thereAlready)
			{
				m_rigidBodies.push_back(rigidBody);
			}
}

void ForceGravity::addRigidBodies(std::vector<BodyRigid *> rigidBodies)
{

		std::vector<BodyRigid *>::iterator itb;
		for (itb = rigidBodies.begin(); itb < rigidBodies.end(); itb++)
		{
			bool thereAlready = false;
			for (std::vector<BodyRigid *>::iterator itb2=m_rigidBodies.begin(); itb2 < m_rigidBodies.end(); itb2++)
			{
				if (*itb == *itb2)
				{
					thereAlready = true;
					break;
				}
			}
			if (!thereAlready)
			{
				m_rigidBodies.push_back(*itb);
			}
		}

}

void ForceGravity::apply(double t)
{
	Vect3 forceVectNoMass = m_g * m_direction;
	std::vector<BodyRigid *>::iterator itb;
	for (itb = m_rigidBodies.begin(); itb < m_rigidBodies.end(); itb++)
	{
		BodyRigid * bodyRigid = *itb;
		Vect3 forceVect = forceVectNoMass * bodyRigid->m_mass;
		bodyRigid->forceAccum(forceVect,false);
	}
}

}