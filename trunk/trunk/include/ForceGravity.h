#ifndef OMD_FORCEGRAVITY_H
#define OMD_FORCEGRAVITY_H

#include "force.h"
#include "OMD.h"
#include <vector>
#include "BodyRigid.h"

namespace OMD
{
class ForceGravity :
	public Force
{
public:
	ForceGravity(std::string const &name, double g, Vect3 const &direction=Vect3(0,0,-1) );
	ForceGravity(std::string const &name, double g, vector<double> const &direction );
	~ForceGravity(void);

	void addBodyRigid(BodyRigid *rigidBody);
	void addRigidBodies(std::vector<BodyRigid *> rigidBodies);

	double m_g;
	Vect3 m_direction;

protected:
	void apply(double t);

private:
	std::vector<BodyRigid *> m_rigidBodies;
};
};
#endif