#include "JointCylindrical.h"

namespace OMD
{

	JointCylindrical::JointCylindrical ( string const& name, BodyRigid* parent, Vect3 parent2joint, BodyRigid* child, Vect3 joint2child,Vect3 axis, double q0, double u0 ) : JointAxis ( name, parent, parent2joint, child, joint2child, axis)
	{
	}

	JointCylindrical::JointCylindrical ( std::string const & name, BodyRigid* parent, vector<double> parent2joint, BodyRigid* child, vector<double> joint2child, vector<double> axis,double q0, double u0 ) : JointAxis ( name, parent, parent2joint, child, joint2child, axis)
	{
	}

	JointCylindrical::~JointCylindrical()
	{
	}

	MatNxN JointCylindrical::getJacobian( int ParentOrChild )
	{

		MatNxN p12 = getJacobianP12(ParentOrChild);
		MatNxN p22 = getJacobianP22(ParentOrChild);
		//MatNxN n11 = getJacobianN11b(ParentOrChild);

		//std::cout << "p12: " << p12 << std::endl;
		//std::cout << "p22: " << p22 << std::endl;

		MatNxN out = concatV(p12,p22);
		return out;
	}

	MatNxN JointCylindrical::getJacobianModified(int ParentOrChild)
	{
		MatNxN p12 = getJacobianModP12(ParentOrChild);
		MatNxN p22 = getJacobianModP22(ParentOrChild);

		//std::cout << "p12: "<< std::endl << p12 << std::endl;
		//std::cout << "p22: "<< std::endl << p22 << std::endl;
		MatNxN out = concatV(p12,p22);
        return out;
	}

	vector<double> JointCylindrical::getViolation()
	{
		vector<double> out;
		MatNxN p12 = getConstraintViolationP12();
		MatNxN p22 = getConstraintViolationP22();

		out.push_back(p12(0,0));
		out.push_back(p12(1,0));
		out.push_back(p22(0,0));
		out.push_back(p22(1,0));
		return out;
	}

	VectN JointCylindrical::getGamaPound()
	{
		VectN p12 = getGamaPoundP12();
		VectN p22 = getGamaPoundP22();
		//MatNxN n11 = getGamaPoundN11b();

		VectN out = concatV(p12, p22);
		return out;
	}
}