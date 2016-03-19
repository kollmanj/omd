/********************************************************
 * Swig module description file for wrapping a C++ class.
 * Generate by saying "swig -python -shadow number.i".   
 * The C module is generated in file number_wrap.c; here,
 * module 'number' refers to the number.py shadow class.
 ********************************************************/
%module OMDlib
%{
//#include "OMD.h"
#include "OMDConfig.h"
#include "Body.h"
#include "BodyRigid.h"
#include "Joint.h"
#include "Joint1DOF.h"
#include "JointRevolute.h"
#include "JointTranslational.h"
#include "JointCylindrical.h"
#include "JointSpherical.h"
#include "JointUniversal.h"
#include "Branch.h"
#include "Force.h"
#include "Force1Body.h"
#include "ForceRevJnt.h"
#include "ForceTransJnt.h"
#include "ForceRevJntSpringDamp.h"
#include "Curve2D.h"
#include "Curve2DLinInterp.h"
#include "Curve2DSine.h"
#include "ForceTransJntSpringDamp.h"
#include "ForceRevJntPIDCurve2D.h"
#include "ForceGravity.h"
#include "ForceBuoyancy.h"
#include "Force2BodySpringDamp.h"
#include "Tree.h"
#include "Integrator.h"
#include "Model.h"
#include "Model1.h"
#include "Model2.h"
#include "Model3.h"
#include "IntegratorRK4.h"
#include "ForceContact.h"
//#include "example.h"
%}

%include "std_string.i"
%include "std_vector.i"
// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
//   %template(VecVecDouble) vector< vector<double> >;
}

//%include "example.h"
%include Curve2D.h
%include Curve2DSine.h
namespace OMD
{

/*
class Vect3 :
{
        public:
        Vect3(double x, double y, double z);
};
*/

class BodyRigid :
public Body
{
        public:
        BodyRigid(std::string const &name, 
                double const &mass,
                vector<double> const &inertia,
                vector<double> const &pos, 
                vector<double> const &q, 
                vector<double> const &vel,
                vector<double> const &wl,
                bool const &fixed = false);
        double getX() const;
        double getY() const;
        double getZ() const;
        double getXd() const;
        double getYd() const;
        double getZd() const;
        double getE0() const;
        double getE1() const;
        double getE2() const;
        double getE3() const;
        double getWlx() const;
        double getWly() const;
        double getWlz() const;
        std::vector<double> getXAxis() const;
        std::vector<double> getYAxis() const;
        std::vector<double> getZAxis() const;
};

class ForceContact : public Force
{
public:
        ForceContact ( std::string const& name, btCollisionObjectArray objects, double stiff, double damp, double frict, double thresh );
        ForceContact ( std::string const& name, btCollisionWorld *collisionWorld, double stiff, double damp, double frict, double thresh );
        ForceContact ( std::string const& name, double stiff, double damp, double frict, double thresh );
        void addBox(double x, double y, double z, std::vector<double> offset, std::vector<double> rot, BodyRigid *body=0);
        void addCapsule(double radius, double height, std::vector<double> offset, std::vector<double> rot, BodyRigid *body=0);
        void addSphere(double radius, std::vector<double> offset, std::vector<double> rot, BodyRigid *body=0);
        void addCylinder(double radius, double width, std::vector<double> offset, std::vector<double> rot, BodyRigid *body=0);
        void setCollisionMargin(double cm);
};

class Force1Body:
public Force
{
        public:
        Force1Body(std::string const &name, BodyRigid * body, 
                        vector<double> const &f,
                        vector<double> const &forceLocation,
                        bool const &forceIsLocal, 
                        vector<double> const &t, 
                        bool const &torqueIsLocal=true); 
        void setTorque(double x, double y, double z);
        void setForce(double x, double y, double z);
};

class ForceBuoyancy:
public Force
{
        public:
        ForceBuoyancy(std::string const &name, BodyRigid * body, 
                        vector<double> const &f,
                        vector<double> const &forceLocation,
                        bool const &forceIsLocal, 
                        vector<double> const &t, 
                        bool const &torqueIsLocal=true); 
        void setTorque(double x, double y, double z);
        void setForce(double x, double y, double z);
};

class ForceGravity:
public Force
{
        public:
        ForceGravity(std::string const &name,  
                        double g,
                        vector<double> const &direction);
};

class IntegratorEuler:
public Integrator
{JointRevolute* Model2::addJointRevolute(string const& name, std::string parentname, vector<double> parent2joint, std::string childname, vector<double> joint2child, vector<double> axis, double q0 = 0, double u0 =0);

        public:JointRevolute* Model2::addJointRevolute(string const& name, std::string parentname, vector<double> parent2joint, std::string childname, vector<double> joint2child, vector<double> axis, double q0 = 0, double u0 =0);

                IntegratorEuler(void);
                ~IntegratorEuler(void);
};

class IntegratorRK4:
public Integrator
{
        public:
                IntegratorRK4(void);
                ~IntegratorRK4(void);
};

class Force2BodySpringDamp:
public Force
{
        public:
                Force2BodySpringDamp( std::string const& name, BodyRigid *body1, BodyRigid *body2, double k, double c, double fl, vector<double> body1Offset, vector<double> body2Offset);
                ~Force2BodySpringDamp(void);
};


class Model1 :
public Model
{
        public:
        Model1(void);
        ~Model1(void);
        std::vector<double> solve( double t, bool storeAccels=false);
        std::vector<double> getState();
        BodyRigid* Model1::addBodyRigid(std::string const &name, double const &mass, std::vector<double> const &inertia, std::vector<double> const &pos, std::vector<double> q, std::vector<double> const &vel, std::vector<double> const &wl, bool const &fixed=false);
        Force1Body * Model1::addForceOnBody(std::string const &name, BodyRigid * body, std::vector<double> const &f, std::vector<double> const &forceLocation, bool const &forceLocal=true);
        ForceBuoyancy * Model1::addForceBuoyancy(std::string const &name, BodyRigid * body, std::vector<double> const &f, std::vector<double> const &forceLocation, bool const &forceLocal=true);
        Force1Body * Model1::addTorqueOnBody(std::string const &name, BodyRigid * body, std::vector<double> const &t, bool const &torqueLocal=true);
        Force2BodySpringDamp * Model1::addForce2BodySpringDamp ( std::string const& name, BodyRigid *body1, BodyRigid *body2, double k, double c, double fl, std::vector<double> body1Offset, std::vector<double> body2Offset);
        ForceGravity * Model1::addForceGravity(std::string const &name, double g, std::vector<double> const &direction);
        void integrate( double t0, double t1, bool storeBodyAccels=false);
        IntegratorRK4 * setIntegratorRK4();
        IntegratorEuler * setIntegratorEuler();
        ForceContact * addForceContact ( string const& name, double stiff, double damp, double frict, double thresh);
        BodyRigid *getBody( std::string bodyname );
};

class JointTranslational : 
public Joint1DOF
{
public:
JointTranslational(std::string const &name, BodyRigid * parent, std::vector<double> const &parent2joint, BodyRigid *child, std::vector<double> const &joint2child, std::vector<double> const &axis, double q0 = 0., double u0 = 0.);
};

class JointRevolute : public Joint1DOF
{
public:
JointRevolute ( std::string const& name, BodyRigid* parent, std::vector<double> parent2joint, BodyRigid* child, std::vector<double> joint2child, std::vector<double> axis,double q0=0, double u0=0 );
};

class JointSpherical : public Joint
{
public:
JointSpherical::JointSpherical ( std::string const& name, BodyRigid* parent, std::vector<double> parent2joint, BodyRigid* child, std::vector<double> joint2child,double q0=0, double u0=0 );
};

class JointCylindrical: public JointAxis
{
public:
JointCylindrical ( std::string const & name, BodyRigid* parent, std::vector<double> parent2joint, BodyRigid* child, std::vector<double> joint2child, std::vector<double> axis,double q0=0, double u0=0 );
};

class JointUniversal : public JointAxis
{
public:
JointUniversal ( std::string const& name, BodyRigid* parent, std::vector<double> parent2joint, std::vector<double> pAxis, BodyRigid* child, std::vector<double> joint2child, std::vector<double> cAxis );
};

class ForceRevJnt :
   virtual public Force
{
public:
   ForceRevJnt(std::string const& name,JointRevolute *jnt,double trq);
   ~ForceRevJnt(void);

   virtual void apply(double t);
   void SetTorque(double trq);
   void Add2Torque(double trq);
   JointRevolute * getJoint() {return m_joint;};
};

class ForceRevJntPIDCurve2D :
   virtual public Force
{
public:
   ForceRevJntPIDCurve2D(string const& name,JointRevolute * jnt,double p, double i, double d, Curve2D *curve);
};

class ForceRevJntSpringDamp :
   virtual public Force
{
public:
   ForceRevJntSpringDamp(string const& name,JointRevolute * jnt,double k, double c, double fl = 0);
   ~ForceRevJntSpringDamp(void);
};

class Model2 :
public Model1
{
        public:
        Model2(void);
        ~Model2(void);
        JointTranslational* Model2::addJointTranslational(std::string const &name, BodyRigid * parent, std::vector<double> const &parent2joint, BodyRigid *child, std::vector<double> const &joint2child, std::vector<double> const & axis, double q0 = 0., double u0 = 0.);
        JointTranslational* Model2::addJointTranslational(std::string const &name, std::string parentname, std::vector<double> parent2joint, std::string childname, std::vector<double> joint2child, std::vector<double> axis, double q0 = 0., double u0 = 0.);
        JointRevolute* Model2::addJointRevolute(std::string const& name, BodyRigid * parent, std::vector<double> parent2joint, BodyRigid* child, std::vector<double> joint2child, std::vector<double> axis, double q0=0, double u0=0);
        JointRevolute* Model2::addJointRevolute(std::string const& name, std::string parentname, std::vector<double> parent2joint, std::string childname, std::vector<double> joint2child, std::vector<double> axis, double q0 = 0, double u0 =0);
        ForceRevJnt* addForceRevJnt(std::string const& name, JointRevolute * jnt, double trq);
        bool Model2::buildTree();
        std::vector<double> solve( double t, bool storeAccels=false);
        BodyRigid* Model2::addBodyRigid (  std::string name, double mass, std::vector<double> inertia, std::vector<double> orientation, bool fixed = false);
        ForceRevJntPIDCurve2D * addForceRevJntPIDCurve2D(std::string const& name, JointRevolute * jnt, double p, double i, double d, Curve2DSine *curve);
        ForceRevJntSpringDamp * addForceRevJntSpringDamp(string const& name, JointRevolute *jnt, double k, double c, double fl=0);
};

class Model3 :
public Model1
{
        public:
        Model3(void);
        BodyRigid* Model3::addBodyRigid(std::string const &name, double const &mass, std::vector<double> const &inertia, std::vector<double> const &pos, std::vector<double> q, std::vector<double> const &vel, std::vector<double> const &wl, bool const &fixed=false);
        JointTranslational* addJointTrans(std::string const &name, BodyRigid * parent, std::vector<double> parent2joint, BodyRigid *child, std::vector<double> axis, double q0 = 0., double u0 = 0.);
        JointTranslational* addJointTrans(std::string const &name, std::string parentname, std::vector<double> parent2joint, std::string childname, std::vector<double> axis, double q0 = 0., double u0 = 0.);
        JointRevolute* addJointRevolute(std::string const& name, BodyRigid * parent, std::vector<double> parent2joint, BodyRigid* child, std::vector<double> axis, double q0 = 0, double u0 =0);
        JointRevolute* addJointRevolute(std::string const& name, std::string parentname, std::vector<double> parent2joint, std::string childname, std::vector<double> axis, double q0 = 0, double u0 =0);
        JointCylindrical* addJointCylindrical(std::string const& name, BodyRigid * parent, std::vector<double> parent2joint, BodyRigid* child, std::vector<double> axis, double q0 = 0, double u0 =0);
        JointSpherical* addJointSpherical( std::string const& name, BodyRigid * parent, std::vector<double> parent2joint, BodyRigid* child);
        JointUniversal* addJointUniversal( std::string const& name, BodyRigid* parent, std::vector<double> parent2joint, std::vector<double> pAxis, BodyRigid* child, std::vector<double> cAxis);
        JointUniversal* addJointUniversal( std::string const& name, std::string const& parentname, std::vector<double> parent2joint, std::vector<double> pAxis, std::string const& childname, std::vector<double> cAxis);
        ForceRevJnt* addForceRevJnt(std::string const& name, JointRevolute * jnt, double trq);
        ForceRevJntPIDCurve2D * addForceRevJntPIDCurve2D(std::string const& name, JointRevolute * jnt, double p, double i, double d, Curve2DSine *curve);
        ForceRevJntSpringDamp * addForceRevJntSpringDamp(string const& name, JointRevolute *jnt, double k, double c, double fl=0);
        void calcIndependentStates();
        std::vector<int> getIndependentIndices();
        void kinematicSolve(std::vector<double> indStates);
        ~Model3(void);
};

};

