#ifndef OMD_OMD_H
#define OMD_OMD_H

#include <iostream>

/*! \mainpage OMD
 *
 * \section intro_sec Introduction
 *
 * This software is implements methodology put forth in: \n
 * Recursive Derivation of Explicit Equations of Motion For Efficient Dynamic/Control Simulation of Large Multibody Systems \n
 * PH. D Dissertation August 1990 \n
 * Author: Kurt S. Anderson \n
 * \n
 * This software also implements methodolgoy put forth in: \n
 * Computer-Aided Analysis of Mechanical Systems \n
 * by: Parvis E. Nikravesh
 * ISBN: 0-13-164220-0 025 \n
 * \n
 * This tool can be used to simulate multi-body dynamics, i.e. mechanical systems. \n
 * Bodies have mass and inertia and can be connected with joints restricting motion. \n
 * Forces can be added to act upon the bodies.
 * Contact is considered a force and is implemented using Bullet's Contact library. \n
 *
 * For most applictations the user will be working in the Model1, Model2 or Model3 class almost exclusively.
 * Use methods such as addJointTrans to add a translational joint. \n
 * Your application needs to instantiate the model and keep it around to access components of the model.
 * For example, use the Model method: getBody to get a pointer to the body then access Body methods such as \n
 * getRot() to get the rotation matrix for the body \n
 * \section install_sec Installation
 *
 * \subsection step1 Step 1: Download source
 *
 * etc...
 * @author John Kollman
 */
#include <cmath> // or math.h
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

namespace OMD
{
const double PI = 4.0*atan(1.0);
typedef Eigen::Quaternion<double> Quat;
typedef Eigen::Matrix<double,3,3> Mat3x3;
typedef Eigen::Matrix<double,3,4> Mat3x4;
typedef Eigen::Matrix<double,4,3> Mat4x3;
typedef Eigen::Matrix<double,6,6> Mat6x6;
typedef Eigen::Matrix<double,4,4> Mat4x4;
typedef MatrixXd MatNxN;
typedef Eigen::Vector3d Vect3;
typedef Eigen::Vector4d Vect4;
typedef Eigen::Matrix<double,6,1> Vect6;
typedef Eigen::Matrix<double,Dynamic,1> VectN;
typedef double Scalar;

//typedef Mat3x3(Quat(1,0,0,0)) eye;
#define LOCAL_COORD true;
#define GLOBAL_COORD false;

inline Mat3x3 define(Vect3 row1, Vect3 row2, Vect3 row3)
{
    Mat3x3 out;
    out(0,0)=row1(0); out(0,1)=row1(1); out(0,2)=row1(2);
    out(1,0)=row2(0); out(1,1)=row2(1); out(1,2)=row2(2);
    out(1,0)=row1(0); out(1,1)=row1(1); out(2,2)=row1(2);

    return out;
}

inline Mat3x3 skew(Vect3 const &a)
{
	Mat3x3 out;
	out(0,0) = 	  0; out(0,1) =-a(2); out(0,2) =  a(1);
	out(1,0) = a(2); out(1,1) =    0; out(1,2) = -a(0);
	out(2,0) =-a(1); out(2,1) = a(0); out(2,2) =     0;

	return out;
}

inline Mat3x3 eye()
{
	Mat3x3 out;
	out(0,0) = 	  1; out(0,1) =    0; out(0,2) =     0;
	out(1,0) =    0; out(1,1) =    1; out(1,2) =     0;
	out(2,0) =    0; out(2,1) =    0; out(2,2) =     1;

	return out;
}

inline Mat3x4 L(Quat const &q)
{
	Mat3x4 out;
	out(0,0) =-q.x(); out(0,1) = q.w(); out(0,2) = q.z(); out(0,3) =-q.y();
	out(1,0) =-q.y(); out(1,1) =-q.z(); out(1,2) = q.w(); out(1,3) = q.x();
	out(2,0) =-q.z(); out(2,1) = q.y(); out(2,2) =-q.x(); out(2,3) = q.w();

	return out;
}

inline Mat3x4 G(Quat const &q)
{
	Mat3x4 out;
	out(0,0) =-q.x(); out(0,1) = q.w(); out(0,2) =-q.z(); out(0,3) = q.y();
	out(1,0) =-q.y(); out(1,1) = q.z(); out(1,2) = q.w(); out(1,3) =-q.x();
	out(2,0) =-q.z(); out(2,1) =-q.y(); out(2,2) = q.x(); out(2,3) = q.w();

	return out;
}

/// equation 6.65 of Nikravesh
/// represented by a with line over it
inline Mat4x4 skew4x4(Vect3 const &v)
{
	Mat4x4 out(4,4);

	Vect4 top(0,-v.x(),-v.y(),-v.z());
	Vect3 side(v.x(),v.y(),v.z());

	Mat3x4 temp;
	temp << side,-skew(v);  //concatinate vertically

	out << top.transpose(),temp;       // concatinate horizonatlly

    return out;
}

inline Quat getQDot(Quat const &q, Vect3 const &wl)
{
	Vect4 temp = 0.5 * L(q).transpose() * wl;
	return Quat(temp(0),temp(1),temp(2),temp(3));
}

inline Mat3x3 getInverse(Mat3x3 const &m)
{
	double invdet = 1/ m.determinant();
    double m00 =  (m(1,1)*m(2,2)-m(2,1)*m(1,2))*invdet;
    double m01 = -(m(0,1)*m(2,2)-m(0,2)*m(2,1))*invdet;
    double m02 =  (m(0,1)*m(1,2)-m(0,2)*m(1,1))*invdet;
    double m10 = -(m(1,0)*m(2,2)-m(1,2)*m(2,0))*invdet;
    double m11 =  (m(0,0)*m(2,2)-m(0,2)*m(2,0))*invdet;
    double m12 = -(m(0,0)*m(1,2)-m(1,0)*m(0,2))*invdet;
    double m20 =  (m(1,0)*m(2,1)-m(2,0)*m(1,1))*invdet;
    double m21 = -(m(0,0)*m(2,1)-m(2,0)*m(0,1))*invdet;
    double m22 =  (m(0,0)*m(1,1)-m(1,0)*m(0,1))*invdet;
    Mat3x3 out;
	out << m00,m01,m02,
           m10,m11,m12,
           m20,m21,m22;
	return out;
}

inline Scalar magnitude(Vect3 const &v)
{
	return sqrt( v.x()*v.x() + v.y()*v.y() + v.z()*v.z() );
}

inline MatNxN dropRow(MatNxN a, int i)
{
	MatNxN out(a.rows()-1,a.cols());
	//out << a.topRows(i-1), a.bottomRows( a.rows() - i );
	out << a.topRows(i), a.bottomRows( a.rows() - i - 1 );
	return out;
}

inline MatNxN concatH(MatNxN a, MatNxN b)
{
	if (a.rows() == 0)
	{
		return b;
	}

	MatNxN out(a.rows(),a.cols() + b.cols());
	out << a,b;
	return out;
}

inline MatNxN concatH(MatNxN a, double b)
{
	MatNxN out(a.rows(),a.cols() + 1);
	out << a,b;
	return out;
}

inline MatNxN concatV(MatNxN a, MatNxN b)
{
	if (a.rows() == 0)
	{
		return b;
	}
	//  concatenate vertically
	MatNxN out(a.rows()+b.rows(),b.cols());

	//std::cout << "a: " << a << std::endl;
	//std::cout << "b: " << b << std::endl;

	out << a,b;
	return out;
}

inline MatNxN concatV(MatNxN a, double b)
{
	if (a.rows() == 0)
	{
		MatNxN out(1,1);
		out << b;
		return out;
	}

	MatNxN out(a.rows()+1, a.cols());
	out << a,b;
	return out;
}

inline MatNxN concatD(MatNxN a, MatNxN b)
{
	MatNxN zeros1(a.rows(),b.cols());
	zeros1.fill(0);
	MatNxN zeros2(b.rows(),a.cols());
	zeros2.fill(0);
	MatNxN temp1 = concatH(a,zeros1);
	MatNxN temp2 = concatH(zeros2,b);
	return concatV(temp1,temp2);
}

inline Mat6x6 fillsSC(const Vect3 &gama, const Mat3x3 &pk_c_k)
{
	Mat6x6 out;
	out<<   pk_c_k(0,0),  pk_c_k(0,1),		pk_c_k(0,2),	gama.y()*pk_c_k(2,0) - gama.z() * pk_c_k(1,0),  gama.y()*pk_c_k(2,1) - gama.z() * pk_c_k(1,1),  gama.y()*pk_c_k(2,2) - gama.z() * pk_c_k(1,2),
            pk_c_k(1,0),  pk_c_k(1,1),		pk_c_k(1,2),	gama.z()*pk_c_k(0,0) - gama.x() * pk_c_k(2,0),  gama.z()*pk_c_k(0,1) - gama.x() * pk_c_k(2,1),  gama.z()*pk_c_k(0,2) - gama.x() * pk_c_k(2,2),
            pk_c_k(2,0),  pk_c_k(2,1),		pk_c_k(2,2),	gama.x()*pk_c_k(1,0) - gama.y() * pk_c_k(0,0),  gama.x()*pk_c_k(1,1) - gama.y() * pk_c_k(0,1),  gama.x()*pk_c_k(1,2) - gama.y() * pk_c_k(0,2),
            0.0,           0.0,				0.0,			pk_c_k(0,0),									pk_c_k(0,1),									pk_c_k(0,2),
            0.0,           0.0,				0.0,			pk_c_k(1,0),									pk_c_k(1,1),									pk_c_k(1,2),
            0.0,           0.0,				0.0,			pk_c_k(2,0),									pk_c_k(2,1),									pk_c_k(2,2);
	return out;
}

}
#endif
