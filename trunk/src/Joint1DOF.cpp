#include "Joint1DOF.h"

namespace OMD
{
	Joint1DOF::Joint1DOF(string const &name,
				BodyRigid *parent,
				Vect3 parent2joint,
				BodyRigid *child,
				Vect3 joint2child,
				Vect3 axisInParent,
				double q0,
				double u0 ):
				 JointAxis(name,parent,parent2joint,child,joint2child,axisInParent)
	{
//		axisInParent.normalise(); // normalize in place
//		m_axis_pk = axisInParent;
		m_q = q0;
		m_u = u0;

//        pk_c_k0 = parent->getRot().getTranspose()*child->getRot();	// from k to p[k]
//        m_axis_k = pk_c_k0.getTranspose() * m_axis_pk;
		m_sSC.setIdentity();
	}

	Joint1DOF::Joint1DOF(string const &name, BodyRigid *parent, vector<double> const &parent2joint,
				BodyRigid *child, vector<double> const &joint2child,vector<double> const &axisInParent,double q0, double u0 ):
					JointAxis(name,parent,parent2joint,child,joint2child,axisInParent)
	{
//		axisInParent.normalise(); // normalize in place
//		m_axis_pk = axisInParent;
		m_q = q0;
		m_u = u0;

//        pk_c_k0 = parent->getRot().getTranspose()*child->getRot();	// from k to p[k]
//        m_axis_k = pk_c_k0.getTranspose() * m_axis_pk;
		m_sSC.setIdentity();
	}

	Joint1DOF::~Joint1DOF()
	{
	}

    // implementation of equation 78 & 80 page 59
	void Joint1DOF::ForwardSubstitute( double t )
	{
		Vect6 sSCT_sAhat = m_sSC.transpose()*(m_parent->m_sAhat);

		Vect6 tempv6 = m_child->m_sIhat*sSCT_sAhat;

        tempv6 = tempv6 - m_child->m_sFhat;
		m_udot = m_neg_inv_sM_sP.dot(tempv6);
		m_child->m_sAhat = m_udot*m_P_;
		m_child->m_sAhat += sSCT_sAhat;
	}

	void Joint1DOF::triangularize(double t)
	{
		BodyRigid *parent = getParent();
		BodyRigid *child = getChild();

		Mat6x6 childsIhat = child->m_sIhat;
		Vect6 Ihat_P_ = childsIhat*m_P_;

		m_sM = m_P_.dot(Ihat_P_); // eq85
		m_invsM = 1.0 / m_sM;

		m_neg_inv_sM_sP = -m_invsM*m_P_;

        // outer product
        Mat6x6 temp = Ihat_P_ * m_neg_inv_sM_sP.transpose();

		Mat6x6 eye6x6;
		eye6x6.setIdentity();

		m_sT = m_sSC * (temp + eye6x6);

		Vect6 v6temp;
		v6temp = m_sT * child->m_sFhat; // part of eq 84  F is taken outside the "()"
		parent->m_sFhat += v6temp;  // eq 84 end

		Mat6x6 m_sSC_transpose = m_sSC.transpose();
//		m_sIhat_sSCT = child->m_sIhat * m_sSC_transpose;
		Mat6x6 sIhat_sSCT = child->m_sIhat * m_sSC_transpose;

//		temp = m_sT * m_sIhat_sSCT;
		temp = m_sT * sIhat_sSCT;
		parent->m_sIhat += temp;  // last part of equation 83 page 61
	}
   std::vector<double> Joint1DOF::getState( )
	{
      std::vector<double> state_vector;
      state_vector.push_back(m_q);
      state_vector.push_back(m_u);

		return state_vector;
	}
	unsigned int Joint1DOF::setState(std::vector<double> state_vector)
	{
		m_q = state_vector[0];
		m_u = state_vector[1];
      return 2;
	}
	std::vector<double> Joint1DOF::getDot( )
	{
      std::vector<double> dot_vector;
      dot_vector.push_back(m_qdot);
      dot_vector.push_back(m_udot);
      return dot_vector;
	}
	unsigned int Joint1DOF::getStateSize()
	{
		return 2;
	}

	void Joint1DOF::kinematics()
    {
        m_sSC = fillsSC(m_gamma_pk,pk_A_k);
    }
}