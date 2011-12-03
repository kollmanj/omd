#include "JointAxis.h"

namespace OMD
{

	JointAxis::JointAxis(string const &name,
				BodyRigid *parent,
				vector<double> const &parent2joint,
				BodyRigid *child,
				vector<double> const &joint2child,
				vector<double> const &axisInParent):
	Joint(name,parent,parent2joint,child,joint2child)
	{
		m_axis_pk << axisInParent[0], axisInParent[1], axisInParent[2];
		m_axis_pk.normalize(); // normalize in place

		// TODO: do this quaternions
		pk_c_k0 = parent->getRot().transpose()*child->getRot();	// from k to p[k]
		m_axis_k = pk_c_k0.transpose() * m_axis_pk;
	}

	JointAxis::JointAxis(string const &name,
		BodyRigid *parent,
		Vect3 parent2joint,
		BodyRigid *child,
		Vect3 joint2child,
		Vect3 axisInParent):
	Joint(name,parent,parent2joint,child,joint2child)
	{
		axisInParent.normalize(); // normalize in place
		m_axis_pk = axisInParent;

		// TODO: do this quaternions
		pk_c_k0 = parent->getRot().transpose()*child->getRot();	// from k to p[k]
		m_axis_k = pk_c_k0.transpose() * m_axis_pk;
	}

	JointAxis::JointAxis(string const &name,
		BodyRigid *parent,
		Vect3 parent2joint,
		Vect3 axisInParent,
		BodyRigid *child,
		Vect3 joint2child,
		Vect3 axisInChild):
	Joint(name,parent,parent2joint,child,joint2child)
	{
		axisInParent.normalize(); // normalize in place
		m_axis_pk = axisInParent;
		m_axis_k = axisInChild;
	}

	JointAxis::JointAxis(string const &name,
				BodyRigid *parent,
				vector<double> const &parent2joint,
				vector<double> const &axisInParent,
				BodyRigid *child,
				vector<double> const &joint2child,
				vector<double> const &axisInChild):
	Joint(name,parent,parent2joint,child,joint2child)
	{
		m_axis_pk << axisInParent[0], axisInParent[1], axisInParent[2];
		m_axis_pk.normalize(); // normalize in place

		m_axis_k << axisInChild[0], axisInChild[1], axisInChild[2];
	}


	MatNxN JointAxis::getConstraintViolationP22() const
	{
		Vect3 si = getParentAxisInGlobal();
		Vect3 sipl = getParent2JointGlobal();
		Vect3 sjpl = getChild2JointLocal();
		Vect3 d = m_child->getPosition(sjpl) - m_parent->getPosition(sipl);
		MatNxN yet2DropRow = skew(si)*d;
		unsigned int row2drop = row2Drop(si);
		return dropRow(yet2DropRow, row2drop);
	}

	MatNxN JointAxis::getConstraintViolationP12() const
	{
		Vect3 si = getParentAxisInGlobal();
		Vect3 sj = getChildAxisInGlobal();
		MatNxN si_s = skew(si);
		MatNxN sj_s = skew(sj);
		MatNxN yet2DropRow = si_s * sj;
		//        MatNxN yet2DropRow = sj_s * si;
		//        std::cout << "yet2DropRow: "<<yet2DropRow << std::endl;
		// need to drop a row, method described page 189 Nikravesh
		unsigned int row2drop = row2Drop(si);
		//        std::cout << "si: " << si << std::endl;
		//        std::cout << "sj: " << sj << std::endl;
		//        std::cout << "after drop: "<<yet2DropRow.dropRow(row2drop) << std::endl;
		return dropRow(yet2DropRow, row2drop);
		//return yet2DropRow;
	}

	MatNxN JointAxis::getJacobianModP22(int ParentOrChild)
	{
		unsigned int row2drop;
		if (ParentOrChild == PARENT)
		{
			Vect3 si = getParentAxisInGlobal();
			MatNxN si_s = skew(si);

			MatNxN Phi_ri = -1 * si_s;

			Vect3 sj = getChildAxisInGlobal();
			MatNxN sj_s = skew(sj);

			Vect3 sip = getParent2JointGlobal();
			Mat3x3 sip_s = skew(sip);

			Vect3 sipl = getParent2JointGlobal();
			Vect3 sjpl = getChild2JointLocal();
			Vect3 d = m_child->getPosition(sjpl) - m_parent->getPosition(sipl);
			Mat3x3 Ai = m_child->getRot();

			MatNxN Phi_pi = (si_s*sip_s + skew(d)*si_s) * Ai;

			// need to drop a row, method described page 189 Nikravesh
			row2drop = row2Drop(si);

			MatNxN temp = concatH(Phi_ri, Phi_pi);

			MatNxN result = dropRow(temp,row2drop);

			return result;
		}
		else
		{
			Mat3x3 Aj = m_child->getRot();

			Vect3 si = getParentAxisInGlobal();

			Vect3 sjp = getChild2JointGlobal();
			Mat3x3 sjp_s = skew(sjp);

			MatNxN Phi_rj = skew(si);

			MatNxN Phi_pj = -1*skew(si)*sjp_s*Aj;

			// need to drop a row, method described page 189 Nikravesh
			row2drop = row2Drop(si);

			MatNxN temp = concatH(Phi_rj, Phi_pj);

			MatNxN result = dropRow(temp,row2drop);
			return result;
			//return temp;
		}
	}

	MatNxN JointAxis::getJacobianModP12(int ParentOrChild)
	{
		unsigned int row2drop;
		if (ParentOrChild == PARENT)
		{
			Mat3x3 Ai = m_parent->getRot();

			Vect3 si = getParentAxisInGlobal();
			Vect3 sj = getChildAxisInGlobal();

			MatNxN si_s = skew(si);
			MatNxN sj_s = skew(sj);
			MatNxN part1(3,3);   // zeros
			part1.fill(0);
			MatNxN part2 = sj_s*si_s*Ai;

			// need to drop a row, method described page 189 Nikravesh
			row2drop = row2Drop(si);

			MatNxN temp = concatH(part1,part2);

			MatNxN result = dropRow(temp,row2drop);

			return result;
			//return temp;
		}
		else
		{
			Mat3x3 Aj = m_child->getRot();

			Vect3 si = getParentAxisInGlobal();
			Vect3 sj = getChildAxisInGlobal();

			MatNxN si_s = skew(si);
			MatNxN sj_s = skew(sj);
			MatNxN part1(3,3);  // zeros
			part1.fill(0);
			MatNxN part2 = -1*si_s * sj_s * Aj;

			// need to drop a row, method described page 189 Nikravesh
			//unsigned int row2drop = row2Drop(si);
			row2drop = row2Drop(si);

			// concatenate Horizontally
			MatNxN temp = concatH(part1,part2);

			MatNxN result = dropRow(temp,row2drop);
			////MatNxN result = part1.concatH(part2);
			return result;
			//return temp;
		}
	}

	VectN JointAxis::getGamaPoundN11a()
	{
		Vect3 sil = getParentAxisInLocal();
		MatNxN si_dot = m_parent->getVectorDot(sil);
		Vect3 sjl = getChildAxisInLocal();
		MatNxN sj_dot = m_child->getVectorDot(sjl);
		Vect3 si = getParentAxisInGlobal();
		Vect3 sj = getChildAxisInGlobal();

		Vect3 i_omega_global = m_parent->getAngularVelocityGlobal();
		Mat3x3 i_omega_global_s = skew(i_omega_global);
		Vect3 j_omega_global = m_child->getAngularVelocityGlobal();
		Mat3x3 j_omega_global_s = skew(j_omega_global);

		return -2*si_dot.transpose()*sj_dot + si_dot.transpose()*i_omega_global_s*sj + sj_dot.transpose()*j_omega_global_s*si;
	}

	VectN JointAxis::getGamaPoundN11b()
	{
		std::vector<Vect3> hiandhj = getHiHj();
		Vect3 hi = hiandhj[0];
		Vect3 hj = hiandhj[1];

		Vect3 hil = m_parent->m_q.inverse() *hi;
		Vect3 hjl = m_child->m_q.inverse() *hj;
		MatNxN hi_dot = m_parent->getVectorDot(hil);
		MatNxN hj_dot = m_child->getVectorDot(hjl);

		Vect3 i_omega_global = m_parent->getAngularVelocityGlobal();
		Mat3x3 i_omega_global_s = skew(i_omega_global);
		Vect3 j_omega_global = m_child->getAngularVelocityGlobal();
		Mat3x3 j_omega_global_s = skew(j_omega_global);

		return -2*hi_dot.transpose()*hj_dot + hi_dot.transpose()*i_omega_global_s*hj + hj_dot.transpose()*j_omega_global_s*hi;
	}

	VectN JointAxis::getGamaPoundP22()
	{
		Vect3 sil = getParentAxisInLocal();
		Vect3 si_dot = m_parent->getVectorDot(sil);
		Mat3x3 si_dot_s = skew(si_dot);
		Vect3 si = getParentAxisInGlobal();
		Mat3x3 si_s = skew(si);

		Vect3 sipl = getParent2JointLocal();
		Vect3 sjpl = getChild2JointLocal();
		Vect3 d_dot = m_child->getVelocityGlobal(sjpl) - m_parent->getVelocityGlobal(sipl);

		Vect3 i_omega_global = m_parent->getAngularVelocityGlobal();
		Mat3x3 i_omega_global_s = skew(i_omega_global);

		Vect3 j_omega_global = m_child->getAngularVelocityGlobal();
		Mat3x3 j_omega_global_s = skew(j_omega_global);

		Vect3 sip_dot = m_parent->getVectorDot(sipl);
		Vect3 sjp_dot = m_parent->getVectorDot(sjpl);

		Vect3 d = m_child->getPosition(sjpl) - m_parent->getPosition(sipl);
		Mat3x3 d_s = skew(d);

		MatNxN yet2DropRow = -2*si_dot_s*d_dot + si_s*(i_omega_global_s*sip_dot - skew(j_omega_global)*sjp_dot) + d_s*i_omega_global_s*si_dot;
		unsigned int row2drop = row2Drop(si);
		return dropRow(yet2DropRow, row2drop);
		//return yet2DropRow;
	}

	VectN JointAxis::getGamaPoundP12()
	{
		Vect3 sil = getParentAxisInLocal();
		Vect3 si_dot = m_parent->getVectorDot(sil);
		Mat3x3 si_dot_s = skew(si_dot);
		Vect3 sjl = getChildAxisInLocal();
		Vect3 sj_dot = m_child->getVectorDot(sjl);
		Vect3 sj = getChildAxisInGlobal();
		Mat3x3 sj_s = skew(sj);
		Vect3 i_omega_global = m_parent->getAngularVelocityGlobal();
		Mat3x3 i_omega_global_s = skew(i_omega_global);
		Vect3 j_omega_global = m_child->getAngularVelocityGlobal();
		Mat3x3 j_omega_global_s = skew(j_omega_global);
		Vect3 si = getParentAxisInGlobal();
		Mat3x3 si_s = skew(si);

		MatNxN yet2DropRow = -2.0 * si_dot_s * sj_dot + ( sj_s * i_omega_global_s * si_dot)-(si_s * j_omega_global_s * sj_dot);
		// need to drop a row, method described page 189 Nikravesh
		unsigned int row2drop = row2Drop(si);
		//return yet2DropRow;
		return dropRow(yet2DropRow, row2drop);
		//return yet2DropRow;
	}

	MatNxN JointAxis::getJacobianP12(int ParentOrChild)
	{
		unsigned int row2drop;
		if (ParentOrChild == PARENT)
		{
			MatNxN Ci = getC(PARENT);
			Vect3 sj = getChildAxisInGlobal();
			Vect3 si = getParentAxisInGlobal();
			Mat3x3 sjskew = skew(sj);

			MatNxN zero(3,3);
			zero.fill(0);

			MatNxN Phi_pi = -1 * sjskew * Ci;
			row2drop = row2Drop(si);

			MatNxN yet2DropRow = concatH(zero,Phi_pi);

			return dropRow(yet2DropRow, row2drop);
			//return yet2DropRow;
		}
		else
		{
			MatNxN Cj = getC(CHILD);
			Vect3 si = getParentAxisInGlobal();
			Mat3x3 siskew = skew(si);

			MatNxN zero(3,3);
			zero << 0,0,0,
					0,0,0,
					0,0,0;

			MatNxN Phi_pj = skew(si)*Cj;
			row2drop = row2Drop(si);

			MatNxN yet2DropRow = concatH(zero, Phi_pj);
			return dropRow(yet2DropRow, row2drop);
		}
	}

	std::vector<Vect3> JointAxis::getHiHj() const
	{
		Vect3 si = getParentAxisInGlobal();
		Vect3 sj = getChildAxisInGlobal();
		// find a vector not co-linear with si
		// first try x axis of body 1 but if x does not work use y
		Vect3 p_xaxis = m_parent->getRot() * Vect3(1,0,0);
		Vect3 p_yaxis = m_parent->getRot() * Vect3(0,1,0);
		Vect3 p_zaxis = m_parent->getRot() * Vect3(0,0,1);
		Vect3 a1,a2;
		if (fabs(si.dot(p_xaxis)) < 0.9)
		{
			a1 = p_xaxis;
			if (fabs(sj.dot(p_yaxis)) < 0.9)
			{
				a2 = p_yaxis;
			}
			else
			{
				a2 = p_zaxis;
			}
		}
		else
		{
			a1 = p_yaxis;
			if (fabs(sj.dot(p_xaxis)) < 0.9)
			{
				a2 = p_xaxis;
			}
			else
			{
				a2 = p_zaxis;
			}
		}
		Vect3 hi = si.cross(a1);
		Vect3 hj = sj.cross(a2);
		std::vector<Vect3> out;
		out.push_back(hi);
		out.push_back(hj);
		return out;
	}

	MatNxN JointAxis::getConstraintViolationN11a() const
	{
		Vect3 si = getParentAxisInGlobal();
		Vect3 sj = getChildAxisInGlobal();
		double dotProductsisj = si.dot(sj);
		MatNxN out(1,1);
		out(0,0) = dotProductsisj;
		return out;
	}

	MatNxN JointAxis::getConstraintViolationN11b() const
	{
		std::vector<Vect3> hiandhj = getHiHj();
		Vect3 hi = hiandhj[0];
		Vect3 hj = hiandhj[1];
		double dotProducthihj = hi.dot(hj);
		MatNxN out(1,1);
		out(0,0) = dotProducthihj;
		return out;
	}

	MatNxN JointAxis::getJacobianModN11a(int ParentOrChild)
	{
		MatNxN si = getParentAxisInGlobal();
		MatNxN sj = getChildAxisInGlobal();

		if (ParentOrChild == PARENT)
		{
			//MatNxN zero(3,3);
			//zero.fill(0);
			Mat3x3 Ai = m_parent->getRot();

			MatNxN Phi_ri(1,3);
			Phi_ri.fill(0);
			MatNxN Phi_pi = -1*sj.transpose()*skew(si)*Ai;

			MatNxN out = concatH(Phi_ri, Phi_pi);

			return out;
		}
		else
		{
			//MatNxN zero(3,3);
			//zero.fill(0);
			Mat3x3 Aj = m_child->getRot();

			MatNxN Phi_rj(1,3);
			Phi_rj.fill(0);
			MatNxN Phi_pj = -1*si.transpose()*skew(sj)*Aj;

			MatNxN out = concatH(Phi_rj, Phi_pj);

			return out;
		}
	}

	MatNxN JointAxis::getJacobianModN11b(int ParentOrChild)
	{
		std::vector<Vect3> hiandhj = getHiHj();
		MatNxN hi = hiandhj[0];
		MatNxN hj = hiandhj[1];

		if (ParentOrChild == PARENT)
		{
			MatNxN zero(3,3);
			zero.fill(0);
			Mat3x3 Ai = m_parent->getRot();

			MatNxN Phi_ri(1,3);
			Phi_ri.fill(0);
			MatNxN Phi_pi = -1*hj.transpose()*skew(hi)*Ai;

			MatNxN out = concatH(Phi_ri, Phi_pi);

			return out;
		}
		else
		{
			MatNxN zero(3,3);
			zero.fill(0);
			Mat3x3 Aj = m_child->getRot();

			MatNxN Phi_rj(1,3);
			Phi_rj.fill(0);
			MatNxN Phi_pj = -1*hi.transpose()*skew(hj)*Aj;

			MatNxN out = concatH(Phi_rj, Phi_pj);

			return out;
		}
	}

	MatNxN JointAxis::getJacobianN11a(int ParentOrChild)
	{
		MatNxN si = getParentAxisInGlobal();
		MatNxN sj = getChildAxisInGlobal();

		if (ParentOrChild == PARENT)
		{
			MatNxN Ci = getC(PARENT);
			MatNxN z(1,3);
			z.fill(0);
			MatNxN Phi_pi = sj.transpose()*Ci;
			
			MatNxN out = concatH(z, Phi_pi);
			return out;
		}
		else
		{
			MatNxN Cj = getC(CHILD);
			MatNxN z(1,3);
			z.fill(0);
			MatNxN Phi_pj = si.transpose()*Cj;

			MatNxN out = concatH(z, Phi_pj);
			return out;
		}
	}

	MatNxN JointAxis::getJacobianN11b(int ParentOrChild)
	{
		std::vector<Vect3> hiandhj = getHiHj();
		MatNxN hi = hiandhj[0];
		MatNxN hj = hiandhj[1];
		if (ParentOrChild == PARENT)
		{
			MatNxN Ci = getC2(PARENT);
			MatNxN z(1,3);
			z.fill(0);
			MatNxN Phi_pi = hj.transpose()*Ci;

			MatNxN out = concatH(z, Phi_pi);
			return out;
		}
		else
		{
			MatNxN Cj = getC2(CHILD);
			MatNxN z(1,3);
			z.fill(0);
			MatNxN Phi_pj = hi.transpose()*Cj;

			MatNxN out = concatH(z, Phi_pj);
			return out;
		}
	}

	MatNxN JointAxis::getJacobianP22(int ParentOrChild)
	{
		unsigned int row2drop;
		if (ParentOrChild == PARENT)
		{
			MatNxN Ci = getC(PARENT);
			Vect3 sj = getChildAxisInGlobal();
			Vect3 si = getParentAxisInGlobal();
			Mat3x3 sjskew = skew(sj);
			Mat3x3 siskew = skew(si);

			Vect3 sipl = getParent2JointGlobal();
			Vect3 sjpl = getChild2JointLocal();
			Vect3 d = m_child->getPosition(sjpl) - m_parent->getPosition(sipl);

			Mat3x3 dskew = skew(d);

			MatNxN Bi = getB(PARENT);

			MatNxN Phi_ri = -1*siskew;

			MatNxN Phi_pi = -1*siskew*Bi - dskew*Ci;
			row2drop = row2Drop(si);

			MatNxN yet2DropRow = concatH(Phi_ri, Phi_pi);

			return dropRow(yet2DropRow, row2drop);
		}
		else
		{
			Vect3 si = getParentAxisInGlobal();
			Mat3x3 siskew = skew(si);

			Vect3 sipl = getParent2JointGlobal();
			Vect3 sjpl = getChild2JointLocal();
			Vect3 d = m_child->getPosition(sjpl) - m_parent->getPosition(sipl);

			Mat3x3 dskew = skew(d);

			MatNxN Bj = getB(CHILD);

			MatNxN Phi_ri = siskew;

			MatNxN Phi_pi = siskew*Bj;
			row2drop = row2Drop(si);

			MatNxN yet2DropRow = concatH(Phi_ri, Phi_pi);

			return dropRow(yet2DropRow, row2drop);
		}
	}

	MatNxN JointAxis::getC(int ParentOrChild)
	{
		BodyRigid *body;
		Vect3 spl;
		if (ParentOrChild == PARENT)
		{
			body = m_parent;
			spl = getParentAxisInLocal();
		}
		else
		{
			body = m_child;
			spl = getChildAxisInLocal();
		}

		Quat qt = body->m_q;
		Mat3x4 Gmat = G(qt);
		MatNxN splskew4x4 = skew4x4(spl);

		Vect4 temp(qt.w(), qt.x(), qt.y(), qt.z());

		return 2.0*(Gmat*splskew4x4 + spl*temp.transpose());
	}

	MatNxN JointAxis::getC2(int ParentOrChild)
	{
		BodyRigid *body;
		std::vector<Vect3> hiandhj = getHiHj();
		Vect3 hi = hiandhj[0];
		Vect3 hj = hiandhj[1];
		Vect3 sl;
		if (ParentOrChild == PARENT)
		{
			body = m_parent;
			sl = (m_parent->getRot()).transpose()*hi;
		}
		else
		{
			body = m_child;
			sl = (m_child->getRot()).transpose()*(hj);
		}

		Quat qt = body->m_q;
		Mat3x4 Gmat = G(qt);
		MatNxN slskew4x4 = skew4x4(sl);

		Vect4 temp(qt.w(), qt.x(), qt.y(), qt.z());

		return 2.0*(Gmat*slskew4x4 + sl*temp.transpose());
	}

}