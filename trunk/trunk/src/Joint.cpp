#include "Joint.h"

namespace OMD
{

	Joint::Joint(std::string name)
	{
		m_name = name;

		m_sT << 1,0,0,0,0,0,
			0,1,0,0,0,0,
			0,0,1,0,0,0,
			0,0,0,1,0,0,
			0,0,0,0,1,0,
			0,0,0,0,0,1;
	}

	Joint::Joint(string name, BodyRigid *parent, vector<double> const &parent2joint, BodyRigid *child, vector<double> const &joint2child)
	{
		m_name = name;
		m_parent = parent;
		m_child = child;
		r_pk_j << parent2joint[0], parent2joint[1], parent2joint[2];
		m_r_j_k << joint2child[0], joint2child[1], joint2child[2];
		
		m_sT << 1,0,0,0,0,0,
			0,1,0,0,0,0,
			0,0,1,0,0,0,
			0,0,0,1,0,0,
			0,0,0,0,1,0,
			0,0,0,0,0,1;
	}

	Joint::Joint(std::string name, BodyRigid *parent, Vect3 &parent2joint, BodyRigid *child, Vect3 &joint2child)
	{
		m_name = name;
		m_parent = parent;
		m_child = child;
		r_pk_j = parent2joint;
		m_r_j_k = joint2child;


		m_sT << 1,0,0,0,0,0,
			0,1,0,0,0,0,
			0,0,1,0,0,0,
			0,0,0,1,0,0,
			0,0,0,0,1,0,
			0,0,0,0,0,1;
		//	m_sIhat_sSCT.set2Identity(); // can't be this
	}
	MatNxN Joint::getJacobianModS3(int ParentOrChild)
	{

		if (ParentOrChild == PARENT)
		{
			bool identity = true;
			//MatNxN part1(3,3,identity);
			MatNxN part1(3,3);
			part1 <<	1,0,0,
						0,1,0,
						0,0,1;

			Mat3x3 Ai = m_parent->getRot();

			Vect3 sip = getParent2JointGlobal();
			Mat3x3 sip_s = skew(sip);

			MatNxN part2 = -1 *sip_s * Ai;

			///  concatenate horizontally
			MatNxN temp;
			temp = concatH(part1,part2);

			return temp;
		}
		else
		{
			bool identity = true;
			MatNxN part1(3,3);
			part1 <<	-1, 0, 0,
						 0,-1, 0,
						 0, 0,-1;


			Mat3x3 Aj = m_child->getRot();
			Vect3 sjp = getChild2JointGlobal();
			Mat3x3 sjp_s = skew(sjp);

			MatNxN part2 =  sjp_s * Aj;

			//  concatenate horizontally
			MatNxN temp;
			temp = concatH(part1,part2);

			return temp;
		}
	};

	MatNxN Joint::getConstraintViolationS3() const
	{
		Vect3 sipl = getParent2JointLocal();
		Vect3 sjpl = getChild2JointLocal();

		MatNxN result = m_parent->getPosition(sipl) - m_child->getPosition(sjpl);
		return result;
	}

	MatNxN Joint::getJacobianS3(int ParentOrChild)
	{
		if (ParentOrChild == PARENT)
		{
			MatNxN B = getB(PARENT);
			//bool set2identity =true;
			MatNxN Phi_ri(3,3);
			Phi_ri <<	1,0,0,
				0,1,0,
				0,0,1;
			//  concatenate horizontally
			MatNxN out;
			out = concatH(Phi_ri,B);
			return out;

		}
		else
		{
			MatNxN B = getB(CHILD);
			//bool set2identity =true;
			//MatNxN Phi_ri(3,3,set2identity);
			MatNxN Phi_ri(3,3);
			Phi_ri <<	-1, 0, 0,
				0,-1, 0,
				0, 0,-1;
			//  concatenate horizontally
			MatNxN out;
			out = concatH(Phi_ri,-B);
			return out;
		}
	}

	VectN Joint::getGamaPoundS3()
	{
		Vect3 i_omega_global = m_parent->getAngularVelocityGlobal();
		Mat3x3 i_omega_global_s = skew(i_omega_global);
		Vect3 j_omega_global = m_child->getAngularVelocityGlobal();
		Mat3x3 j_omega_global_s = skew(j_omega_global);

		Vect3 sipl = getParent2JointLocal();
		Vect3 sip_dot = m_parent->getVectorDot(sipl);
		Vect3 sjpl = getChild2JointLocal();
		Vect3 sjp_dot = m_child->getVectorDot(sjpl);

		return -1*i_omega_global_s*sip_dot + j_omega_global_s*sjp_dot;
	}

	unsigned int Joint::row2Drop(Vect3 si) const
	{
		double row0 = fabs(si.z()) + fabs(si.y());
		double row1 = fabs(si.z()) + fabs(si.x());
		double row2 = fabs(si.y()) + fabs(si.x());

		if (row0 <= row1 && row0 <= row2)
		{
			//            std::cout << "drop row: " << 0 << std::endl;
			return 0;
		}
		else if ( row1 <= row2 )
		{
			//            std::cout << "drop row: " << 1 << std::endl;
			return 1;
		}
		else
		{
			//            std::cout << "drop row: " << 2 << std::endl;
			return 2;
		}
	}


	// page 201 table 7.1 Nikravesh
	MatNxN Joint::getB(int ParentOrChild)
	{
		Body *body;
		Vect3 spl;
		if (ParentOrChild == PARENT)
		{
			body = m_parent;
			spl = getParent2JointLocal();
		}
		else
		{
			body = m_child;
			spl = getChild2JointLocal();
		}
		Quat qt = body->m_q;
		Mat3x4 Gmat = G(qt);
		MatNxN splskew4x4 = skew4x4(spl);

		Vect4 temp(qt.w(),qt.x(),qt.y(),qt.z());

		return 2.0*(Gmat*splskew4x4 + spl*temp.transpose());
	}

}