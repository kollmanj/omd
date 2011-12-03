/***************************************************************************
 *   Copyright (C) 2008 by John Kollman,,,                                 *
 *   opensourcemultibodydynamics@gmail.com                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "ForceRevJntSpringDamp.h"
#include "JointRevolute.h"
namespace OMD
{
   ForceRevJntSpringDamp::ForceRevJntSpringDamp(string const& name,
                            JointRevolute * jnt,
                            double k, double c, double fl):m_joint(jnt),
                                                            Force ( name ),
                                                            m_k(k),
                                                            m_c(c),
                                                            m_fl(fl)
{

}

ForceRevJntSpringDamp::~ForceRevJntSpringDamp(void)
{
}

	void ForceRevJntSpringDamp::apply(double t)
	{
      m_torque = (m_fl - m_joint->m_q)*m_k  - m_joint->m_u * m_c;

      Vect3 torque_on_parent = m_joint->m_axis_pk * -m_torque;
      Vect3 torque_on_child = m_joint->m_axis_k*m_torque;
      // can apply torque at cg becuase it dosn't matter for a rigid body
      //m_joint->m_parent->ForceAccum(Vect3(0,0,0),torque_on_parent);
      //m_joint->m_child->ForceAccum(Vect3(0,0,0),torque_on_child);
	  m_joint->m_parent->torqueAccum(torque_on_parent,true);
	  m_joint->m_child->torqueAccum(torque_on_child,true);
   }

   void ForceRevJntSpringDamp::SetStiffness(double k)
   {
      m_k= k;
   }

    void ForceRevJntSpringDamp::SetDamping(double c)
   {
      m_c= c;
   }

   double ForceRevJntSpringDamp::GetCompression()
   {
       return (m_fl - m_joint->m_q);
   }
}
