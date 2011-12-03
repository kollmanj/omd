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
#include "ForceTransJntSpringDamp.h"

namespace OMD
{
   ForceTransJntSpringDamp::ForceTransJntSpringDamp(string const& name,
                            JointTranslational & jnt,
                            double k, double c, double fl):m_joint(jnt),
                                                            Force ( name ),
                                                            m_k(k),
                                                            m_c(c),
                                                            m_fl(fl)
{

}

ForceTransJntSpringDamp::~ForceTransJntSpringDamp(void)
{
}

	void ForceTransJntSpringDamp::apply(double t)
	{
      double scalarForce = (m_fl - m_joint.m_q)*m_k  - m_joint.m_u * m_c;
      Vect3 force_on_parent = m_joint.m_axis_pk * -scalarForce;
      Vect3 force_on_child = m_joint.m_axis_k*scalarForce;
      //m_joint.m_parent->ForceAccum(force_on_parent,m_joint.r_pk_j.crossProduct(force_on_parent));
      //m_joint.m_child->ForceAccum(force_on_child,-m_joint.m_r_j_k.crossProduct(force_on_child));
	  m_joint.m_parent->forceAccum(force_on_parent,true,m_joint.r_pk_j);
	  m_joint.m_child->forceAccum(force_on_child,true,m_joint.m_r_j_k);
   }

   void ForceTransJntSpringDamp::setStiffness(double k)
   {
      m_k= k;
   }

    void ForceTransJntSpringDamp::setDamping(double c)
   {
      m_c= c;
   }

   double ForceTransJntSpringDamp::getCompression()
   {
       return (m_fl - m_joint.m_q);
   }
}
