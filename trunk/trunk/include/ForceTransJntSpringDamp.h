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
#ifndef OMDFORCETRANSJNTSPRINGDAMP_H
#define OMDFORCETRANSJNTSPRINGDAMP_H
#include "Force.h"
#include "JointTranslational.h"

namespace OMD
{
class ForceTransJntSpringDamp :
   virtual public Force
{
public:
   ForceTransJntSpringDamp(string const& name,JointTranslational & jnt,double k, double c, double fl = 0);
   ~ForceTransJntSpringDamp(void);

   virtual void apply(double t);
   void setStiffness(double k);
   void setDamping(double c);
    double getCompression();

   private:
	JointTranslational &m_joint;
	double m_k;
	double m_c;
	double m_fl;
};
}

#endif
