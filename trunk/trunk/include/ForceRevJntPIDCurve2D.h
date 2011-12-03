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
#ifndef OMDFORCEREVJNTPIDCURVE2D_H
#define OMDFORCEREVJNTPIDCURVE2D_H
#include "Force.h"
#include "JointRevolute.h"
#include "Curve2D.h"
#include <vector>
namespace OMD
{
class ForceRevJntPIDCurve2D :
   virtual public Force
{
public:
   ForceRevJntPIDCurve2D(string const& name,JointRevolute * jnt,double p, double i, double d, Curve2D *curve);
   ForceRevJntPIDCurve2D(string const& name,JointRevolute * jnt,double p, double i, double d, std::vector<Curve2D *> curves);
   ~ForceRevJntPIDCurve2D(void);

   virtual void apply(double t);
   double getError(){return m_e0;};

	private:
      JointRevolute *m_joint;
      double m_p,m_i,m_d,m_t0,m_e0;
//      Curve2D *m_curve;
	  std::vector<Curve2D *> m_curves;
};
}

#endif
