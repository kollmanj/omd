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
#include "Curve2DLinInterp.h"
//#include <iostream>
namespace OMD
{

	Curve2DLinInterp::Curve2DLinInterp ( string const& name, std::vector<double> x, std::vector<double> y ) :
                                                                                                Curve2D(name),
                                                                                                mx(x),
                                                                                                my(y)
   {
   }

   Curve2DLinInterp::Curve2DLinInterp (string const& name ): Curve2D(name)
   {
   }

   void Curve2DLinInterp::setIndependent(std::vector<double> x)
   {
      mx = x;
   }

   void Curve2DLinInterp::setDependent(std::vector<double> y)
   {
      my=y;
   }

	Curve2DLinInterp::~Curve2DLinInterp()
	{
	}

   double Curve2DLinInterp::getDependent(double x)
   {
      vector<double>::iterator it;
      int i0 = -1;
      int i1 = -1;
      for (it=mx.begin(); it < mx.end(); it++)
      {
         if (x <= *it)
         {
            i1++;
            break;
         }
         i1++;
         i0++;
      }

      // we have several possibilities
      // 1.  x < anything in mx
      if ( i0 == -1 )
      {
         return my[0];
      }
      // 2. x is in the range of mx
      else if ( i1 != i0 )
      {
         double xdiff = mx[i1] - mx[i0];
         double ydiff = my[i1] - my[i0];
         return my[i0] + (x-mx[i0])*ydiff/xdiff;
      }
      // 3. x > anything in mx
      else
      {
         return my[my.size()-1];
      }
   }


}
