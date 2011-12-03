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
#ifndef OMDCURVE2DLININTERP_H
#define OMDCURVE2DLININTERP_H
#include "Curve2D.h"

#include <vector>
namespace OMD
{
//using namespace OMD;
	/**
		@author John Kollman
	*/
	class Curve2DLinInterp : virtual public Curve2D
	{
		public:
			Curve2DLinInterp ( string const& name = "name" );
			Curve2DLinInterp ( string const& name, std::vector<double> x, std::vector<double> y );
         void setIndependent(std::vector<double> x);
         void setDependent(std::vector<double> y);

			~Curve2DLinInterp();

			virtual double getDependent(double x);

		private:
         std::vector<double> mx;
         std::vector<double> my;

	};

}

#endif
