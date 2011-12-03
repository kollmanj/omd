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
#include "Curve2DSine.h"
#include <math.h>

namespace OMD
{

	Curve2DSine::Curve2DSine ( string const& name, double amp, double offset, double freq, double shift ) :
        Curve2D(name),
        m_amp(amp),
        m_offset(offset),
        m_freq(freq),
        m_shift(shift)
	{
	}


	Curve2DSine::~Curve2DSine()
	{
	}

   double Curve2DSine::getDependent(double x)
   {
      return m_offset + m_amp*sin(m_freq*(x + m_shift));
   }


}
