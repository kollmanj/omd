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
#ifndef _OMDSTATES_H_
#define _OMDSTATES_H_
//#include "OMDModel3.h"
#include "BodyRigid.h"
//#include "OMDMatrixNxN.h"
#include <vector>

namespace OMD
{
class States
{
public:
	States(std::vector<BodyRigid *> *bods);
//	inline void addBody( BodyRigid * body ) { m_bodies.push_back(body); };
	~States(void);
	std::vector< double > getIndependentDots(MatNxN allAccels);
	inline std::vector<int> getIndependentIndices(){return m_IndependentIndices;};
	inline std::vector<int> getDependentIndices(){return m_DependentIndices;};
    inline std::vector<int> getIndependentIndices2(){return m_IndependentIndices2;};
	inline std::vector<int> getDependentIndices2(){return m_DependentIndices2;};
	std::vector<double> getIndependentStates();
	void setStates(std::vector<double> state_vector, double dt);
	MatNxN getDependentPos();
	std::vector<double> getPos(std::vector<int> p);
	void setDependentPos(MatNxN pos);
	void setPos(std::vector<int> c, MatNxN pos);
	void setDependentVel(MatNxN vel);
	double getDot(unsigned int index);
	double getDot2(unsigned int index);
private:
	std::vector<int> m_IndependentIndices;
    std::vector<int> m_DependentIndices;
    std::vector<int> m_IndependentIndices2;
    std::vector<int> m_DependentIndices2;
protected:
   friend class Model3;
   std::vector<BodyRigid *> * m_bodies;
   void calcIndependentStates(MatNxN jacobian, MatNxN jacobianM);
};
};
#endif
