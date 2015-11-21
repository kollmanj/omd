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

#include "States.h"
#include <iostream>
//#include "OMDVector4.h"

namespace OMD
{

   States::States(std::vector<BodyRigid *> *bods):m_bodies(bods)
   {

   }

   States::~States(void)
   {
   }

   void States::setStates(std::vector<double> state_vector, double dt)
   {

//       ///// for debug
//       std::vector<double>::iterator itd;
//       int count = 0;
//       for (itd=state_vector.begin(); itd<state_vector.end(); ++itd)
//       {
//           std::cout << "s_" << count <<"_: " << (*itd) << std::endl;
//           count++;
//       }
//       ///// for debug
       // first just set the independent state vector that we got
        std::vector< int >::iterator it;
        unsigned int i=0;
        for (it = m_IndependentIndices2.begin(); it < m_IndependentIndices2.end(); ++it)
		{
		    int bodynumber = (*it)/6;
		    int bodydof = (*it)%6;
		    BodyRigid *b = (*m_bodies)[bodynumber];

		    switch (bodydof)
		    {
		        case 0:
                    //b->setXdot(state_vector[i]);
					b->m_vel(0) = state_vector[i];
                    break;
                case 1:
                    //b->setYdot(state_vector[i]);
					b->m_vel(1) = state_vector[i];
                    break;
                case 2:
                    //b->setZdot(state_vector[i]);
					b->m_vel(2) = state_vector[i];
                    break;
                case 3:
                    //b->setAngularVelocityXL(state_vector[i]);
					b->m_wl(0) = state_vector[i];
                    break;
                case 4:
                    //b->setAngularVelocityYL(state_vector[i]);
					b->m_wl(1) = state_vector[i];
                    break;
                case 5:
                    //b->setAngularVelocityZL(state_vector[i]);
					b->m_wl(2) = state_vector[i];
                    break;
                default:
                    std::cout << "error in setting independent states" << std::endl;
                    break;
		    }
		    i++; // process two states, velocity and position each loop
		}


        for (it = m_IndependentIndices.begin(); it < m_IndependentIndices.end(); ++it)
		{
		    int bodynumber = (*it)/7;
		    int bodydof = (*it)%7;
		    BodyRigid *b = (*m_bodies)[bodynumber];

		    switch (bodydof)
		    {
		        case 0:
                    //b->setX(state_vector[i]);
					b->m_pos(0) = state_vector[i];
                    break;
                case 1:
                    //b->setY(state_vector[i]);
					b->m_pos(1) = state_vector[i];
                    break;
                case 2:
                    //b->setZ(state_vector[i]);
					b->m_pos(2) = state_vector[i];
                    break;
                case 3:
                    //b->setE0(state_vector[i]);
					b->m_q.w() = state_vector[i];
                    break;
                case 4:
                    //b->setE1(state_vector[i]);
					b->m_q.x() = state_vector[i];
                    break;
                case 5:
                    //b->setE2(state_vector[i]);
					b->m_q.y() = state_vector[i];
                    break;
                case 6:
                    //b->setE3(state_vector[i]);
					b->m_q.z() = state_vector[i];
                    break;
                default:
                    std::cout << "error in setting independent states" << std::endl;
                    break;
		    }
		    i++; // process two states, velocity and position each loop
		}

		// we set the independent states
		// now determine dependent states based on step c.2 page 321
		// first estimate each independent state with tyalor series
		// but, if delta t = 0 then taylor series will not change estimate
		if (dt > 0.00001)
		{


//       ///// for debug
//       std::vector<int>::iterator itd;
//       int count = 0;
//       for (itd=m_DependentIndices.begin(); itd<m_DependentIndices.end(); ++itd)
//       {
//           std::cout << "s_" << count <<"_: " << (*itd) << std::endl;
//           count++;
//       }
//       ///// for debug


            std::vector< int >::iterator it2;
            for (it2 = m_DependentIndices.begin(); it2 < m_DependentIndices.end(); ++it2)
            {
                ///////
                int bodynumber = (*it2)/7;
                int bodydof = (*it2)%7;
                BodyRigid *b = (*m_bodies)[bodynumber];

                switch (bodydof)
                {
                    case 0:
                        /// TODO use acceleration when we put that in body
						//b->setX(b->getX() + dt*b->getXdot()); // taylor series approx
						b->m_pos.x() = b->m_pos.x() + dt * b->m_vel.x();
//						b->setX(b->getX() + dt*b->getXdot() + 0.5*dt*dt*b->getAccelGlobalX()); // taylor series approx
                        break;
                    case 1:
                        //b->setY(b->getY() + dt*b->getYdot());
						b->m_pos.y() = b->m_pos.y() + dt * b->m_vel.y();
//                        b->setY(b->getY() + dt*b->getYdot() + 0.5*dt*dt*b->getAccelGlobalY());
                        break;
                    case 2:
                        //b->setZ(b->getZ() + dt*b->getZdot());
						b->m_pos.z() = b->m_pos.z() + dt * b->m_vel.z();
//                        b->setZ(b->getZ() + dt*b->getZdot() + 0.5*dt*dt*b->getAccelGlobalZ());
                        break;
                    case 3:
						//b->setE0(b->getE0() + dt*((b->getQuatDot()).w));
						b->m_q.w() = b->m_q.w() + dt * b->getQuatDot().w();
//                        b->setE0(b->getE0() + dt*((b->getQuatDot()).w) + 0.5*dt*dt*b->getE0DotDot());
                        break;
                    case 4:
                        //b->setE1(b->getE1() + dt*((b->getQuatDot()).x));
						b->m_q.x() = b->m_q.x() + dt * b->getQuatDot().x();
//                        b->setE1(b->getE1() + dt*((b->getQuatDot()).x)+ 0.5*dt*dt*b->getE1DotDot());
                        break;
                    case 5:
                        //b->setE2( b->getE2() + dt*((b->getQuatDot()).y));
						b->m_q.y() = b->m_q.y() + dt * b->getQuatDot().y();
//                        b->setE2( b->getE2() + dt*((b->getQuatDot()).y)+ 0.5*dt*dt*b->getE2DotDot());
                        break;
                    case 6:
                        //b->setE3( b->getE3() + dt*((b->getQuatDot()).z));
						b->m_q.z() = b->m_q.z() + dt * b->getQuatDot().z();
//                        b->setE3( b->getE3() + dt*((b->getQuatDot()).z)+ 0.5*dt*dt*b->getE3DotDot());
                        break;
                    default:
                        std::cout << "error in setting dependent states" << std::endl;
                        break;
                }
                ///////
            }
		}
		//std::cout << "dt: " << dt << std::endl;

   }

   void States::calcIndependentStates(MatNxN jacobian, MatNxN jacobianM)
   {

              int bodycount=0;

			  /// TODO m_bodies.size() - m_fixedBodies (or something like that)
       for (int i=0; i < m_bodies->size(); i++)
       {
           BodyRigid *b = (*m_bodies)[i];
           if (!b->isFixed())
           {
               bodycount++;
           }
        }

//       std::cout << jacobian << std::endl;
	    //unsigned int bodyCount = m_bodies->size();
//        std::cout << jacobian << std::endl;
        //std::vector<int> pivots = (jacobian.transpose()).getPivot();
	    //std::vector<int> pivots;
		////
		Eigen::FullPivLU<Matrix<double,Dynamic,Dynamic> > flu(jacobian);
		//Eigen::ColPivHouseholderQR<Matrix<double,11,14>> ldlt_(jacobian);
		Eigen::Matrix<int,Dynamic,Dynamic> pivots_mat =  flu.permutationQ().indices() ;
		//std::cout << ldlt_.colsPermutation().indices() << std::endl;

        int numOPivots = pivots_mat.rows();
//        std::cout << "Number of Pivots: " << numOPivots << std::endl;
        /// independent dofs = m
        int m = bodycount*7 - jacobian.rows();
        m_IndependentIndices.clear();
        m_DependentIndices.clear();
        for ( int i = pivots_mat.rows()-1; i >= 0; i--)
        {
            if ( m_IndependentIndices.size() < m )
            {
                m_IndependentIndices.push_back(pivots_mat(i,0));
            }
            else
            {
                m_DependentIndices.push_back(pivots_mat(i,0));
            }
        }

//        /// for debug
//        std::vector< int >::iterator it3;
//        unsigned int count2 = 0;
//        for (it3=pivots.begin(); it3<pivots.end(); ++it3)
//        {
//            std::cout << "pivot " << count2 <<"= " << (*it3) << std::endl;
//            count2++;
//        }
        /// for debug
//
//        std::vector< int >::iterator it123;
//        unsigned int i=0;
//        for (it123 = m_IndependentIndices.begin(); it123 < m_IndependentIndices.end(); ++it123)
//		{
//		    std::cout << "independent State " << i << ": " << (*it123) << std::endl;
//		    i++;
//		}
//        /// for debug
//
//        /// for debug
//        std::vector< int >::iterator it4;
//        unsigned int i4=0;
//        for (it4 = m_DependentIndices.begin(); it4 < m_DependentIndices.end(); ++it4)
//		{
//		    std::cout << "Dependent State " << i4 << ": " << (*it4) << std::endl;
//		    i4++;
//		}
//        /// for debug

        //std::vector<int> pivots2 = (jacobianM.transpose()).getPivot();

		Eigen::FullPivLU < Matrix<double,Dynamic,Dynamic> > flu2(jacobianM);
		//Eigen::ColPivHouseholderQR<Matrix<double,11,14>> ldlt_(jacobian);
		Eigen::Matrix<int,Dynamic,1> pivots2_mat =  flu2.permutationQ().indices() ;

        int numOPivots2 = pivots2_mat.rows();
//        std::cout << "Number of Pivots: " << numOPivots << std::endl;
        /// independent dofs = m
        int m2 = bodycount*6 - jacobianM.rows();
        m_IndependentIndices2.clear();
        m_DependentIndices2.clear();
        for ( int i = numOPivots2-1; i >= 0; i--)
        {
            if ( m_IndependentIndices2.size() < m )
            {
                m_IndependentIndices2.push_back(pivots2_mat(i));
            }
            else
            {
                m_DependentIndices2.push_back(pivots2_mat(i));
            }
        }
//        /// for debug
//        std::vector< int >::iterator it3;
//        unsigned int count2 = 0;
//        for (it3=pivots.begin(); it3<pivots.end(); ++it3)
//        {
//            std::cout << "pivot " << count2 <<"= " << (*it3) << std::endl;
//            count2++;
//        }
        /// for debug
//
//        std::vector< int >::iterator it;
//        unsigned int i=0;
//        for (it = m_IndependentIndices2.begin(); it < m_IndependentIndices2.end(); ++it)
//		{
//		    std::cout << "independent State2 " << i << ": " << (*it) << std::endl;
//		    i++;
//		}
//        /// for debug
//
//        /// for debug
//        std::vector< int >::iterator it4;
//        unsigned int i4=0;
//        for (it4 = m_DependentIndices2.begin(); it4 < m_DependentIndices2.end(); ++it4)
//		{
//		    std::cout << "Dependent State 2" << i4 << ": " << (*it4) << std::endl;
//		    i4++;
//		}
//		int crap123 =0;
//        /// for debug
   }

   std::vector<double> States::getPos(std::vector<int> p)
   {
	   std::vector<double> pos;
	   std::vector< int >::iterator it;
        for (it = p.begin(); it < p.end(); ++it)
		{
			int bodynumber = (*it)/7;
		    int bodydof = (*it)%7;
		    BodyRigid *b = (*m_bodies)[bodynumber];

		    switch (bodydof)
		    {
		        case 0:
                    pos.push_back(b->m_pos.x());
                    break;
                case 1:
                    pos.push_back(b->m_pos.y());
                    break;
                case 2:
                    pos.push_back(b->m_pos.z());
                    break;
                case 3:
                    pos.push_back(b->m_q.w());
                    break;
                case 4:
                    pos.push_back(b->m_q.x());
                    break;
                case 5:
                    pos.push_back(b->m_q.y());
                    break;
                case 6:
                    pos.push_back(b->m_q.z());
                    break;
                default:
                    std::cout << "error in getting independent states" << std::endl;
                    break;
			}
		}
			return pos;
   }

   MatNxN States::getDependentPos()
   {
	    MatNxN pos(m_DependentIndices.size(),1);
	    std::vector< int >::iterator it;
		int count =0;
        for (it = m_DependentIndices.begin(); it < m_DependentIndices.end(); ++it)
		{
			int bodynumber = (*it)/7;
		    int bodydof = (*it)%7;
		    BodyRigid *b = (*m_bodies)[bodynumber];

		    switch (bodydof)
		    {
		        case 0:
                    pos(count,0) = b->m_pos.x();
                    break;
                case 1:
                    pos(count,0) = b->m_pos.y();
                    break;
                case 2:
                    pos(count,0) = b->m_pos.z();
                    break;
                case 3:
                    pos(count,0) = b->m_q.w();
                    break;
                case 4:
                    pos(count,0) = b->m_q.x();
                    break;
                case 5:
                    pos(count,0) = b->m_q.y();
                    break;
                case 6:
                    pos(count,0) = b->m_q.z();
                    break;
                default:
                    std::cout << "error in getting independent states" << std::endl;
                    break;
			}
			count ++;
		}
			return pos;
   }

   double States::getDot( unsigned int index )
   {
	  		int bodynumber = index/7;
		    int bodydof = index%7;
			BodyRigid *b = (*m_bodies)[bodynumber];

	  		switch (bodydof)
		    {
		        case 0:
                    return b->m_vel.x();
                    break;
                case 1:
					return b->m_vel.y();
                    break;
                case 2:
                    return b->m_vel.z();
                    break;
                case 3:
                    return (b->getQuatDot()).w();
                    break;
                case 4:
                    return (b->getQuatDot()).x();
                    break;
                case 5:
                    return (b->getQuatDot()).y();
                    break;
                case 6:
					return (b->getQuatDot()).z();
                    break;
                default:
                    std::cout << "error in getting dot" << std::endl;
					return 0;
                    break;
		    }



   }

   double States::getDot2( unsigned int index )
   {
	  		int bodynumber = index/6;
		    int bodydof = index%6;
			BodyRigid *b = (*m_bodies)[bodynumber];

	  		switch (bodydof)
		    {
		        case 0:
                    return b->m_vel.x();
                    break;
                case 1:
					return b->m_vel.y();
                    break;
                case 2:
                    return b->m_vel.z();
                    break;
                case 3:
                    return b->m_wl.x();
                    break;
                case 4:
                    return b->m_wl.y();
                    break;
                case 5:
                    return b->m_wl.z();
                    break;

                default:
                    std::cout << "error in getting dot" << std::endl;
					return 0;
                    break;
		    }
   }

   void States::setPos(std::vector<int> c, MatNxN pos)
   {
	   std::vector< int >::iterator it;
	   int i=0;
	   for (it = c.begin(); it < c.end(); ++it)
	   {
		   	int bodynumber = (*it)/7;
		    int bodydof = (*it)%7;
		    BodyRigid *b = (*m_bodies)[bodynumber];
			double p = pos(i,0);
		    switch (bodydof)
		    {
		        case 0:
					//b->setX(p);
					b->m_pos.x() = p;
                    break;
                case 1:
                    //b->setY(p);
					b->m_pos.y() = p;
                    break;
                case 2:
                    //b->setZ(p);
					b->m_pos.z() = p;
                    break;
                case 3:
					//b->setE0(p);
					b->m_q.w() = p;
                    break;
                case 4:
                    //b->setE1(p);
					b->m_q.x() = p;
                    break;
                case 5:
                    //b->setE2(p);
					b->m_q.y() = p;
                    break;
                case 6:
                    //b->setE3(p);
					b->m_q.z() = p;
                    break;
                default:
                    std::cout << "error in setting Dependent states" << std::endl;
                    break;
			}
			i++;
	   }
   }
   void States::setDependentPos(MatNxN pos)
   {
	    std::vector< int >::iterator it;
		int i=0;
        for (it = m_DependentIndices.begin(); it < m_DependentIndices.end(); ++it)
		{
			int bodynumber = (*it)/7;
		    int bodydof = (*it)%7;
		    BodyRigid *b = (*m_bodies)[bodynumber];
			double p = pos(i,0);
		    switch (bodydof)
		    {
		        case 0:
					//b->setX(p);
					b->m_pos.x() = p;
                    break;
                case 1:
                    //b->setY(p);
					b->m_pos.y() = p;
                    break;
                case 2:
                    //b->setZ(p);
					b->m_pos.z() = p;
                    break;
                case 3:
					//b->setE0(p);
					b->m_q.w() = p;
                    break;
                case 4:
                    //b->setE1(p);
					b->m_q.x() = p;
                    break;
                case 5:
                    //b->setE2(p);
					b->m_q.y() = p;
                    break;
                case 6:
                    //b->setE3(p);
					b->m_q.z() = p;
                    break;
                default:
                    std::cout << "error in setting Dependent states" << std::endl;
                    break;
			}
			i++;
		}
   }

      void States::setDependentVel(MatNxN vel)
   {
	    std::vector< int >::iterator it;
		int i=0;
        for (it = m_DependentIndices2.begin(); it < m_DependentIndices2.end(); ++it)
		{
			int bodynumber = (*it)/6;
		    int bodydof = (*it)%6;
		    BodyRigid *b = (*m_bodies)[bodynumber];
			double v = vel(i,0);
		    switch (bodydof)
		    {
		        case 0:
					//b->setXdot(v);
					b->m_vel.x() = v;
                    break;
                case 1:
                    //b->setYdot(v);
					b->m_vel.y() = v;
                    break;
                case 2:
                    //b->setZdot(v);
					b->m_vel.z() = v;
                    break;
                case 3:
					//b->setAngularVelocityXL(v);
					b->m_wl.x() = v;
//					std::cout << "BodyRigid: " << b->getName() << "set to: " << v << std::endl;
//					std::cout << "BodyRigid: " << b->getName() << "now it its: " << b->getE0Dot() << std::endl;
                    break;
                case 4:
                    //b->setAngularVelocityYL(v);
					b->m_wl.y() = v;
//                    std::cout << "BodyRigid: " << b->getName() << "set to: " << v << std::endl;
//					std::cout << "BodyRigid: " << b->getName() << "now it its: " << b->getE1Dot() << std::endl;
                    break;
                case 5:
                    //b->setAngularVelocityZL(v);
					b->m_wl.z() = v;
//                    std::cout << "BodyRigid: " << b->getName() << "set to: " << v << std::endl;
//					std::cout << "BodyRigid: " << b->getName() << "now it its: " << b->getE2Dot() << std::endl;
                    break;

                default:
                    std::cout << "error in setting Dependent states" << std::endl;
                    break;
			}
			i++;
		}
   }

   std::vector<double> States::getIndependentStates()
   {
        std::vector<double> state;
        std::vector< int >::iterator it;
        for (it = m_IndependentIndices2.begin(); it < m_IndependentIndices2.end(); ++it)
		{
		    int bodynumber = (*it)/6;
		    int bodydof = (*it)%6;
		    BodyRigid *b = (*m_bodies)[bodynumber];

		    switch (bodydof)
		    {
		        case 0:
                    state.push_back(b->m_vel.x());
                    break;
                case 1:
                    state.push_back(b->m_vel.y());
                    break;
                case 2:
                    state.push_back(b->m_vel.z());
                    break;
                case 3:
                    state.push_back(b->m_wl.x());
                    break;
                case 4:
                    state.push_back(b->m_wl.y());
                    break;
                case 5:
                    state.push_back(b->m_wl.z());
                    break;
                default:
                    std::cout << "error in getting independent states" << std::endl;
                    break;
		    }
		}
		for (it = m_IndependentIndices.begin(); it < m_IndependentIndices.end(); ++it)
		{
		    int bodynumber = (*it)/7;
		    int bodydof = (*it)%7;
		    BodyRigid *b = (*m_bodies)[bodynumber];

		    switch (bodydof)
		    {
		        case 0:
                    state.push_back(b->m_pos.x());
                    break;
                case 1:
                    state.push_back(b->m_pos.y());
                    break;
                case 2:
                    state.push_back(b->m_pos.z());
                    break;
                case 3:
                    state.push_back(b->m_q.w());
                    break;
                case 4:
                    state.push_back(b->m_q.x());
                    break;
                case 5:
                    state.push_back(b->m_q.y());
                    break;
                case 6:
                    state.push_back(b->m_q.z());
                    break;
                default:
                    std::cout << "error in getting independent states" << std::endl;
                    break;
		    }
		}
//		std::cout << state.size() << std::endl;
		return state;
   }

    /// TODO: rather than passing in all Accels maybe set them and get them from the body
   std::vector< double > States::getIndependentDots(MatNxN allAccels)
   {
       std::vector< double > independentDots;
        std::vector< int >::iterator it;
        unsigned int accelIndice;
        for (it = m_IndependentIndices2.begin(); it < m_IndependentIndices2.end(); ++it)
        {
		    int bodynumber = (*it)/6;
		    int bodydof = (*it)%6;
		    BodyRigid *b = (*m_bodies)[bodynumber];

   		    switch (bodydof)
		    {
		        case 0:
                    accelIndice = bodynumber*6 + 0;
                    independentDots.push_back(allAccels(*it,0));
                    break;
                case 1:
                    accelIndice = bodynumber*6 + 1;
                    independentDots.push_back(allAccels(*it,0));
                    break;
                case 2:
                    accelIndice = bodynumber*6 + 2;
                    independentDots.push_back(allAccels(*it,0));
                    break;
                case 3:
                    accelIndice = bodynumber*6 + 3;
                    independentDots.push_back(allAccels(*it,0));
                    break;
                case 4:
                    accelIndice = bodynumber*6 + 4;
                    independentDots.push_back(allAccels(*it,0));
                    break;
                case 5:
                    accelIndice = bodynumber*6 + 5;
                    independentDots.push_back(allAccels(*it,0));
                    break;

                default:
                    std::cout << "error in getting independent dots" << std::endl;
                    break;
		    }
        }

        for (it = m_IndependentIndices.begin(); it < m_IndependentIndices.end(); ++it)
		{
		    int bodynumber = (*it)/7;
		    int bodydof = (*it)%7;
		    BodyRigid *b = (*m_bodies)[bodynumber];

		    switch (bodydof)
		    {
		        case 0:
                    independentDots.push_back(b->m_vel.x());
                    break;
                case 1:
                    independentDots.push_back(b->m_vel.y());
                    break;
                case 2:
                    independentDots.push_back(b->m_vel.z());
                    break;
                case 3:
                    independentDots.push_back((b->getQuatDot()).w());
                    break;
                case 4:
                    independentDots.push_back((b->getQuatDot()).x());
                    break;
                case 5:
                    independentDots.push_back((b->getQuatDot()).y());
                    break;
                case 6:
                    independentDots.push_back((b->getQuatDot()).z());
                    break;
                default:
                    std::cout << "error in getting independent dots" << std::endl;
                    break;
		    }
		}

		return independentDots;
   }
}
