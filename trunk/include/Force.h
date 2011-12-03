#ifndef OMD_FORCE_H
#define OMD_FORCE_H

#include <string>

using namespace std;
namespace OMD
{
/// \brief
/// Virtual Base class representing a force applied to a body or bodies
///
///	@author John Kollman
///
///
class Force
{

public:

	Force(std::string const &name): m_name(name)
	{
	}

	virtual ~Force(void)
	{
	}
	
	string getName(){return m_name;}

protected:
	virtual void apply(double t)=0;
	friend class Model;
	friend class Model1;
	//friend class Model2;
	friend class Model3;

private:
	string m_name;
};
};
#endif