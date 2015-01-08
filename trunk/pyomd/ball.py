# ball dropping under gravity
# just prints out position
from pyomd import *

# create Identity Matrix 
eye = [1,0,0,0,1,0,0,0,1];
# create zero Matrix 
zero = [0,0,0,0,0,0,0,0,0];
# define g as the gravitational constant
g = 9.81
# define ball mass 
ballmass = 1
# the model which contains all OMD stuff
mymodel = Model2()
# add inertial reference frame body to model
# last argument (True) specifies that is is fixed to ground
irf = mymodel.addBodyRigid("irf",0,zero,eye,True)
dof1 = mymodel.addBodyRigid("dof1",0,zero,eye,True)
dof2 = mymodel.addBodyRigid("dof2",0,zero,eye,True)
dof3 = mymodel.addBodyRigid("dof3",0,zero,eye,True)

dof4 = mymodel.addBodyRigid("dof4",0,zero,eye,True)
dof5 = mymodel.addBodyRigid("dof5",0,zero,eye,True)
ball = mymodel.addBodyRigid("ball",10,eye,eye,True)

jnt1=mymodel.addJointRevolute("jnt1","irf",zero,"dof1",zero,[1,0,0])
jnt2=mymodel.addJointRevolute("jnt2","dof1",zero,"dof2",zero,[0,1,0])
jnt3=mymodel.addJointRevolute("jnt3","dof2",zero,"dof3",zero,[0,0,1])

jnt4=mymodel.addJointTranslational("jnt4","dof3",zero,"dof4",zero,[1,0,0])
jnt5=mymodel.addJointTranslational("jnt5","dof4",zero,"dof5",zero,[0,1,0])
jnt6=mymodel.addJointTranslational("jnt6","dof5",zero,"ball",zero,[0,0,1])

#initialLocation=Vector3(0,0,1)
# give the ball initial translational velocity
transVel=[0,0,0]
# give the ball initial rotational velocity
rotVel=[0,0,0]
#ball = make6DOF(mymodel,'ball',initialLocation,eye,ballmass,eye,transVel,rotVel)

mymodel.buildTree()
# all the bodies are assembled into the joint tree

# now add forces 
# define a force vector
frc = [0,-g*ballmass,0];
# gravity on ball
ballgrav = mymodel.addForceOnBody('ballgrav',ball,frc,zero)

# make the integrator
#integrator = IntegRK4(mymodel) 

# we have built our simple model so step forward in time
time = 0. # start time at 0 
dt = 0.0005 # time step
endtime = 2 # time at which simulation stops
while time < endtime:
   mymodel.integrate(time,time+dt)
   time = time + dt;
   x = ball.getX();
   y = ball.getY();
   z = ball.getZ();
   # get ball position
   #ballpos = ball.getPosition()
   #ballpos = mymodel.getBodyPosition('ball')
   print("time: ", time, "position: ", [x, y, z])






