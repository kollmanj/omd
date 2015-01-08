# ball dropping under gravity
# and comming into contact with ground
# uses vpython to animate
from pyomd import *
from visual import * # going to use visual for animation

g = 9.81
# define ball mass 
ballmass = 1
ballradius = 0.5
floorheight = 0.1
floorwidth = 10.0

# create Identity Matrix 
eye = [1,0,0,0,1,0,0,0,1];
# create zero Matrix 
zero = [0,0,0,0,0,0,0,0,0];

# the model which contains all OMD stuff
mymodel = Model2()
irf = mymodel.addBodyRigid('irf',0,zero,eye,True)
# add inertial reference frame body to model
# last argument (True) specifies that is is fixed to ground

initialLocation=[0,1,0]
# give the ball initial translational velocity
transVel=[0,0,0]
# give the ball initial rotational velocity
rotVel=[0,0,0]
dof1 = mymodel.addBodyRigid("dof1",0,zero,eye,True)
dof2 = mymodel.addBodyRigid("dof2",0,zero,eye,True)
dof3 = mymodel.addBodyRigid("dof3",0,zero,eye,True)

dof4 = mymodel.addBodyRigid("dof4",0,zero,eye,True)
dof5 = mymodel.addBodyRigid("dof5",0,zero,eye,True)
ball = mymodel.addBodyRigid("ball",ballmass,eye,eye,True)
ball = make6DOF(mymodel,'ball',initialLocation,eye,ballmass,eye,transVel,rotVel)
initialLocation=[0.1,2.1,0.0]
transVel=[0,0,0]
rotVel=[0,-1,-.1]

dof1 = mymodel.addBodyRigid("dof1",0,zero,eye,True)
dof2 = mymodel.addBodyRigid("dof2",0,zero,eye,True)
dof3 = mymodel.addBodyRigid("dof3",0,zero,eye,True)

dof4 = mymodel.addBodyRigid("dof4",0,zero,eye,True)
dof5 = mymodel.addBodyRigid("dof5",0,zero,eye,True)
#ball2 = mymodel.addBodyRigid("ball2",ballmass,eye,initialLocation,[0,0,0,1],[0,0,0],[0,0,0])
ball2 = mymodel.addBodyRigid("ball2",ballmass,eye,eye)
#BodyRigid* addBodyRigid(std::string const &name, double const &mass, std::vector<double> const &inertia, std::vector<double> const &pos, std::vector<double> q, std::vector<double> const &vel, std::vector<double> const &wl, bool const &fixed=false);
#ball2 = make6DOF(mymodel,'ball2',initialLocation,eye,ballmass,eye,transVel,rotVel)

# this body will be attached it inertial reference Frame 
bulb = mymodel.addBodyRigid('bulb',ballmass/5,eye,eye)
# 2 intermediate bodies
bulbi1 = mymodel.addBodyRigid('bubli1',0,zero,eye)
bulbi2 = mymodel.addBodyRigid('bubli2',0,zero,eye)
jntAxis=[1,0,0]
p2j = [0.6,3,0.6] # parent to joint
j2c = [0,0,0] # joint to child
mymodel.addJointRevolute('pendJnt',irf,p2j,bulbi1,j2c,jntAxis,0,0)
p2j = [0,0,0] # parent to joint
jntAxis=[0,1,0]
mymodel.addJointRevolute('pendJnt',bulbi1,p2j,bulbi2,j2c,jntAxis,0,0)
jntAxis=[0,0,1]
j2c = [0,-1,0] # joint to child
mymodel.addJointRevolute('pendJnt',bulbi2,p2j,bulb,j2c,jntAxis,0,0)

mymodel.buildTree()

# all the bodies are assembled into the joint tree

# now add forces 
# define a force vector
frc = [0,-g*ballmass,0]
# gravity on ball
#ballgrav = mymodel.addForce1Body('ballgrav',ball,frc)
#ball2grav = mymodel.addForce1Body('ball2grav',ball2,frc)
grav = mymodel.addForceGravity('grav',g,[0,-1,0])
#bulbgrav = mymodel.addForce1Body('bulbgrav',bulb,frc)
# add contact force between body: irf which will have a cube
# and body:  ball which will have a sphere associated with it
stiff = 10000   # contact stiffness
damp = 10      # contact damping
frict = 0.3    # contact friction
thresh = 0.0001  # friction threshhold
contactf = mymodel.addForceContact('contactf',stiff,damp,frict,thresh)

contactf.setCollisionMargin(0.0001)
# add a box to irf for contact
contactf.addBox(floorwidth/2,floorheight/2,floorwidth/2,[0,0,0],[1,0,0,0,1,0,0,0,1])
# add a sphere to ball for contact
contactf.addCapsule(0.5,0.0,[0,0,0],[1,0,0,0,1,0,0,0,1],ball)
# add a sphere to ball2 for contact
contactf.addCapsule(0.5,0.0,[0,0,0],[1,0,0,0,1,0,0,0,1],ball2)
# add a sphere to bulb for contact
contactf.addCapsule(0.2,0.0,[0,0,0],[1,0,0,0,1,0,0,0,1],bulb)
# make the integrator

# before starting integration make vpython stuff for animation
scene2 = display(title='scene2',
     width=300, height=300,
     center=(0,1,0))
scene2.x = 0
scene2.y = 0
scene2.select()
vfloor = box(length=floorwidth, height=floorheight, width=floorwidth,material=materials.wood)
#vfloor = box(length=floorwidth, height=floorheight, width=floorwidth)
f1=frame()
vball = sphere(frame=f1,pos=(0,0,0), radius = ballradius,color=color.red,material=materials.marble)
#vball = sphere(frame=f1,pos=(0,0,0), radius = ballradius,color=color.red)
f2=frame()
vball2 = sphere(frame=f2,pos=(0,0,0), radius = ballradius,color=color.green,material=materials.marble)
#vball2 = sphere(frame=f2,pos=(0,0,0), radius = ballradius,color=color.green)
f3=frame()
vbulb = sphere(frame=f3,pos=(0,0,0), radius=0.2,color=color.yellow,material=materials.emissive)
#vbulb = sphere(frame=f3,pos=(0,0,0), radius=0.2,color=color.yellow)
# make a little cylinder for the bulb threads
vbulb2 = cylinder(frame=f3,pos=(0,0.1,0),axis=(0,0.2,0),radius=0.1,color=color.yellow)
# make a little cylinder for the chord
vbulb3 = cylinder(frame=f3,pos=(0,0.3,0),axis=(0,0.7,0),radius=0.03,color=color.white)
# we have built our simple model so step forward in time
time = 0. # start time at 0 
dt = 0.0005 # time step
endtime = 4.0 # time at which simulation stops
#endtime = .001 # time at which simulation stops
count =0
count2 =0
while time < endtime:
   rate(1000)
   mymodel.integrate(time,time+dt)
   time = time + dt;
   # get ball position
   setPositionOrientation(ball,f1)
  # vball.axis=[ballaa[0],ballaa[1],ballaa[2]]
  # vball.angle=ballaa[3]
   setPositionOrientation(ball2,f2)
   setPositionOrientation(bulb,f3)

   print time
   #print [ball2Rot.getItem(1,0), ball2Rot.getItem(1,1),ball2Rot.getItem(1,2)]
   #print [ball2Rot.getItem(0,0), ball2Rot.getItem(0,1),ball2Rot.getItem(0,2)]
   count = count+1
   #if (count == 30):
   #   count=0;
   #   im = ImageGrab.grab((0,0,300,300)) # if window located at (0,0), width=height=600
   #   filename = 'm'+str(count2)+'.jpg'
   #   count2=count2+1
   #   im.save(filename) # where filename is incremented for each image



