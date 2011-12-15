# ball dropping under gravity
# and comming into contact with ground
# uses vpython to animate
from pyomd import *

# before starting integration make vpython stuff for animation
scene2 = display(title='scene2',
     width=300, height=300,
     center=(0,1,0))
scene2.x = 0
scene2.y = 0
scene2.select()
floorheight = 0.1
floorwidth = 20 
vfloor = box(length=floorwidth, height=floorheight, width=floorwidth,material=materials.wood)

# create Identity Matrix 
eye = [1,0,0,0,1,0,0,0,1];
# create zero Matrix 
zero = [0,0,0,0,0,0,0,0,0];
# define g as the gravitational constant
g = 9.81
# define ball mass 
ballMass = 1
ballRadius = 0.5
jntSpacing = 1.1*ballRadius

# the model which contains all OMD stuff
mymodel = Model2()
irf = mymodel.addBodyRigid('irf',0,zero,eye,True)
# add inertial reference frame body to model
# last argument (True) specifies that is is fixed to ground

initialLocation=[0,1,0]
# give the ball initial translational velocity
transVel=[1.1,0,0]
# give the ball initial rotational velocity
rotVel=[0,0,0]
ball0 = make6DOF(mymodel,'ball0',initialLocation,eye,ballMass,eye,transVel,rotVel)


# all the bodies are assembled into the joint tree

# now add forces 
# define a force vector
frc = [0,-g*ballMass,0]
# gravity on ball
# add contact force between body: irf which will have a cube
# and body:  ball which will have a sphere associated with it
stiff = 20000   # contact stiffness
damp = 100      # contact damping
frict = 0.5    # contact friction
thresh = 0.001  # friction threshhold
contactf = mymodel.addForceContact('contactf',stiff,damp,frict,thresh)
# add a box to irf for contact
contactf.addBox(floorwidth/2,floorheight/2,floorwidth/2,[0,0,0],[1,0,0,0,1,0,0,0,1])
# add a sphere to ball for contact)
# add a sphere to ball for contact
contactf.addCapsule(ballRadius,0.0,[0,0,0],eye,ball0)

bodyCount = 30

frames = []
f1=frame()
frames.append(f1)
vball = sphere(frame=frames[0],pos=(0,0,0), radius = ballRadius,color=color.red,material=materials.marble)

for i in range (1,bodyCount):
   bodyname = 'ball' + str(i)
   print bodyname
   mymodel.addBodyRigid(bodyname,ballMass,eye,eye)
   p2j = [0,jntSpacing,0]
   j2c = [0,jntSpacing,0]
   jAxis = [0,0,1]

   parentBodyName = 'ball' + str(i-1)
   print parentBodyName
   mymodel.addJointRevolute('test',parentBodyName,p2j,bodyname,j2c,jAxis,0,0)
   forceName = 'ball' + str(i)

   f2=frame()
   frames.append(f2)
   vball = sphere(frame=frames[i],pos=(0,0,0), radius = ballRadius,color=color.green,material=materials.marble)
   contactf.addCapsule(ballRadius,0.0,[0,0,0],eye,mymodel.getBody(bodyname))
   forceName = 'fgrav' + str(i)
   #mymodel.addForceOnBody(forceName,mymodel.getBody(bodyname),frc,[0,0,0],False)

# make the integrator
mymodel.buildTree()
grav = mymodel.addForceGravity('grav',g,[0,-1,0])

#vfloor = box(length=floorwidth, height=floorheight, width=floorwidth)
#vball = sphere(frame=f1,pos=(0,0,0), radius = ballradius,color=color.red)
#vball2 = sphere(frame=f2,pos=(0,0,0), radius = ballradius,color=color.green)
# we have built our simple model so step forward in time
time = 0. # start time at 0 
dt = 0.01 # time step
endtime = 30 # time at which simulation stops
#endtime = .001 # time at which simulation stops
#arrow1 = arrow(pos=(0,0,0),axis=(0,0,0),shaftwidth=0.2)
#arrow2 = arrow(pos=(0,0,0),axis=(0,0,0),shaftwidth=0.2)
#pntrs = []
#pntrs.append(arrow1)
#pntrs.append(arrow2)
while time < endtime:
   rate(2000)
   mymodel.integrate(time,time+dt)
   time = time + dt;
   # get ball position
   for i in range (0,bodyCount):
      f = frames[i]
      bodyname = 'ball' + str(i)
      setPositionOrientation(mymodel.getBody(bodyname),f)
   
   print time
print 'done'

