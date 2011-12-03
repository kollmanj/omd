# ball dropping under gravity
# and comming into contact with ground
# uses vpython to animate
from pyomd import *

# define g as the gravitational constant
g = 9.81
# define ball mass 
ballmass = 1
ballradius = 1.0
floorheight = 0.1
floorwidth = 2
# the model which contains all OMD stuff
mymodel = Model1()

stiff = 20000   # contact stiffness
damp = 10      # contact damping
frict = 0.6   # contact friction
thresh = 0.001  # friction threshhold
contactf = mymodel.addForceContact('contactf',stiff,damp,frict,thresh)

#box1 = mymodel.addBodyRigid('box1',1,[1,0,0,0,1,0,0,0,1],[0,0,0],[1,0,0,0],[0,0,0],[0,0,0],True)
contactf.addBox(floorwidth/2,floorheight/2,floorwidth/2,[0,0,0],[1,0,0,0,1,0,0,0,1])
#contactf.addCapsule(.5,0.0,[0,0,0],[1,0,0,0,1,0,0,0,1]) 

ball = mymodel.addBodyRigid('ball',ballmass,[1,0,0,0,1,0,0,0,1],[0,1.65,0],[1,0,0,0],[0,0,0],[0,0,10],False) 
contactf.addCapsule(ballradius,0.0,[0,0,0],[1,0,0,0,1,0,0,0,1],ball) 
#ballgrav = mymodel.addForceOnBody('ballgrav',ball,[0,-g*ballmass,0],[0,0,0],False)
grav = mymodel.addForceGravity('grav',g,[0,-1,0])


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

time = 0. # start time at 0 
dt = 0.001 # time step
endtime = 1 # time at which simulation stops
#endtime = .001 # time at which simulation stops
while time < endtime:
   #mymodel.calcIndependentStates()
   rate(100)
   #x = ball.getX()
   #y = ball.getY()
   #z = ball.getZ()
   setPositionOrientation(ball,f1)
   #f1.pos = [x,y,z]
   mymodel.integrate(time,time+dt)
   time = time + dt;
   print time
   #print 'Time: ', time, 'Pos: ', x, y, z
 
