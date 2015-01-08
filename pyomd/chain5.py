# chain link dropping under gravity
# and comming into contact with ground
# uses vpython to animate
from pyomd import *
#from visual import * # going to use visual for animation
#import Image # from PIL
#import ImageGrab # from PIL
# Units: kg and mm
# here are some parameters
linkR =  8.5	# chain link dimension 
linkLength =  linkR*4	# chain link dimension m
linkWidth =   linkR*2       # chain link dimension m
linkHeight = linkR*4 	# chain link dimension m
linkMass = 0.16         # mass in kg
floorheight =  linkR*.2
floorwidth =  linkR*130
# create Identity Matrix 
#eye = Matrix3(1,0,0,0,1,0,0,0,1)
linkI =[28,0,0,0,59,0,0,0,49];
# define g as the gravitational constant
g = 9810 # mm / s^2

# this is the VPython scene which describes view
scene2 = display(title='scene2',
     width=300, height=300, x =0.1, y=0,
     center=(0,20,0), autoscale = False,
     range = 200, forward = (-1,-.8,-1))

# floor box
vfloor = box(pos=(0,-floorheight,0),length=floorwidth, height=floorheight, width=floorwidth,material=materials.wood)


# the model which contains all OMD stuff
mymodel = Model2()

frames = []

# VPython geoetry
f0 = frame() # this is a reference frame in VPython that will parent geometry
frames.append(f0)
c1 = cylinder(frame=f0, pos=(0,linkR*2,-linkR),axis=(0,0,linkR*2),radius=linkR) # pin
c2 = cylinder(frame=f0, pos=(0,-linkR*2,-linkR),axis=(0,0,linkR*2),radius=linkR) # pin
sc = sphere(frame=f0, pos=(0,0,0),radius=linkR*0.1) # just to show the center
b1 = box(frame=f0, pos=(0,0,-linkWidth/2-linkWidth*0.05), length=linkR*1.8, height=linkLength, width=linkWidth*0.1) #side
b2 = box(frame=f0, pos=(0,0,linkWidth/2+linkWidth*0.05), length=linkR*1.8, height=linkLength, width=linkWidth*0.1) #side

# initial location of the link
initialLocation=[0,linkR*8,0.0]
# give toe link initial translational velocity
transVel=[0.0,0,0]
# give the link initial rotational velocity
rotVel=[-0.1,0,1]
# make 6DOF is a shortcut in Model1 to get a body with 6 DOF, 
# otherwise it would have to be built up 1 dof at a time
link = make6DOF(mymodel,'link0',initialLocation,eye,linkMass,linkI,transVel,rotVel)
# now add forces 
# define a force vector
frc = [0,-g*linkMass,0];
# gravity on link
#linkgrav = mymodel.addForce1Body('linkgrav',link,frc)
# add contact force between body: irf which will have a cube
# and body:  link which will have a sphere associated with it
stiff = 90000   # contact stiffness
damp = 25      # contact damping
frict = 0.6    # contact friction
thresh = 0.0001  # friction threshhold
# this tells the model there will be contact and defines parameters for it
contactf = mymodel.addForceContact('contactf',stiff,damp,frict,thresh)
#contactf.setCollisionWorldScale(1);
contactf.setCollisionMargin(0.000001)
# add a box to inertial reference frame (irf), no body defined means irf
contactf.addBox(floorwidth/2,floorheight/2,floorwidth/2,[0,0,0],[1,0,0,0,1,0,0,0,1])
# the contact geometry is defined relative to cg so we need vector
# defining displacement
offset = [0,linkR*2,0.0]
cylRot = [1,0,0,0,0,1,0,-1,0]; # rot matrix for cylinder
contactf.addCylinder(linkR,linkR*2,offset,cylRot,link)
offset = [0,-linkR*2,0.0]
contactf.addCylinder(linkR,linkR*2,offset,cylRot,link)
#offset = Vector3(0,0,linkWidth/2-linkWidth*0.05)
#contactf.addBox(linkR,linkLength/2,linkWidth*0.05,offset,eye,link)
#contactf.addCapsule(linkR,0,offset,eye,link)

# Now we have 1 link, make loop for more links
numOLinks = 30
for i in range (1,numOLinks):
   bodyname = 'link' + str(i)
   print bodyname
   mymodel.addBodyRigid(bodyname,linkMass,eye,eye)
   p2j = [0,linkR*2,0]
   j2c = [0,linkR*2,0]
   jAxis = [0,0,1]

   parentBodyName = 'link' + str(i-1)
   print parentBodyName
   jointName = 'jnt' + str(i)
   print jointName
   mymodel.addJointRevolute(jointName,parentBodyName,p2j,bodyname,j2c,jAxis,0,0)
   forceName = 'linkgForce' + str(i)
   #mymodel.addForce1Body(forceName,mymodel.getBody(bodyname),frc)

   f2=frame()
   frames.append(f2)
   cylinder(frame=frames[i], pos=(0,linkR*2,-linkR),axis=(0,0,linkR*2),radius=linkR) # pin
   sphere(frame=frames[i], pos=(0,0,0),radius=linkR*0.1) # just to show the center
   box(frame=frames[i], pos=(0,0,-linkWidth/2-linkWidth*0.05), length=linkR*1.8, height=linkLength, width=linkWidth*0.1) #side
   box(frame=frames[i], pos=(0,0,linkWidth/2+linkWidth*0.05), length=linkR*1.8, height=linkLength, width=linkWidth*0.1) #side
   offset = [0,linkR*2,0.0]
   contactf.addCylinder(linkR,linkR*2,offset,cylRot,mymodel.getBody(bodyname))
   forceName = 'fgrav' + str(i)
   #mymodel.addForce1Body(forceName,mymodel.getBody(bodyname),frc)
	

linkgrav = mymodel.addForceGravity('linkgrav',g,[0,-1,0])
# build the tree of bodies, needs to be done for all models of type model1
mymodel.buildTree() # no arguments
# make the integrator
#integrator = IntegRK4(mymodel) 

time = 0. # start time at 0 
dt = 0.00005 # time step
endtime = 0.5 # time at which simulation stops
#count =0
#count2 =0
#raw_input()
while time < endtime:
   rate(1000)  # controls speed of animation
   mymodel.integrate(time,time+dt)
   time = time + dt;
 
   #loop through and place all links
   for i in range (0,numOLinks):
      f = frames[i]
      bodyname = 'link' + str(i)
      setPositionOrientation(mymodel.getBody(bodyname),f)
      #f.pos = pos(mymodel.getBody(bodyname))
      #f.up=up(mymodel.getBody(bodyname))
      #f.axis=axis(mymodel.getBody(bodyname))

   #f = contactf.getContactForces(link)
   #if (len(f) > 0):
   #        time = 100
  
   #print 'time: ', time
   #print 'f: ', f 
#   count = count+1
#   if (count == 30):
#      count=0;
#      im = ImageGrab.grab((0,0,300,300)) # if window located at (0,0), width=height=600
#      filename = 'm'+str(count2)+'.jpg'
#      count2=count2+1
#      im.save(filename) # where filename is incremented for each image
print 'done'


 
