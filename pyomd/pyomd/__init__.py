from OMDlib import *

# try to install visual if not warn that somethings wont work and make variable
# that records whether visual was imported or not
try:
	from visual import * # going to use visual for animation
	visualInstalled = True
except ImportError:
	print "Sorry, you don't have the visual (vpython.org) module installed,"
	print "several methods in this script relies on it.  Please install or avoid"
	print "using the functions that rely on it."
	visualInstalled = False

# create Identity Matrix 
eye = DoubleVector(9)
eye =[1,0,0,0,1,0,0,0,1];
# create zero Matrix 
zero = DoubleVector(9)
zero = [0,0,0,0,0,0,0,0,0];

# makes a body with all six DOF in model2
def make6DOF(model, bodyname,initialLocation,initialRot,bodymass,inertia,initTransVel=[0,0,0],initRevVel=[0,0,0]):
   irfname = 'DefaultIRF'
   #ibn : intermediatebodynames
   ibn = []
   #ijn : intermeidatejointnames
   ijn = []
   for i in range(0,5):
      # make all the intermediate bodies name based on body
      ibn.append( bodyname + 'ib' + str(i) )
      ijn.append( bodyname + 'ij' + str(i) )

   # need 1 more joint
   ijn.append( bodyname + 'ij' + '5' )

   for name in ibn:
      model.addBodyRigid(name,0,zero,eye)

   # make final body and give it the name user specified
   body = model.addBodyRigid(bodyname,bodymass,initialRot,inertia)

   # rather than using loop do it the long way, more readable
   p2j = [0,0,0]
   j2c = [0,0,0]
   jaxis = [1,0,0]
   model.addJointTranslational(ijn[0],irfname,p2j,ibn[0],j2c,jaxis,0,initTransVel[0])
   jaxis = [0,1,0]
   model.addJointTranslational(ijn[1],ibn[0],p2j,ibn[1],j2c,jaxis,0,initTransVel[1])
   jaxis = [0,0,1]
   model.addJointTranslational(ijn[2],ibn[1],p2j,ibn[2],j2c,jaxis,0,initTransVel[2])
   # make revolute joints
   jaxis = [1,0,0]
   model.addJointRevolute(ijn[3],ibn[2],p2j,ibn[3],j2c,jaxis,0,initRevVel[0])
   jaxis = [0,1,0]
   model.addJointRevolute(ijn[4],ibn[3],p2j,ibn[4],j2c,jaxis,0,initRevVel[1])
   jaxis = [0,0,1]
   j2c = [0,0,0]
   model.addJointRevolute(ijn[5],ibn[4],initialLocation,bodyname,j2c,jaxis,0,initRevVel[2])
   return body

def pos(body,offset=[0,0,0]):
   p = [body.getX(), body.getY(), body.getZ()]
   return [p[0]+offset[0],p[1]+offset[1],p[2]+offset[2]]

def setOrientation(body_, frame_):
   xaxis = body_.getXAxis()
   yaxis = body_.getYAxis()
   frame_.axis = xaxis
   frame_.up = yaxis

def setPositionOrientation(body_, frame_):
   frame_.pos = pos(body_)
   setOrientation(body_,frame_)
