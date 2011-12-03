from pyomd import *

mymodel = Model2()
initialLocation = [0,0,0]
ballmass = 1.0
transVel = [0,0,0]
rotVel = [0,0,0]
ball = make6DOF(mymodel,'ball',initialLocation,eye,ballmass,eye,transVel,rotVel)
