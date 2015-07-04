# Introduction #

This page documents how to build OMD

Opensource Multibody Dynamics (OMD) is written in C++ and uses cmake so you can use your favorite IDE.

# Details #
  1. Use SVN to check out the code. <br /> If you use Windows, you will probably want this: <br /> http://tortoisesvn.net/downloads.html <br /> The command to check out the code base:` svn checkout https://omd.googlecode.com/svn/trunk/ omd `<br />
  1. Get Eigen.  <br />  http://eigen.tuxfamily.org <br />  OMD uses the Eigen math library.  To build, OMD has to be able to find Eigen.<br />
  1. Get CMake <br /> http://www.cmake.org/ <br />
  1. Get Bullet Collision and Math Library (Optional)<br /> http://code.google.com/p/bullet/downloads/list <br />  OMD uses the collision library from bullet to do contact.  <br />  You may be interested in Bullet instead of OMD.  Bullet is geared towards video games and the like. <br />
  1. Get swig (Optional) <br />  http://www.swig.org/ <br /> swig is used to make OMD available within python <br />
  1. Run CMake, below is an example:
![https://omd.googlecode.com/svn/cmakepict.png](https://omd.googlecode.com/svn/cmakepict.png)

CMake should be used to setup your favorite IDE,

In Linux install cmake:
$sudo apt-get install cmake-curses-gui
and or ?
sudo apt-get install cmake-qt-gui

and that is it!