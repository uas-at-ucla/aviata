#!/usr/bin/env python
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
from pyquaternion import Quaternion
import sys

WIDTH_PIXELS = 640
HEIGHT_PIXELS = 480
V_CAMERA_DISTANCE = 15.0
H_CAMERA_DISTANCE = 15.0

window = 0

pos = np.array([0.0, 0.0, 0.0])
att = Quaternion(axis=[0, 0, 1], angle=0)

def InitGL(Width, Height): 
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glDepthFunc(GL_LESS)
    glEnable(GL_DEPTH_TEST)
    glShadeModel(GL_SMOOTH)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
    gluLookAt(-H_CAMERA_DISTANCE, 0 , -V_CAMERA_DISTANCE, 0, 0, 0, V_CAMERA_DISTANCE, 0, -H_CAMERA_DISTANCE)
    glMatrixMode(GL_MODELVIEW)


def makeStructure():
    glBegin(GL_QUADS)

    glColor3f(0.0,1.0,0.0)
    glVertex3f( 1.0, 1.0,-1.0)
    glVertex3f(-1.0, 1.0,-1.0)
    glVertex3f(-1.0, 1.0, 1.0)
    glVertex3f( 1.0, 1.0, 1.0) 

    glColor3f(1.0,0.0,0.0)
    glVertex3f( 1.0,-1.0, 1.0)
    glVertex3f(-1.0,-1.0, 1.0)
    glVertex3f(-1.0,-1.0,-1.0)
    glVertex3f( 1.0,-1.0,-1.0) 

    glColor3f(0.0,1.0,1.0)
    glVertex3f( 1.0, 1.0, 1.0)
    glVertex3f(-1.0, 1.0, 1.0)
    glVertex3f(-1.0,-1.0, 1.0)
    glVertex3f( 1.0,-1.0, 1.0)

    glColor3f(1.0,1.0,0.0)
    glVertex3f( 1.0,-1.0,-1.0)
    glVertex3f(-1.0,-1.0,-1.0)
    glVertex3f(-1.0, 1.0,-1.0)
    glVertex3f( 1.0, 1.0,-1.0)

    glColor3f(0.0,0.0,1.0)
    glVertex3f(-1.0, 1.0, 1.0) 
    glVertex3f(-1.0, 1.0,-1.0)
    glVertex3f(-1.0,-1.0,-1.0) 
    glVertex3f(-1.0,-1.0, 1.0) 

    glColor3f(1.0,0.0,1.0)
    glVertex3f( 1.0, 1.0,-1.0) 
    glVertex3f( 1.0, 1.0, 1.0)
    glVertex3f( 1.0,-1.0, 1.0)
    glVertex3f( 1.0,-1.0,-1.0)

    glEnd()


def DrawGLScene():
    global pos
    global att

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glLoadIdentity()

    glRotatef(att.degrees, att.axis[0], att.axis[1], att.axis[2])
    glTranslatef(pos[0], pos[1], pos[2])

    makeStructure()

    glutSwapBuffers()


class GraphicsState:
    @staticmethod
    def updatePosition(pos_arg, att_arg):
        global pos
        global att
        pos = pos_arg
        att = att_arg


def main(initFunc=lambda g: None, loopFunc=lambda g: None, noGraphics=False):
    global window

    if noGraphics:
        initFunc(GraphicsState)
        while True:
            loopFunc(GraphicsState)
    else:
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowSize(640,480)
        glutInitWindowPosition(200,200)

        window = glutCreateWindow('OpenGL Graphics')

        def idleFunc():
            glutPostRedisplay()
            loopFunc(GraphicsState)

        glutDisplayFunc(DrawGLScene)
        glutIdleFunc(idleFunc)
        InitGL(640, 480)
        initFunc(GraphicsState)
        glutMainLoop()


if __name__ == "__main__":
    main() 
