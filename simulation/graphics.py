#!/usr/bin/env python3

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
from pyquaternion import Quaternion
import sys

WIDTH_PIXELS = 1280
HEIGHT_PIXELS = 720
V_CAMERA_DISTANCE = 8.0
H_CAMERA_DISTANCE = 8.0

class GraphicsState:
    pos_target = np.array([0.0, 0.0, 0.0])
    yaw_target = 0
    pos = np.array([0.0, 0.0, 0.0])
    att = Quaternion()
    rotors = []
    motor_imputs = []


def InitGL(Width, Height): 
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glDepthFunc(GL_LESS)
    glEnable(GL_DEPTH_TEST)

    # smooth lines
    glEnable(GL_LINE_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST) # GL_NICEST
    glEnable(GL_POINT_SMOOTH)
    glHint(GL_POINT_SMOOTH_HINT, GL_FASTEST) # GL_NICEST
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    glShadeModel(GL_SMOOTH)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
    gluLookAt(-H_CAMERA_DISTANCE, 0 , -V_CAMERA_DISTANCE, 0, 0, 0, V_CAMERA_DISTANCE, 0, -H_CAMERA_DISTANCE)
    glMatrixMode(GL_MODELVIEW)


def makeCube(pos, r):
    x = pos[0]
    y = pos[1]
    z = pos[2]

    glBegin(GL_QUADS)

    glVertex3f(x+r, y+r, z-r)
    glVertex3f(x-r, y+r, z-r)
    glVertex3f(x-r, y+r, z+r)
    glVertex3f(x+r, y+r, z+r) 

    glVertex3f(x+r, y-r, z+r)
    glVertex3f(x-r, y-r, z+r)
    glVertex3f(x-r, y-r, z-r)
    glVertex3f(x+r, y-r, z-r) 

    glVertex3f(x+r, y+r, z+r)
    glVertex3f(x-r, y+r, z+r)
    glVertex3f(x-r, y-r, z+r)
    glVertex3f(x+r, y-r, z+r)

    glVertex3f(x+r, y-r, z-r)
    glVertex3f(x-r, y-r, z-r)
    glVertex3f(x-r, y+r, z-r)
    glVertex3f(x+r, y+r, z-r)

    glVertex3f(x-r, y+r, z+r) 
    glVertex3f(x-r, y+r, z-r)
    glVertex3f(x-r, y-r, z-r) 
    glVertex3f(x-r, y-r, z+r) 

    glVertex3f(x+r, y+r, z-r) 
    glVertex3f(x+r, y+r, z+r)
    glVertex3f(x+r, y-r, z+r)
    glVertex3f(x+r, y-r, z-r)

    glEnd()


def makeRotor(pos, axis, power):
    r = 0.05
    glLineWidth(1.0)

    if power <= 0.0:
        glColor3f(0.0,1.0,1.0)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
    elif power < 0.5:
        glColor3f(0.5, 1-power, 1-power)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
    elif power < 1.0:
        glColor3f(power, 0.5, 0.5)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
    else:
        glColor3f(1.0,0.0,0.0)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

    makeCube(pos, r)

    # show axis of rotation
    glBegin(GL_LINES)
    glVertex3f(pos[0]-axis[0]*r*4, pos[1]-axis[1]*r*4, pos[2]-axis[2]*r*4)
    glVertex3f(pos[0]+axis[0]*r*4, pos[1]+axis[1]*r*4, pos[2]+axis[2]*r*4)
    glEnd()


def makeStructure():
    # show center of mass
    glLineWidth(1.0)
    glColor3f(1, 1, 0)
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
    makeCube([0, 0, 0], 0.05)

    for i in range(len(GraphicsState.rotors)):
        rotor = GraphicsState.rotors[i]
        if rotor['Ct']:
            motor_input = GraphicsState.motor_inputs[i]
            makeRotor(rotor['position'], rotor['axis'], motor_input)


def DrawGLScene():
    pos = GraphicsState.pos
    att = GraphicsState.att

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glLoadIdentity()

    # show position and yaw target
    glLineWidth(1.0)
    glColor3f(1, 0.647, 0)
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
    makeCube(GraphicsState.pos_target, 0.05)
    glBegin(GL_LINES)
    glVertex3f(GraphicsState.pos_target[0], GraphicsState.pos_target[1], GraphicsState.pos_target[2])
    heading = np.exp(1j * GraphicsState.yaw_target)
    glVertex3f(GraphicsState.pos_target[0]+(np.real(heading)*0.25), GraphicsState.pos_target[1]+(np.imag(heading)*0.25), GraphicsState.pos_target[2])
    glEnd()

    glTranslatef(pos[0], pos[1], pos[2])
    glRotatef(att.degrees, att.axis[0], att.axis[1], att.axis[2])

    makeStructure()

    glutSwapBuffers()


def main(loopFunc=lambda g: None, loop_period=1000, keyPressedFunc=lambda k,x,y: None):
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
    glutInitWindowSize(WIDTH_PIXELS, HEIGHT_PIXELS)
    # glutInitWindowPosition(0, 0)

    window = glutCreateWindow('OpenGL Graphics')

    glutDisplayFunc(DrawGLScene)
    glutIdleFunc(None)
    glutKeyboardFunc(keyPressedFunc)
    glutSpecialFunc(keyPressedFunc)
    InitGL(WIDTH_PIXELS, HEIGHT_PIXELS)

    target_time = 0

    def loopWrapperFunc(value):
        loopFunc(GraphicsState)
        glutPostRedisplay()
        nonlocal target_time
        target_time += loop_period
        delay = target_time - glutGet(GLUT_ELAPSED_TIME)
        if delay < 0:
            if delay < -loop_period*5: # print warning if we're 5 loop_periods behind
                print("Warning: trouble keeping up with loop_period")
            delay = 0
        glutTimerFunc(delay, loopWrapperFunc, 0)

    def initLoop(value):
        nonlocal target_time
        target_time = glutGet(GLUT_ELAPSED_TIME)
        loopWrapperFunc(0)

    glutTimerFunc(0, initLoop, 0)

    glutMainLoop()


if __name__ == "__main__":
    main() 
