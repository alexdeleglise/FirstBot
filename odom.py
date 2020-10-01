import pypot.dynamixel
import time
import sched
from math import *

ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_joint_mode([1,2])

dt = 0.1

def direct_kinematics(vg,vd) :
    vd=-vd #même sens positif que la roue gauche
    xpoint=vd + (vd-vg)/2
    thetapoint=(vd-vg)/2.6
    return xpoint,thetapoint

def odom(xpoint,thetapoint, dt) :
    dx = xpoint * dt * math.cos(thetapoint * dt)
    dy = xpoint * dt * math.sin(thetapoint * dt)
    dtheta = thetapoint * dt
    return dx,dy,dtheta

def odom_tick(xprec, yprec, thetaprec, dx, dy ,dtheta) :
    x = xprec + dx
    y = yprec + dy
    theta = thetaprec + dtheta
    return x,y,theta

def calc_odom (xprec,yprec,thetaprec) :
    vd,vg = get_present_speed([1,2])
    return odom_tick(xprec,yprec,thetaprec,odom(direct_kinematics(vg,vd),dt))


start = time.time()

Nstep = 0

x,y,theta = 0

while (1) :
    ctime = time.time() - start
    if (Nstep*0.1 - round(ctime,3) < 0.01) :
        x,y,theta = calc_odom(x,y,theta)
        print (x,y,theta)
        Nstep += 1
