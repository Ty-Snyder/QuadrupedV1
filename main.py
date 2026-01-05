from math import *
import pygame 
import time


#all distance units in mm


#Set up controller
pygame.init()
pygame.joystick.init()
clock = pygame.time.Clock()
controls={
    "LX" : 0,
    "LY" : 1,
    "RX" : 2,
    "RY" : 3,
    "Mode" : 0
}

#[0=fr,1=bl,2=fl,3=br]
legs=[0,1,2,3]

#set up legs (Hip servo,thigh servo, knee servo)
servos={
        0:{0,1,2},
        2:{3,4,5},
        3:{6,7,8},
        1:{9,10,11}
    }

#Math 3 joint quadruped leg
def InverseKinematics(pos):
    x=pos[0]
    y=pos[1]
    z=pos[2]
    a0=30.75
    a1=120
    a2=120
    c0=a0+x
    if z == 0:
        theta0=(pi/2)-(acos(a0/(sqrt(pow(z,2)+pow(c0,2)))))
    else:
        theta0=(pi/2)-(acos(a0/(sqrt(pow(z,2)+pow(c0,2))))+atan(c0/z))
    x1=a0*cos(theta0)
    z1=a0*sin(theta0)
    b1=sqrt(pow(x+a0-x1,2)+pow(y,2)+pow(z-z1,2))
    Alpha0=acos((-(pow(b1,2))+pow(a1,2)+pow(a2,2))/(2*a1*a2))
    theta2=pi-Alpha0
    theta1=(pi/2)-(asin((a2*sin(Alpha0))/b1)+atan(y/sqrt(pow(x+a0-x1,2)+pow(z-z1,2))))

    return [round(theta0*180/pi,5),round(theta1*180/pi,5),round(theta2*180/pi,5)]

#Get angle of legs at home at start to optimize
global hx,hy,hz,home
hx=0
hy=0
hz=210
home=InverseKinematics([hx,hy,hz])

def joystickpos(x,y):
    mag=sqrt(x*x+y*y)
    if y == 0:
        deg=90 if x < 0 else 270
    elif y > 0:
        deg=(atan(x/y)+pi)*(180/pi)
    elif x > 0:
        deg=(atan(x/y)+2*pi)*(180/pi)
    else:
        deg=atan(x/y)*(180/pi)
    mag = 1 if mag > 1 else mag
    deg=round(deg/22.5)*22.5
    return[mag,deg]

class SetLeg():
    def Leg(self, degrees, Leg):
        Theta0=degrees[0]
        Theta1=degrees[1]
        Theta2=degrees[2]

    def Homeall(self):
        SetLeg().Leg(home,0)
        SetLeg().Leg(home,1)
        SetLeg().Leg(home,2)
        SetLeg().Leg(home,3)

def elipse(a, b, rside, rtop):
    rside-=pi/2
    magside=(a*b)/sqrt(pow(a*sin(rside),2)+pow(b*cos(rside),2))
    z=magside*sin(rside)
    if z < 0:
        z=0
    sidey=magside*cos(rside)
    x=sidey*sin(rtop)
    y=sidey*cos(rtop)
    x+=hx
    y+=hy
    z=hz-z
    return(round(x,4),round(y,4),round(z,4))

def main():
    #presetup for main loop
    done=False
    mode=0
    curpos=[0,-pi,-3*pi/2,-pi/2]
    
    #steps per second
    sps=1

    #ticks per second
    tps=80

    #distance per step (mm)
    mps=120

    #elipse a and b
    ellipsa=mps/2
    ellipsb=ellipsa*4/6

    controller = None

    while not done:

        #check for controllers
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                if event.type == pygame.QUIT:
                    done = True

                #find 
                joy = pygame.joystick.Joystick(event.device_index)
                print(joy)
                controller = joy
                print(f"Joystick {joy.get_instance_id()} connencted")   

            if event.type == pygame.JOYDEVICEREMOVED:
                SetLeg().Homeall()
                time.sleep(3)
                exit("Controller disconnected")
        if not controller == None:
            #Movement mode
            if mode == 0:

                #get joystick possitions as vector input(axis x,axisy) output(magnitude,radians)
                LeftJ = joystickpos(controller.get_axis(controls["LX"]),controller.get_axis(controls["LY"]))
                RightJ = round(controller.get_axis(controls["RX"]))
                
                #degrees per tick
                dpt = (sps*2*pi)/(tps)

                #run math for each leg
                for i in legs:

                    #elipse theta
                    if round(LeftJ[0]) > 0 or not RightJ == 0 :
                        curpos[i]=curpos[i] + dpt
                    else:
                        SetLeg().Homeall()
                        curpos[i] = -i*pi/2
                    
                    #top view rotation angle of elipse
                    if i == 0 or i == 2:
                        rtop=(LeftJ[1]-round(RightJ*45))*pi/180
                    elif i == 1 or i == 3:
                        rtop=(LeftJ[1]+round(RightJ*45))*pi/180

                    #get leg position
                    if round(LeftJ[0]) > 0 and curpos[i] >= 0:
                        pos=elipse(ellipsa,ellipsb,curpos[i],rtop)
                        degrees=InverseKinematics(pos)
                        SetLeg().Leg(degrees,i)
                        print(i,pos,degrees,rtop) 
                    elif round(LeftJ[0]) == 0 and not RightJ == 0 and curpos[i] >= 0:
                        rtop=pi/2 if (RightJ < 0 and (i == 1 or i == 3)) or (RightJ > 0 and (i == 0 or i == 2)) else 3*pi/2
                        pos=elipse(ellipsa,ellipsb,curpos[i],rtop)
                        degrees=InverseKinematics(pos)
                        SetLeg().Leg(degrees,i)
                        print(i,pos,degrees,rtop)
                
                #Switch modes
                if controller.get_button(controls["Mode"]):
                    while controller.get_button(controls["Mode"]):
                        pygame.event.get()
                    mode=1
                    SetLeg().Homeall()
                    print("Stationary mode")

            #Stationary mode
            if mode == 1:

                #Switch modes
                if controller.get_button(controls["Mode"]):
                    while controller.get_button(controls["Mode"]):
                        pygame.event.get()
                    mode=0
                    SetLeg().Homeall()
                    curpos[i] = -i*pi/2
                    print("Movement mode")

        #limit how many times per second main loop runs
        clock.tick(tps)
main()
