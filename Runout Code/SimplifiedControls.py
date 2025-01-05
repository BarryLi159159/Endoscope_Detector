import serial
import time
import math

arduino = serial.Serial('COM11', 9600, timeout=0.5) 

time.sleep(2)
UpperRoller = 1
LowerRoller = 2
StraigheningAssembly = 3
Endoscope = 4

UpperRollerLocation = 0 
LowerRollerLocation = 0
StraigheningAssemblyLocaton = 0
EndoscopeAngle = 0
FirstSend = 1

#Directions
Clockwise = 0
CounterClockwise = 1
Up = 0 
Down = 1
Forward = 0
Backward = 1

MovementConfirm = arduino.readline()

LimitMotor = 0 #Global variable defining the currently running motor
LimitDirection = 0 #GLobal variable defing the current running motors direction

def MotorMove(Motor,Dir,Speed,Steps,LimitCheck = 0): #Raw movement command 
    Spd = str(Speed).rjust(4, '0')
    dataStr = str(Motor) + str(Dir) + str(LimitCheck) + str(Spd) + str(Steps) + "\n"
    arduino.write(dataStr.encode())

def Actuation(Motor,Distance,Direction,FirstSend = 0,LimitCheck = 1,VideoCapture = 0):
    time.sleep(0.5)
    Datain = 1
    global UpperRollerLocation, LowerRollerLocation, EndoscopeAngle, StraigheningAssemblyLocaton
    global LimitMotor, LimitDirection, MovementConfirm

    if FirstSend == 1:
        LimitMotor = Motor
        LimitDirection = Direction
    
    while Datain < 2:
        
        print(arduino.in_waiting)
        #LimitConfirm = arduino.readline()
        if FirstSend == 1 or MovementConfirm.decode("utf-8") == "Done":
            Datain = 3
            MovementConfirm = 0
            LimitMotor = Motor
            LimitDirection = Direction

            Nema17Speed = 1200
            Nema17Speed4Digit = str(Nema17Speed).rjust(4, '0') #allows for any 4 digit number to be used 
            Nema23Speed = 550
            Nema23Speed4Digit = str(Nema23Speed).rjust(4, '0')
            
            StepsPerMMNema17 = 200
            StepsPerMMNema23 = 80 #5 mm lead. 400 steps per rev so 400/5 = 80 steps per mm
            StepsPerDegree = 13.5 #

            Angle = Distance
            
            match Motor:
                case 1:
                    if Direction == 0:
                        Dir = 0
                        UpperRollerLocation = UpperRollerLocation + Distance
                    elif Direction == 1:
                        Dir = 1
                        UpperRollerLocation = UpperRollerLocation - Distance
                    Speed =  Nema17Speed4Digit
                    Steps = StepsPerMMNema17 * Distance
                
                case 2:
                    if Direction == 1:
                        Dir = 1
                        LowerRollerLocation = LowerRollerLocation - Distance
                    elif Direction == 0:
                        Dir = 0
                        LowerRollerLocation = LowerRollerLocation + Distance
                    Speed =  Nema17Speed4Digit
                    Steps = StepsPerMMNema17 * Distance
                
                case 3:
                    if Direction == 0:
                        Dir = 0
                        StraigheningAssemblyLocaton = StraigheningAssemblyLocaton + Distance
                    elif Direction == 1:
                        Dir = 1
                        StraigheningAssemblyLocaton = StraigheningAssemblyLocaton - Distance
                    Speed = Nema23Speed4Digit     
                    Steps = Distance * StepsPerMMNema23 

                case 4:
                    if Direction == 0:
                        Dir = 0
                        EndoscopeAngle = EndoscopeAngle + Angle
                    elif Direction == 1:
                        Dir = 1
                        EndoscopeAngle = EndoscopeAngle - Angle
                    Speed = Nema17Speed4Digit
                    Steps = StepsPerDegree * Angle

            dataStr = str(Motor) + str(Dir) + str(LimitCheck) + str(Speed) + str(Steps) + "\n"
            arduino.write(dataStr.encode())
            #FirstSend = 2 first send defaults to 0 sooooooo
            if VideoCapture == 1:
                return
            while MovementConfirm != ("Limit" or "Done"):
                MovementConfirm = arduino.readline()
                print(MovementConfirm)
                if MovementConfirm.decode("utf-8") == "Limit":
                    LimitSwitch()
                    return
                elif MovementConfirm.decode("utf-8") == "Done":
                    return
    return

def ResetPostion():
    global UpperRollerLocation 
    global LowerRollerLocation
    global StraigheningAssemblyLocaton
    if UpperRollerLocation > 0:
        Actuation(UpperRoller,abs(UpperRollerLocation),"Down")
    if UpperRollerLocation < 0:
        Actuation(UpperRoller,abs(UpperRollerLocation),"Up")
        

    if LowerRollerLocation > 0:
        Actuation(LowerRoller,abs(LowerRollerLocation),"Down")
        
    if LowerRollerLocation < 0:
        Actuation(LowerRoller,abs(LowerRollerLocation),"Up")

    if StraigheningAssemblyLocaton > 0:
        Actuation(StraigheningAssembly,abs(StraigheningAssemblyLocaton),"Backward")
    if StraigheningAssemblyLocaton < 0:
         Actuation(StraigheningAssembly,abs(StraigheningAssemblyLocaton),"Forward")

def Bendcorrection(EndoscopeLength,EndoscopeDiameter,Sweeps): #this will make the movements required to correct a bend in the endosocpe 
     
    UpperRollerDistanceToEndoscope = 15 - EndoscopeDiameter
    LowerRollerDistanceToEndoscope = 15 - EndoscopeDiameter 
    #45mm between the rollers     
    #Actuation(UpperRoller,UpperRollerDistanceToEndoscope,"Down")
    #Actuation(LowerRoller, LowerRollerDistanceToEndoscope,"Up")
    for Sweep in range(0,Sweeps):
        Actuation(StraigheningAssembly,EndoscopeLength,"Forward")
        Actuation(StraigheningAssembly,EndoscopeLength,"Backward")
        print(Sweep)
    

    ResetPostion()

def Home(): #DO NOT USE THIS UNTIL ALL LIMIT SWITCHES ARE USED AND TESTED 
    Actuation(3,1000,Forward,1)
    Actuation(2,50,Down)
    Actuation(1,50,Up)
    
    InitalizePostion()

def InitalizePostion(): #This will reset all of the location variables to 0
    global UpperRollerLocation, LowerRollerLocation, EndoscopeAngle, StraigheningAssemblyLocaton
    UpperRollerLocation = 0 
    LowerRollerLocation = 0
    StraigheningAssemblyLocaton = 0
    EndoscopeAngle = 0

def LimitSwitch():
    global LimitMotor, LimitDirection
    print("LimitSwitch ",LimitMotor," Direction ", LimitDirection)
    match LimitMotor:
        case 1:
            Speed = 1200
            Steps = 1200
        case 2: 
            Speed = 1200
            Steps = 800
        case 3:
            Speed = 350
            Steps = 1200
        case 4:
            Speed = 600
            Steps = 1200
    if LimitDirection == 0:
        Dir = 1           
    elif LimitDirection == 1:
        Dir = 0
    Actuation(LimitMotor,10,Dir,FirstSend=1,LimitCheck=0)
    return
                   
#ResetPostion()
#InitalizePostion()
#Actuation(3,500,Forward,1)
#Actuation(LowerRoller,15,Down,1)
#Actuation(UpperRoller,5,Up)
#Actuation(UpperRoller,20,Up)
#Home()
#Actuation(StraigheningAssembly,200,Backward,1)
#Actuation(2,15,Down,1)
#Actuation(UpperRoller,15,Up)
#Actuation(UpperRoller,5,Up)
#Home()
