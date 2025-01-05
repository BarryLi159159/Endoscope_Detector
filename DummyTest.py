import serial
import time
import math
import os

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

LENGTH_OF_HORRIZONTAL_LS = 485  # length of the workable space on horizontal lead screw in mm double check this before we start 
LENGTH_OF_VERTICAL_LS = 139.7  # length of the workable space on vertical lead screw in mm, double check this as well
SWEEP_COUNT = 10   # number of sweeps to be done

EndoscopeDiameter = 2.7 # radius of the endoscope in mm
EndoscopeLength = 300 # length of the endoscope in mm
EndoscopeRunout = 12.7

CenterOffsetFormula = (0.2 * EndoscopeDiameter + 1.5 * EndoscopeRunout) # less than 4 Endoscoep runout = 4. if more than 4
CenterOffset =  4 #endoscopes meet x mm below the center 

def OffsetFormula():
    global EndoscopeRunout, EndoscopeDiameter
    DiaCo = 0.2
    if EndoscopeRunout <= 3:
        RunoutCo = 2.1
    else: 
        RunoutCo = 1
    OffsetForm = EndoscopeDiameter * DiaCo + EndoscopeRunout * RunoutCo
    if OffsetForm > 8:
        OffsetForm = 8
    print (OffsetForm)
    return OffsetForm

UpperRollerDistanceToCenterOfScope = (36.75 + 2 + 0.5) + OffsetFormula() #3.5cm
LowerRollerDistanceToCenterOfScope = (21.25 - 0.5) - OffsetFormula() # distance from the upper roller to the scope is 1.5 cm

RadiusOfRoller = 10



StraighteningCount = 0 #number of times the endoscope has been straightened
MeasurmentCount = 0 #number of times the endoscope has been measured

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


#OffsetFormula = (Diameter * 



def MotorMove(Motor,Dir,Speed,Steps,LimitCheck = 0): #Raw movement command 
    Spd = str(Speed).rjust(4, '0')
    dataStr = str(Motor) + str(Dir) + str(LimitCheck) + str(Spd) + str(Steps) + "\n"
    arduino.write(dataStr.encode())

def Actuation(Motor,Distance,Direction,FirstSend = 0,LimitCheck = 1,VideoCapture = 0):
    #time.sleep(0.5)
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

            Nema17Speed = 700 #Speed values are the microseconds between pulses, lower number equals higher speed. 
            Nema17Speed4Digit = str(Nema17Speed).rjust(4, '0') #allows for any 4 digit number to be used 
            Nema23Speed =  140 #Stalls at 300 - somestimes stalls due to a part stuck on the shaft at the end of the system!
            Nema23Speed4Digit = str(Nema23Speed).rjust(4, '0')
            
            MMPerRotNema17 = 0.635 #mm

            StepsPerMMNema17 = 200 * (1/MMPerRotNema17)

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
            if VideoCapture == 1:
                Speed = 1200
            if LimitCheck == 0:
                Speed = 1000
            if LimitCheck == 0 and Motor == 3:
                Steps = 10 * StepsPerMMNema23 #So it moves back further 
                print("Larger Limit movement worked")
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
        Actuation(UpperRoller,abs(UpperRollerLocation),Down)
    if UpperRollerLocation < 0:
        Actuation(UpperRoller,abs(UpperRollerLocation),Up)
        

    if LowerRollerLocation > 0:
        Actuation(LowerRoller,abs(LowerRollerLocation),Down)
        
    if LowerRollerLocation < 0:
        Actuation(LowerRoller,abs(LowerRollerLocation),Up)

    if StraigheningAssemblyLocaton > 0:
        Actuation(StraigheningAssembly,abs(StraigheningAssemblyLocaton),Backward)
    if StraigheningAssemblyLocaton < 0:
         Actuation(StraigheningAssembly,abs(StraigheningAssemblyLocaton),Forward)

def Bendcorrection(EndoscopeLength,UpperRollerDistanceToScope,LowerRollerDistanceToScope,Sweeps): #this will make the movements required to correct a bend in the endosocpe 
    Actuation(LowerRoller, LowerRollerDistanceToScope,Up)
    Actuation(UpperRoller,UpperRollerDistanceToScope,Down)
    EnodscopeRollDistance = (EndoscopeLength - 90)
    for Sweep in range(0,Sweeps):
        Actuation(StraigheningAssembly,EnodscopeRollDistance,Forward)
        Actuation(StraigheningAssembly,EnodscopeRollDistance,Backward)
        print(Sweep)
    
    Home()
    #ResetPostion()

def Home():
    Actuation(2,100,Down,1)
    Actuation(1,100,Up)
    Actuation(3,1000,Forward)
    
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
    if LimitDirection == 0:
        Dir = 1           
    elif LimitDirection == 1:
        Dir = 0
    Actuation(LimitMotor,1.25,Dir,FirstSend=1,LimitCheck=0) #1.25
    return                   

def checkifatstart(): # to be updated when limit switches are installed
    Home()
    return True

def straighten_endoscope(EndoscopeDiameter, EndoscopeLength):
    """
    Straighten the endoscope.
    """
    global StraighteningCount
    global LowerRollerDistanceToCenterOfScope, UpperRollerDistanceToCenterOfScope, RadiusOfRoller
    # calculate the necessary distances
    DISTANCE_TO_START = (LENGTH_OF_HORRIZONTAL_LS) - 10 #The working length of the horizontal lead screw - the radius of the plate - the distance from the wall to the start point (atleast 80mm from the wall)
    LowerDistance =  LowerRollerDistanceToCenterOfScope - (EndoscopeDiameter/2) - RadiusOfRoller 
    RaiseDistance =  UpperRollerDistanceToCenterOfScope - (EndoscopeDiameter/2) - RadiusOfRoller 
    if LowerDistance < 0:
        LowerDistance = 0
    if RaiseDistance < 0 :
        RaiseDistance = 0
    # move the straightening device to the start position
    Actuation(3,DISTANCE_TO_START,Backward,1)

    Bendcorrection(EndoscopeLength,RaiseDistance,LowerDistance, SWEEP_COUNT)

    # move the straightening device to the rest position
    ResetPostion()

    StraighteningCount = StraighteningCount + 1
Home()
straighten_endoscope(EndoscopeDiameter,EndoscopeLength)
#Actuation(3,LENGTH_OF_HORRIZONTAL_LS,Backward,1)

    







