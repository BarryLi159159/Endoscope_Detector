//Features of this code!
//Acceleration
//two way signaling
//All 4 motors (as we've defined now)
//Has Proximity Sensors


//DOES NOT HAVE
//Limit Switches 
//Emergancy stop (?)
//Not More V.5 any new additon, that works will replace the current 
//if a small change or have a new version number if a large change 

//LowerRoller
#define EN_PIN2    31 //enable (Motor 2 on Mega pin 35
#define DIR_PIN2   33 //direction(Motor 2 on Mega pin 39
#define STEP_PIN2  35 //step (Motor 2 on Mega pin 37
#define Limit_SwitchLow A2

//Upper Roller Blue Tape Wire
#define EN_PIN1    38//enable (Motor 2 on Mega 35, motor 1,9
#define DIR_PIN1   40 //direction(Motor 2 on Mega 39 Motor 1,7)
#define STEP_PIN1  42 //step (Motor 2 on Mega 37, motor 1,8 )
#define Limit_SwitchUp A1

// BasePlate
#define EN_PIN3  51 //enable (Motor 2 on Mega 35, motor 1,9
#define DIR_PIN3 49
#define STEP_PIN3 47
#define Limit_SwitchBack A3 //This will go to the Endoscope mounted end 
#define Limit_SwitchFront A4 //This will go to the Horitontal Motor mounted end 
#define Limit_Endoscope A5

// Endoscope Motor 
#define EN_PIN4 22 // fine...
#define DIR_PIN4 24
#define STEP_PIN4 26






String data; // this initalizes the data variable 
const byte numChars = 32;
boolean newData = false;
const unsigned long TimeOut = 10; // This creates a buffer of reading the data 

int ThisLimit = 0;
int ThisSensor = 0;
int ThisEN = 0;
int ThisStep = 0;
int ThisDir = 0;
int LimitState = 0;
int ProxState = 0;
int ProxDirect = 0;
int LimitHit = 0;
int x = 1;
#define LED  13 


void setup() {
  
  pinMode(EN_PIN1, OUTPUT); // set the EN_PIN as an output
  digitalWrite(EN_PIN1, HIGH); // deactivate driver (LOW active)
  pinMode(DIR_PIN1, OUTPUT); // set the DIR_PIN as an output
  digitalWrite(DIR_PIN1, LOW); // set the direction pin to low
  pinMode(STEP_PIN1, OUTPUT); // set the STEP_PIN as an output
  digitalWrite(STEP_PIN1, LOW); // set the step pin to low

  //Motor 2
  pinMode(EN_PIN2, OUTPUT); // set the EN_PIN as an output
  digitalWrite(EN_PIN2, HIGH); // deactivate driver (LOW active)
  pinMode(DIR_PIN2, OUTPUT); // set the DIR_PIN as an output
  digitalWrite(DIR_PIN2, LOW); // set the direction pin to low
  pinMode(STEP_PIN2, OUTPUT); // set the STEP_PIN as an output
  digitalWrite(STEP_PIN2, LOW); // set the step pin to low
   //Motor 3
  
  pinMode(EN_PIN3, OUTPUT); // set the EN_PIN as an output
  digitalWrite(EN_PIN3, HIGH); // deactivate driver (LOW active)
  pinMode(DIR_PIN3, OUTPUT); // set the DIR_PIN as an output
  digitalWrite(DIR_PIN3, LOW); // set the direction pin to low
  pinMode(STEP_PIN3, OUTPUT); // set the STEP_PIN as an output
  digitalWrite(STEP_PIN3, LOW); // set the step pin to low

  pinMode(EN_PIN4, OUTPUT); // set the EN_PIN as an output
  digitalWrite(EN_PIN4, HIGH); // deactivate driver (LOW active)
  pinMode(DIR_PIN4, OUTPUT); // set the DIR_PIN as an output
  digitalWrite(DIR_PIN4, LOW); // set the direction pin to low
  pinMode(STEP_PIN4, OUTPUT); // set the STEP_PIN as an output
  digitalWrite(STEP_PIN4, LOW); // set the step pin to low

  
  pinMode(Limit_SwitchLow,INPUT_PULLUP);
  pinMode(Limit_SwitchUp,INPUT_PULLUP);
  pinMode(Limit_SwitchFront, INPUT_PULLUP);
  pinMode(Limit_SwitchBack,INPUT_PULLUP);
  pinMode(Limit_Endoscope,INPUT_PULLUP);

  Serial.begin(9600); // Opens up the serial port for serial communication with Python 
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  
  data = ""; // Data is empty
}

void LimitSwitch(int motor, bool direction)
{
  switch (motor){
    case 1: //Upper motor
    {
    ThisLimit = Limit_SwitchUp;
    ThisEN = EN_PIN1;
    }
    break;
    case 2: 
    {
    ThisLimit = Limit_SwitchLow;
    ThisEN = EN_PIN2;
    }
    break;
    case 3:
    {
    ThisEN = EN_PIN3;
    if (direction == 0){
    ThisLimit = Limit_SwitchFront;
    }
    else {
    ThisLimit = Limit_SwitchBack;
    }
    }
    break;
    case 4:
    {
    ThisLimit = Limit_Endoscope;
    ThisEN = EN_PIN4;
    }
    break;
  }
  LimitState = analogRead(ThisLimit);
  if (LimitState < 50)
    {
    digitalWrite(ThisEN, HIGH);
    LimitHit = 4;
    data = "";
    //Serial.println("Limit");
    }
  else
  {
    digitalWrite(ThisEN, LOW);
    //Serial.println("Limit: Untouched");
    return 0;
  }
}

void PythonLimitReverse()
{
  Serial.print("Limit");
}

void rotateMotor(int motor, long steps, bool direction, bool LimitCheck, int speed) { // This is the function that controlls the motors 
  LimitHit = 1;
  switch (motor) { // This switch casing is how different motors are selected
    case 1: { //Case 1 for motor 1 ADD LIMIT CHECK PYTHON SIDE FUNCTION TO THIS CASE!!
    digitalWrite(EN_PIN1, LOW);
    digitalWrite(DIR_PIN1, direction);
  
    for (long i = 0; i < steps; i++) 
      {
      digitalWrite(STEP_PIN1, HIGH);
      delayMicroseconds(speed);
      digitalWrite(STEP_PIN1, LOW);
      if (LimitCheck == 1) {
      LimitSwitch(motor,direction);
      }
      if  (LimitHit == 4)
      { 
      break;
      }
      }
      if (LimitHit == 4){
      PythonLimitReverse();
      return;
      }
    digitalWrite(EN_PIN1, HIGH);
    }
  break;
    case 2: {// Case 2 for motor 2
    digitalWrite(EN_PIN2, LOW); 
    digitalWrite(DIR_PIN2, direction);
    for (long i = 0; i < steps; i++) 
      {
      digitalWrite(STEP_PIN2, HIGH);
      delayMicroseconds(speed);
      digitalWrite(STEP_PIN2, LOW);
      LimitSwitch(motor,direction);
      if  (LimitHit == 4)
      { 
      break;
      }
      }

      if (LimitHit == 4){
      PythonLimitReverse();
      return;
      }
    digitalWrite(EN_PIN2, HIGH);
    
    }
  break;
    case 3: {// Set the motor direction
    LimitHit = 1;
    int startingSpeed = 600; // start with a relatively large delay so you know that it will start smoothly
    int accelSteps;
    int deaccelSteps;
    digitalWrite(EN_PIN3, LOW);
    digitalWrite(DIR_PIN3, direction);
    //acceleration phase
    accelSteps =  startingSpeed - speed;
    deaccelSteps = startingSpeed - speed;

    for (long i = 0; i < accelSteps ; i++)  //for loop for number of steps 
    { //to move the motor the step/pulse pin goes HIGH then LOW
    LimitSwitch(motor,direction);
    digitalWrite(STEP_PIN3, HIGH);
    delayMicroseconds(startingSpeed); //Interval between HIGH and LOW for step/pulse lower this value, faster it goes 
    startingSpeed = startingSpeed - 1; // update delay for next loop to be smaller
    digitalWrite(STEP_PIN3, LOW);
    digitalWrite(13,LOW);
    if  (LimitHit == 4)
    { //is not getting into here
      break;
    }
    }
    //constant speed phase
    for (long i = accelSteps; i < (steps - deaccelSteps); i++)  //for loop for number of steps 
    { //to move the motor the step/pulse pin goes HIGH then LO
    LimitSwitch(motor,direction);
    digitalWrite(STEP_PIN3, HIGH);
    delayMicroseconds(speed); //Interval between HIGH and LOW for step/pulse lower this value, faster it goes 
    digitalWrite(STEP_PIN3, LOW);
    if  (LimitHit == 4)
    { //is not getting into here
      break;
    }
    }
    for (long i = (steps - deaccelSteps); i < steps; i++) //deceleration phase
    {
    LimitSwitch(motor,direction);
    digitalWrite(STEP_PIN3, HIGH);
    delayMicroseconds(startingSpeed); //Interval between HIGH and LOW for step/pulse lower this value, faster it goes 
    startingSpeed = startingSpeed + 1; // update delay for next loop to be smaller
    digitalWrite(STEP_PIN3, LOW);
    if  (LimitHit == 4)
    { //is not getting into here
      break;
    }
    }
    if (LimitHit == 4){
      PythonLimitReverse();
      return;
    }
    digitalWrite(EN_PIN3, HIGH);
    }
  break;
    case 4:  {
    //LimitHit = 1; //Variable that checks if limit is hit resets at start of movement command
    digitalWrite(EN_PIN4, LOW);
    // Set the motor direction
    digitalWrite(DIR_PIN4, direction);
      for (long i = 0; i < steps; i++)  //for loop for number of steps 
    {
    LimitSwitch(motor,direction);
    digitalWrite(STEP_PIN4, HIGH);
    delayMicroseconds(speed); //Interval between HIGH and LOW for step/pulse lower this value, faster it goes 
    digitalWrite(STEP_PIN4, LOW);
    if  (LimitHit == 4)
    { //is not getting into here
      break;
    }
    }
    if (LimitHit == 4){
      PythonLimitReverse();
      return;
    }
    digitalWrite(EN_PIN4, HIGH);
    }
  break;
  }
  Serial.print("Done");
}


void loop() {
  //while (x = 1) {LimitSwitch(4,0);} // Testing to make sure limitswitchs are triggered
  static byte ndx = 0;
  char endMarker = '\n';
  char rc; 
  unsigned long T = 0; // timer
  T = millis(); // timer running
  while (millis() - T < TimeOut) {
    // waiting timeout
    while (Serial.available() > 0 && newData == false) {
      // receiving Serial
      rc = Serial.read();
      if (rc != endMarker){
        data += rc ;//add char
        ndx++ ; 
         T = millis();
        if (ndx >= numChars){
          ndx = numChars - 1;
        }
      }
      else {
        data.substring(ndx) = '\0' ;
        ndx = 0;
        newData = true;
      }
    }
  }

  if ((data.length() > 7) && newData == true)  //(data.substring(0,1) == '1') //Motor forward
  {
    //pinMode(LED,HIGH);
    String StrMot = data.substring(0,1); //Reads the value of the motor 
    int IntMot = StrMot.toInt();

    String StrDir = data.substring(1,2); //reasds the 1 digit direction value 
    int IntDir = StrDir.toInt();

    String StrLim = data.substring(2,3);
    int IntLim = StrLim.toInt();

    String StrSpd = data.substring(3,7); //Reads the 4 digit speed value from Python, converts in into an int
    int IntSpd = StrSpd.toInt();


    String StrStep = data.substring(7); //Reads the number for steps sent from pyhton, converts them into an integer
    long IntStep = StrStep.toInt();

    rotateMotor(IntMot, IntStep, IntDir, IntLim, IntSpd); //Calls the control funtion with our decoded values 
    delay(100);
    data = "";//Empties the data point
    newData = false;
    //pinMode(LED,LOW);
  }
}
