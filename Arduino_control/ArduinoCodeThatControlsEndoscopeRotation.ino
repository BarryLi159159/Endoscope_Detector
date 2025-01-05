
//LowerRoller
#define EN_PIN2    35 //enable (Motor 2 on Mega pin 35
#define STEP_PIN2  37 //step (Motor 2 on Mega pin 37
#define DIR_PIN2   39 //direction(Motor 2 on Mega pin 39

//Upper Roller
#define EN_PIN1    45 //enable (Motor 2 on Mega 35, motor 1,9
#define STEP_PIN1  47 //step (Motor 2 on Mega 37, motor 1,8 )
#define DIR_PIN1   49 //direction(Motor 2 on Mega 39 Motor 1,7)

// BasePlate
#define EN_PIN3  25 //enable (Motor 2 on Mega 35, motor 1,9
#define STEP_PIN3 27
#define DIR_PIN3 29

String data; // this initalizes the data variable 
const byte numChars = 32;
boolean newData = false;
const unsigned long TimeOut = 10; // This creates a buffer of reading the data 
#define LED  13 


void setup() {
  
  pinMode(EN_PIN1, OUTPUT); // set the EN_PIN as an output
  digitalWrite(EN_PIN1, HIGH); // deactivate driver (LOW active)
  pinMode(DIR_PIN1, OUTPUT); // set the DIR_PIN as an output
  digitalWrite(DIR_PIN1, LOW); // set the direction pin to low
  pinMode(STEP_PIN1, OUTPUT); // set the STEP_PIN as an output
  digitalWrite(STEP_PIN1, LOW); // set the step pin to low
  digitalWrite(EN_PIN1, LOW); // activate driver

  //Motor 2
  pinMode(EN_PIN2, OUTPUT); // set the EN_PIN as an output
  digitalWrite(EN_PIN2, HIGH); // deactivate driver (LOW active)
  pinMode(DIR_PIN2, OUTPUT); // set the DIR_PIN as an output
  digitalWrite(DIR_PIN2, LOW); // set the direction pin to low
  pinMode(STEP_PIN2, OUTPUT); // set the STEP_PIN as an output
  digitalWrite(STEP_PIN2, LOW); // set the step pin to low
  digitalWrite(EN_PIN2, LOW); // activate driver
   //Motor 3
  
  pinMode(EN_PIN3, OUTPUT); // set the EN_PIN as an output
  digitalWrite(EN_PIN3, HIGH); // deactivate driver (LOW active)
  pinMode(DIR_PIN3, OUTPUT); // set the DIR_PIN as an output
  digitalWrite(DIR_PIN3, LOW); // set the direction pin to low
  pinMode(STEP_PIN3, OUTPUT); // set the STEP_PIN as an output
  digitalWrite(STEP_PIN3, LOW); // set the step pin to low
  digitalWrite(EN_PIN3, LOW); // activate driver

  Serial.begin(9600); // Opens up the serial port for serial communication with Python 
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  
  data = ""; // Data is empty
}


void rotateMotor(int motor, long steps, bool direction, int speed) { // This is the function that controlls the motors 
  switch (motor) { // This switch casing is how different motors are selected
    case 1: //Case 1 for motor 1
    // Set the motor direction
    digitalWrite(DIR_PIN1, direction);
    for (long i = 0; i < steps; i++)  //for loop for number of steps 
    { //to move the motor the step/pulse pin goes HIGH then LOW
    digitalWrite(STEP_PIN1, HIGH);
    delayMicroseconds(speed); //Interval between HIGH and LOW for step/pulse lower this value, faster it goes 
    digitalWrite(STEP_PIN1, LOW);
    }
  break;
    case 2: // Case 2 for motor 2 
    
    digitalWrite(DIR_PIN2, direction);
    for (long i = 0; i < steps; i++) 
    {
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds(speed);
    digitalWrite(STEP_PIN2, LOW);
    }
  break;
    case 3: // Case 2 for motor 3
    
    digitalWrite(DIR_PIN3, direction);
    for (long i = 0; i < steps; i++) 
    {
    digitalWrite(STEP_PIN3, HIGH);
    delayMicroseconds(speed);
    digitalWrite(STEP_PIN3, LOW);
    }
  break;
  }
  
}


void loop() {
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

  if ((data.length() > 6) && newData == true)  //(data.substring(0,1) == '1') //Motor forward
  {
    pinMode(LED,HIGH);
    String StrMot = data.substring(0,1); //Reads the value of the motor 
    int IntMot = StrMot.toInt();

    String StrDir = data.substring(1,2); //reasds the 1 digit direction value 
    int IntDir = StrDir.toInt();

    String StrSpd = data.substring(2,4); //Reads the 2 digit speed value from Python, converts in into an int
    int IntSpd = StrSpd.toInt();


    String StrStep = data.substring(4); //Reads the number for steps sent from pyhton, converts them into an integer
    long IntStep = StrStep.toInt();

    rotateMotor(IntMot, IntStep, IntDir, IntSpd); //Calls the control funtion with our decoded values 
    delay(100);
    data = "";//Empties the data point
    newData = false;
    pinMode(LED,LOW);
  }
}