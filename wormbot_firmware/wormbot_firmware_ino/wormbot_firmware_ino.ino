
/* 
 Firmware for the wormbot 3.0
 

 
 */
 
 //defines for AXIS
 
 #define TOP 1
 #define BOTTOM 2
 #define LEFT 3
 #define RIGHT 4
 #define XAXIS 5
 #define YAXIS 6
 #define ZAXIS 7 
 
 


//pins for steppers
#define stpX 44
#define dirX 45
#define stpZ 40
#define dirZ 41
#define stpY 46
#define dirY 47


//pins for limit switches
#define XMAX 23
#define ZMAX 22
#define XMIN 25
#define YMIN 28
#define ZMIN 24
#define YMAX 29 



#define LAMPPIN 2
#define GFPPIN 4
#define CHERRYPIN 6


#define ACCELSTEPS 500
#define SLOWSPEED 800.0
#define TIMEOUT_COUNTER 10000000


#define MINMS 30 //11

#define NEGATIVE 0
#define POSITIVE 1




#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>


const int stepsPerRev = 200;  // change this to fit the number of steps per revolution
                                     // for your motor



const unsigned long MAXCOMMANDWAIT=360000; 
                                     
unsigned long lastcommandtime=0;

unsigned long maxmotorspin=0;

int lampPin = LAMPPIN; 
int gfpPin = GFPPIN;
int cherryPin = CHERRYPIN;

int tempPin = A1; 

bool DEBUGLIMITS=false;

int newmove=false;
long moveDistance=0;
long moveDeccel=0;
float moveSpeed;

  /*                                   
int dirPinX = mePort[PORT_1].s1;//the direction pin connect to Base Board PORT1 SLOT1
int stpPinX = mePort[PORT_1].s2;//the Step pin connect to Base Board PORT1 SLOT2

int dirPinY = mePort[PORT_2].s1;
int stpPinY = mePort[PORT_2].s2;

MeLimitSwitch xplimitSwitch(PORT_6,1);
MeLimitSwitch xmlimitSwitch(PORT_6,2);
MeLimitSwitch yplimitSwitch(PORT_3,1);
MeLimitSwitch ymlimitSwitch(PORT_3,2);                                     
*/

long curr_x=0;
long curr_y=0;
long curr_z=0;

long x_max=0;
long y_max=0;
long z_max=0;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;


const int smallestDelay = MINMS; // inverse of motor's top speed



boolean checkLimit(int switchpin){
  if (!digitalRead(switchpin)) {return true;} else {return false;}
}//end checklimitswitch




boolean checkAxisLimit(int axis, int adir){
  
  switch(axis){
  
    case XAXIS:
      if (adir==POSITIVE) {
        if (!digitalRead(XMAX)) {return true;} 
        return false; //both aren't at limit 
      }//end if positive
      
      if (adir==NEGATIVE) {
        if (!digitalRead(XMIN)) {return true;} 
        return false; //both aren't at limit 
      }//end if positive
      break;
      
   case YAXIS:
     if (adir==POSITIVE) {
        if (!digitalRead(YMAX)) {return true;} 
        return false; //both aren't at limit 
      }//end if positive
      
      if (adir==NEGATIVE) {
        if (!digitalRead(YMIN)) {return true;}
        return false; //both aren't at limit 
      }//end if positive
      break;
    
   case ZAXIS:
      if (adir==POSITIVE) {
        if (!digitalRead(ZMAX)) {return true;} 
        return false; //both aren't at limit 
      }//end if positive
      
      if (adir==NEGATIVE) {
        if (!digitalRead(ZMIN)) {return true;}
        return false; //both aren't at limit 
      }//end if positive
      break;
    
    
  }//end of switch
  
}//end checkaxislimit

// *** Functions for initializing distributions ***






void goto_machine_max(void){
  
 // move_to_xy(x_max + 100, y_max + 100);

  x_max = curr_x;
  y_max = curr_y;
  
  
}//end goto maching zero



void move_to_x(int x) {
  move_to_xyz(x, curr_y, curr_z);
}

void move_to_y(int y) {
 move_to_xyz(curr_x, y, curr_z);
}

void move_to_z(int z) {
 move_to_xyz(curr_x, curr_y, z);
}

void zero_all_axes(void){
  
  //set all motors to negative
  digitalWrite(dirX,NEGATIVE);
  digitalWrite(dirY,NEGATIVE);
  digitalWrite(dirZ,NEGATIVE);
  
  
  boolean allZero,Xzero,Yzero,Zzero = false;
 
 
  Serial.print("xzero:");
  Serial.println(Xzero);
  
  while(1){
        
  
    if (!checkAxisLimit(XAXIS,NEGATIVE)) digitalWrite(stpX, HIGH);
    delayMicroseconds(MINMS);
    if (!checkAxisLimit(XAXIS,NEGATIVE)) digitalWrite(stpX, LOW);
    delayMicroseconds(MINMS);
    if (checkAxisLimit(XAXIS,NEGATIVE)) {
     Serial.println("x is zero");
        break;
    }
    //Serial.println("x while not zero");
  }//end while all switches not at zero 
  
  
  
  while(1){
        
    
    if (!checkAxisLimit(YAXIS,NEGATIVE)) digitalWrite(stpY, HIGH); 
   delayMicroseconds(MINMS);
    if (!checkAxisLimit(YAXIS,NEGATIVE)) digitalWrite(stpY, LOW); 
    delayMicroseconds(MINMS);
    

        
    if (checkAxisLimit(YAXIS,NEGATIVE)) {
        Serial.println("y is zero");
        break;
    }
       // Serial.println("y while zero");

  }//end while all switches not at zero 
  
  
  
  while(1){
           
    
    if (!checkAxisLimit(ZAXIS,NEGATIVE)) {digitalWrite(stpZ, HIGH); } 
   delayMicroseconds(MINMS);
    if (!checkAxisLimit(ZAXIS,NEGATIVE)) {digitalWrite(stpZ, LOW); } 
    delayMicroseconds(MINMS);
    
    

        
    if (checkAxisLimit(ZAXIS,NEGATIVE)) {
        Serial.println("z is zero");
        break;
    }
        //  Serial.println("z while zero");stuck

  }//end while all switches not at zero 
  
  
  
  curr_x=0;
  curr_y=0;
  curr_z=0;
  return;
  
}//end zero_all axes





void move_to_xyz(long x, long y, long z) {
 
  long dx = x - curr_x;
  long dy = y - curr_y; 
  long dz = z - curr_z;
  if (dx == 0 && dy == 0 && dz ==0) {
    delay(1);
    return;
  }
  
  int limitSwitchX;
  int limitSwitchY;
  int limitSwitchZ;
  
  long numStepsX;
  long numStepsY;
  long numStepsZ;
  
  int xdir,ydir,zdir=0;
  
  if (dx < 0) {
    xdir = NEGATIVE;
    numStepsX = -1 * dx;
  } else {
    xdir = POSITIVE;
    numStepsX = dx;
  }
  
  if (dy < 0) {
    ydir = NEGATIVE;
    numStepsY = -1 * dy;
  } else {
    ydir = POSITIVE;
    numStepsY = dy;
  }
  
   if (dz < 0) {
    zdir = NEGATIVE;
    numStepsZ = -1 * dz;
  } else {
    zdir = POSITIVE;
    numStepsZ = dz;
  }
  
  
  long i_x = 0;
  long i_y = 0;
  long i_z = 0;
  boolean x_reached = (numStepsX == 0) || checkAxisLimit(XAXIS,xdir);
  boolean y_reached = (numStepsY == 0) || checkAxisLimit(YAXIS,ydir);
  boolean z_reached = (numStepsZ == 0) || checkAxisLimit(ZAXIS,zdir);

  
  //set motor directions
  digitalWrite(dirX,xdir);
  digitalWrite(dirY,ydir);
  digitalWrite(dirZ,zdir);

 
    while(!x_reached) {
     
                                       
    
      digitalWrite(stpX, HIGH);
      delayMicroseconds(MINMS);
      digitalWrite(stpX, LOW);
      delayMicroseconds(MINMS);
     
      
      
      i_x++;
      x_reached = (i_x == numStepsX) || checkAxisLimit(XAXIS,xdir);
    }
    
    delay(MINMS);
    
   
    while (!y_reached) {
                                       
    
      digitalWrite(stpY, HIGH);
      delayMicroseconds(MINMS);
       digitalWrite(stpY, LOW);
       delayMicroseconds(MINMS);
      
      
      
      i_y++;
      y_reached = (i_y == numStepsY) || checkAxisLimit(YAXIS,ydir);
    }
    
    while (!z_reached ) {
                                           
      digitalWrite(stpZ, HIGH);
      delayMicroseconds(MINMS);
       digitalWrite(stpZ, LOW);
       delayMicroseconds(MINMS);
      
      
      
      i_z++;
      z_reached = (i_z == numStepsZ) || checkAxisLimit(ZAXIS,zdir);
    }
    

  
  if (dx < 0) curr_x -= i_x;
  else        curr_x += i_x;
  
  if (dy < 0) curr_y -= i_y;
  else        curr_y += i_y;
  
  if (dz < 0) curr_z -= i_z;
  else        curr_z += i_z;
  
  if (checkLimit(XMIN) && xdir == NEGATIVE) curr_x=0;
  if (checkLimit(YMIN) && ydir == NEGATIVE) curr_y=0;
  if (checkLimit(ZMIN) && zdir == NEGATIVE) curr_z=0;
  

  
  
}//end 






void setLamp(int intensity){
  //if (intensity >= 99) digitalWrite(lampPin, HIGH);
 // if (intensity <= 0) digitalWrite(lampPin,LOW);
   analogWrite(lampPin, intensity);
  
  
}//end setLamp

void setGFP(int intensity){
  //if (intensity >= 99) digitalWrite(lampPin, HIGH);
 // if (intensity <= 0) digitalWrite(lampPin,LOW);
   analogWrite(gfpPin, intensity);
  
  
}//end setLamp

void setCherry(int intensity){
  //if (intensity >= 99) digitalWrite(lampPin, HIGH);
 // if (intensity <= 0) digitalWrite(lampPin,LOW);
   analogWrite(cherryPin, intensity);
  
  
}//end setLamp

/* //june11

void newMachineMax(void){
  
   digitalWrite(dirPinX,POSITIVE); //set x pin positive
  while (!checkLimit(XMAX)){
    
      digitalWrite(stpPinX, HIGH);
      delayMicroseconds(MINMS);
      digitalWrite(stpPinX, LOW);
      delayMicroseconds(MINMS);
      curr_x++;
    
  }//end while not at x ==max  
  
  digitalWrite(dirPinY,POSITIVE); //set x pin positive
  while (!checkLimit(YMAX)){
    
      digitalWrite(stpPinY, HIGH);
      delayMicroseconds(MINMS);
      digitalWrite(stpPinY, LOW);
      delayMicroseconds(MINMS);
      curr_y++;
    
  }//end while not at y ==max  

  Serial.println("xmax="+ String(curr_x) + " ymax=" + String(curr_y));
  
}//end newmachine max


void jasonCalibrate(void){
  newMachineZero();
  newMachineMax();
  
}//end jason calibrate
*/ //june11


void quickCalibrate(void){
  
  zero_all_axes();
  delay(3);
  move_to_xyz(3000,3000,3000);
  delay(3);
  zero_all_axes();
  
  
}//end quick calibrate



void setup() {
  
  analogReference(INTERNAL1V1);
  
  // initialize the serial port:
  Serial.begin(9600);
  
  //setup stepper pins
  pinMode(stpX, OUTPUT);
  pinMode(dirX, OUTPUT);
  pinMode(stpY, OUTPUT);
  pinMode(dirY, OUTPUT);
  pinMode(stpZ, OUTPUT);
  pinMode(dirZ, OUTPUT);
  
    
  
  //setup lamp
  pinMode(lampPin, OUTPUT);
   pinMode(gfpPin, OUTPUT);
    pinMode(cherryPin, OUTPUT);
  
 
  //setup limit switches
  pinMode(XMAX, INPUT_PULLUP);
  pinMode(YMAX, INPUT_PULLUP);
  pinMode(XMIN, INPUT_PULLUP);
  pinMode(YMIN, INPUT_PULLUP);
  pinMode(ZMAX, INPUT_PULLUP);
  pinMode(ZMIN, INPUT_PULLUP);

  
  
  //initNormal();
  //initQuadratic();
  //Serial.println("Initializing movement curves...");
 // initInverse();
  
 // calibrate(); //un comment
 //jasonCalibrate();
 //zero_all_axes(); //2020
 //quickCalibrate();// 2020
 //goto_machine_zero();
 //goto_machine_max();
 //goto_machine_zero();
  //Serial.println("RR");
  
}//end setup

void readTemp(void){
  long volts; 
  int i=0;
  for (i=0; i < 999; i++) {
    volts+= analogRead(tempPin);
    //Serial.println(analogRead(tempPin));
  }
  //Serial.print("AVE");
  float aveVolt = (float)volts / 1000.000;
  //Serial.println(aveVolt);
  
  float mvlt= (aveVolt* 1100.000) / 1024.000;
  float ten=10.000;
  float f= mvlt/ten;

float c= (f - 32.000)/1.8000;
//c+=4.000; //calibration 1-31-2021

Serial.print ("C*");
Serial.println(c);

}//end readtemp

void loop(){
  //CHECK LIMIT SWITCHES
 if (DEBUGLIMITS){ 
   if (checkLimit(XMAX)) Serial.println(" X MAX");
    if (checkLimit(YMAX)) Serial.println("Y MAX");
     if (checkLimit(XMIN)) Serial.println("X Min");
     if (checkLimit(YMIN)) Serial.println("Y Min");
     if (checkLimit(ZMAX)) Serial.println("Z max");
     if (checkLimit(ZMIN)) Serial.println("Z min");
 

 }//end if limit debug

  
  
  if (stringComplete){
    //parse command
    //
    
    
   
    if  (inputString.indexOf("ZZ") >=0){
   //   goto_machine_zero();
      //newMachineZero();
      zero_all_axes();
    } else 
    if  (inputString.indexOf("LL") >=0){
      
      //max_all_axes();
      
    } else
    if (inputString.indexOf("CC")>=0){
	quickCalibrate();
    } else
    if (inputString.indexOf("IL") >=0){
        String lightamount = inputString.substring(inputString.indexOf("IL")+2);
        int lumos = lightamount.toInt();
        setLamp(lumos);
        
    } else
    if (inputString.indexOf("GL") >=0){
        String lightamount = inputString.substring(inputString.indexOf("GL")+2);
        int lumos = lightamount.toInt();
        setGFP(lumos);
        
    } else
    if (inputString.indexOf("TR") >=0){
        readTemp();
    } else
    if (inputString.indexOf("CL") >=0){
        String lightamount = inputString.substring(inputString.indexOf("CL")+2);
        int lumos = lightamount.toInt();
        setCherry(lumos);
        
    } else
    if (inputString.indexOf("P")>=0){ 
      Serial.print (curr_x);
      Serial.print (",");
      Serial.print(curr_y);
      Serial.print (",");
      Serial.println (curr_z);
      
    }else //end if p
    if(inputString.indexOf("MX") >=0){
      String moveamnt = inputString.substring(inputString.indexOf("MX")+2);
      long mv_x=moveamnt.toInt();
      if (mv_x <0) mv_x=0;
      move_to_x(mv_x);
    } //
    else
    if(inputString.indexOf("MZ") >=0){
      String moveamnt = inputString.substring(inputString.indexOf("MZ")+2);
      long mv_z=moveamnt.toInt();
      if (mv_z <0) mv_z=0;
      move_to_z(mv_z);
    } //
    else 
    if(inputString.indexOf("MY") >=0){
      String moveamnt = inputString.substring(inputString.indexOf("MY")+2);
      long mv_y=moveamnt.toInt();
      if (mv_y <0) mv_y=0;
      move_to_y(mv_y);
    } else
   
    if (inputString.indexOf("M") >= 0 && inputString.indexOf(",") >= 0) {
      String x_str = inputString.substring(inputString.indexOf("M") + 1,  inputString.indexOf(","));
      long x = x_str.toInt();
      String noX_str = inputString.substring(inputString.indexOf(",") + 1);
      long y = noX_str.substring(0,noX_str.indexOf(",")).toInt();
      long z = noX_str.substring(noX_str.indexOf(",") + 1).toInt();
      //Serial.println(String(x) + "-" + String(y) + "-" + String(z));
      move_to_xyz(x, y, z);
      
    }
    
    
    
    //Serial.println(inputString);
    inputString="";
    stringComplete=false;
    Serial.println("RR");
    //reset command timer
    lastcommandtime=millis();
    
  }//end if complete
  
  unsigned long currtime;
  currtime = millis();
  
  //if ((currtime - lastcommandtime) > MAXCOMMANDWAIT){
   // Serial.println("RR");
   // lastcommandtime=millis();
 // }//end if timeout  
  
}//end loop

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}
