#include <LiquidCrystal.h>
#include <SoftwareSerial.h>  
#include <Wire.h>
//#define bluetoothTx 5
//#define bluetoothRx 9


#define DEVICE (0x53) // Device address as specified in data sheet 
#define unit8 unsigned char
int bluetoothTx = 10;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 8;  // RX-I pin of bluetooth mate, Arduino D3
byte _buff[6];
byte _buff1[6];
int d;
 int a;
 int a1;
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

LiquidCrystal lcd(12, 11, 6, 4, 3, 2);
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

/*  VARIABLES for pulse sensor   */
int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin
int MledPin = 7;//pin to lighten led every 1 minite
/* VARIABLES FOR TEMPERATURE SENSOR*/
int photocellPin = 1; //  Temperature Sensor output wire connected to analog pin 1
int photocellVal = 0; // photocell variable
int mytemperature1 = 0;//the current temperature
int ledPin = 9;// led for temperature test
int ledState = 0;
int mintemperature = 30;


/* VARIABLES FOR INTERRUPTION
these variables are volatile because they are used during the interrupt service routine!*/
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, the Inter-Beat Interval
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

volatile int rate[10];                    // used to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long MinCounter = 0; 
volatile unsigned long SecCounter = 0; 
volatile boolean MLed = false ;
volatile int beatsCounter = 0; 
volatile boolean flag = false;//time counter flag 
volatile unsigned long lastBeatTime = 0;           // used to find the inter beat interval
volatile int P =512;                      // used to find peak in pulse wave
volatile int T = 512;                     // used to find trough in pulse wave
volatile int thresh = 512;                // used to find instant moment of heart beat
volatile int amp = 100;                   // used to hold amplitude of pulse waveform
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = true;       // used to seed rate array so we startup with reasonable BPM


void setup(){
   Wire.begin();  
  pinMode(ledPin, OUTPUT);// pin that will light when
  lcd.begin(16, 2);//lcd setup
  pinMode(blinkPin,OUTPUT);         // pin that will blink to your heartbeat!
  pinMode(MledPin,OUTPUT);
  pinMode(fadePin,OUTPUT);          // pin that will fade to your heartbeat!
  //Serial.begin(115200);             // we agree to talk fast!
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
   // UN-COMMENT THE NEXT LINE IF YOU ARE POWERING The Pulse Sensor AT LOW VOLTAGE, 
   // AND APPLY THAT VOLTAGE TO THE A-REF PIN
   //analogReference(EXTERNAL); 
   Serial.begin(9600);//serial output baud rate
     bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600
   //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeTo(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeTo(POWER_CTL, 0x08);
}



void loop(){
/*PULSE SAMPLE*/
Pulse_Sample();
/*Temperature sample*/
Temperature_sample(); 
fall();
char recvChar;
  if(bluetooth.available())  // If the bluetooth sent any characters
  {
    recvChar=bluetooth.read();
    // Send any characters the bluetooth prints to the serial monitor
   Serial.print(recvChar);  
    bluetooth.print(recvChar);
    if(recvChar=='t'){
    Serial.println(mytemperature1);
    bluetooth.println(mytemperature1);
    }
    if(recvChar=='b'){
    Serial.println(beatsCounter);
    bluetooth.println(beatsCounter);
    }
//   if(recvChar=='f'){
//      if(d>10||d<-10){
//    Serial.println("Fall down");
//    bluetooth.println("Fall down");
//  }
  //  }
     
  }
  if(Serial.available())  // If stuff was typed in the serial monitor
  {
    recvChar =Serial.read();
    // Send any characters the Serial monitor prints to the bluetooth
   bluetooth.println(recvChar);
   Serial.println(recvChar);  
    if(recvChar=='t'){
    bluetooth.println(mytemperature1);
    Serial.println(mytemperature1);
    }
//    if(recvChar=='b'){
//    bluetooth.println(beatsCounter);
//    Serial.println(beatsCounter);
//    }
     //if(recvChar=='f'){
     // if(d>10||d<-10){
   // Serial.println("Fall down");
   // bluetooth.println("Fall down");
 // }
    //}
  }
  // and loop forever and ever!



}
void fall(){
 readAccel(); // read the x/y/z tilt
  delay(500); // only read every 0,5 seconds
  readAccel1();
  delay(500);
  detect();
  delay(500);
  if(d>10||d<-10){
    Serial.println("Fall down");
  }

}

void readAccel() {
  uint8_t howManyBytesToRead = 6;
  readFrom( DATAX0, howManyBytesToRead, _buff); //read the acceleration data from the ADXL345

  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  int x = (((int)_buff[1]) << 8) | _buff[0];   
  int y = (((int)_buff[3]) << 8) | _buff[2];
  int z = (((int)_buff[5]) << 8) | _buff[4];

//  Serial.print("x: ");
//  Serial.print( x );
//  Serial.print(" y: ");
//  Serial.print( y );
//  Serial.print(" z: ");
//  Serial.println( z );
  a=z;
}

void readAccel1() {
  uint8_t howManyBytesToRead = 6;
  readFrom1( DATAX0, howManyBytesToRead, _buff1); //read the acceleration data from the ADXL345

  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  int x1 = (((int)_buff1[1]) << 8) | _buff1[0];   
  int y1 = (((int)_buff1[3]) << 8) | _buff1[2];
  int z1 = (((int)_buff1[5]) << 8) | _buff1[4];

//  Serial.print("x1: ");
//  Serial.print( x1 );
//  Serial.print(" y1: ");
//  Serial.print( y1 );
//  Serial.print(" z1: ");
//  Serial.println( z1 );
  a1=z1;
  
}


void detect(){
 d =a1-a;
// Serial.println( d);
//if(d>10 ||d<-10){
//Serial.println( "Fall down" );
//}

}

void writeTo(byte address, byte val) {
  Wire.beginTransmission(DEVICE); // start transmission to device 
  Wire.write(address);             // send register address
  Wire.write(val);                 // send value to write
  Wire.endTransmission();         // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void readFrom(byte address, int num, byte _buff[]) {
  Wire.beginTransmission(DEVICE); // start transmission to device 
  Wire.write(address);             // sends address to read from
  Wire.endTransmission();         // end transmission

  Wire.beginTransmission(DEVICE); // start transmission to device
  Wire.requestFrom(DEVICE, num);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())         // device may send less than requested (abnormal)
  { 
    _buff[i] = Wire.read();    // receive a byte
    i++;
  }
  Wire.endTransmission();         // end transmission
}

void readFrom1(byte address, int num, byte _buff1[]) {
  Wire.beginTransmission(DEVICE); // start transmission to device 
  Wire.write(address);             // sends address to read from
  Wire.endTransmission();         // end transmission

  Wire.beginTransmission(DEVICE); // start transmission to device
  Wire.requestFrom(DEVICE, num);    // request 6 bytes from device

  int j = 0;
  while(Wire.available())         // device may send less than requested (abnormal)
  { 
    _buff1[j] = Wire.read();    // receive a byte
    j++;
  }
  Wire.endTransmission();         // end transmission
}
/*PULSE SAMPLE PART*/
void Pulse_Sample(){
//sendDataToProcessing('S', Signal);     // send Processing the raw Pulse Sensor data
  if (QS == true){                       // Quantified Self flag is true when arduino finds a heartbeat
        fadeRate = 255;                  // Set 'fadeRate' Variable to 255 to fade LED with pulse
        //sendDataToProcessing('B',BPM);   // send heart rate with a 'B' prefix
        //sendDataToProcessing('Q',IBI);   // send time between beats with a 'Q' prefix
        QS = false;                      // reset the Quantified Self flag for next time    
     }
  /*if(flag == true){
  if (MinCounter<60){
    
     if (SecCounter<500){
     SecCounter += 1;//count ms until 1 s
   }
     else{
       
     MinCounter += 1;//count s until 1 min
     Serial.println( MinCounter);
     SecCounter = 0;//reload;
   }
  }
   else{
   MLed = true;//FLAG for minite led
  // beatsCounter = 0;//reset beats
   MinCounter = 0;//reload
   }
   flag = false;
  }
  
  ledFadeToBeat();
    delay(20); //  take a break
  if (MLed == true){
    Serial.print("The beat is:" );
    Serial.println(beatsCounter); 
    digitalWrite(MledPin, HIGH);
    delay(50);
    beatsCounter = 0;//reset beats
  }*/
  ledFadeToBeat();
    delay(20); //  take a break
}

void ledFadeToBeat(){
    fadeRate -= 15;                         //  set LED fade value
    fadeRate = constrain(fadeRate,0,255);   //  keep LED fade value from going into negative numbers!
    analogWrite(fadePin,fadeRate);          //  fade LED
  }


//void sendDataToProcessing(char symbol, int data ){
    //Serial.print(symbol);                // symbol prefix tells Processing what type of data is coming
   // Serial.println(data);                // the data to send culminating in a carriage return
  //}

/*TRMPERATURE PART*/
void Temperature_sample(){
photocellVal = analogRead(photocellPin);
  mytemperature1 = (photocellVal/10) - 25;
 // Serial.print("Requesting temperatures...");
  //Serial.println("DONE");
  //Serial.print("Temperature for this Device is: ");
  //Serial.print(mytemperature1);
  //Serial.println(" Degree"); 
//  lcd.setCursor(0, 0); 
//  lcd.print("Temperature Now:");
//  lcd.setCursor(0, 1); 
// lcd.print(mytemperature1);
  
  if (mytemperature1 <= mintemperature && ledState == 0){
    digitalWrite(ledPin, LOW); // turn on LED
    ledState = 1;
 //Serial.println("Temperature Normal."); 
//Serial.println(" ");
    lcd.setCursor(2, 1); 
    lcd.print(" C Normal ^_^"); 
  }
   if (mytemperature1 > mintemperature && ledState == 1) {
    digitalWrite(ledPin, HIGH); // turn off LED
    
    ledState = 0;
   Serial.println("Temperature TOO HIGH!"); 
    Serial.println(" ");
    lcd.setCursor(2, 1); 
    lcd.print(" C TOO HIGH!!!");  
  }  
    if (mytemperature1 <= mintemperature && ledState == 1){
    digitalWrite(ledPin, LOW); // turn on LED
    ledState = 1;
 //  Serial.println("Temperature Normal."); 
  //  Serial.println(" ");
    lcd.setCursor(2, 1); 
    lcd.print(" C Normal ^_^");  
  }
     if (mytemperature1 > mintemperature && ledState == 0) {
    digitalWrite(ledPin, HIGH); // turn off LED
    
    ledState = 0;
  Serial.println("Temperature TOO HIGH!"); 
   Serial.println(" "); 
    lcd.setCursor(2, 1); 
    lcd.print(" C TOO HIGH!!!"); 
  }  
  delay(500);       
}


/*INTERRUPTION*/
void interruptSetup(){     
  // Initializes Timer2 to throw an interrupt every 2mS.
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER 
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE THE INITIAL TIMEING:124
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED      
} 

// THIS IS THE TIMER 2 INTERRUPT SERVICE ROUTINE. 
// Timer 2 makes sure that we take a reading every 2 miliseconds
ISR(TIMER2_COMPA_vect){                         // triggered when Timer2 counts to 124
    //flag = true;
    cli();                                      // disable interrupts while we do this
    Signal = analogRead(pulsePin);              // read the Pulse Sensor 
    sampleCounter += 2;                         // keep track of the time in mS with this variable
    int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

//  find the peak and trough of the pulse wave
    if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
        if (Signal < T){                        // T is the trough
            T = Signal;                         // keep track of lowest point in pulse wave 
         }
       }
      
    if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
        P = Signal;                             // P is the peak
       }                                        // keep track of highest point in pulse wave
    
  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
if (N > 250){                                   // avoid high frequency noise
  if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){        
    Pulse = true;                               // set the Pulse flag when we think there is a pulse
    digitalWrite(blinkPin,HIGH);       // turn on pin 13 LED
    beatsCounter += 1;
    //Serial.println(beatsCounter);
    IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
    lastBeatTime = sampleCounter;               // keep track of time for next pulse
         
         if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
             firstBeat = false;                 // clear firstBeat flag
             return;                            // IBI value is unreliable so discard it
            }   
         if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
            secondBeat = false;                 // clear secondBeat flag
               for(int i=0; i<=9; i++){         // seed the running total to get a realisitic BPM at startup
                    rate[i] = IBI;                      
                    }
            }
          
    // keep a running total of the last 10 IBI values
    word runningTotal = 0;                   // clear the runningTotal variable    

    for(int i=0; i<=8; i++){                // shift data in the rate array
          rate[i] = rate[i+1];              // and drop the oldest IBI value 
          runningTotal += rate[i];          // add up the 9 oldest IBI values
        }
        
    rate[9] = IBI;                          // add the latest IBI to the rate array
    runningTotal += rate[9];                // add the latest IBI to runningTotal
    runningTotal /= 10;                     // average the last 10 IBI values 
    BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
    QS = true;                              // set Quantified Self flag 
    // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
}

  if (Signal < thresh && Pulse == true){     // when the values are going down, the beat is over
      digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
      Pulse = false;                         // reset the Pulse flag so we can do it again
      amp = P - T;                           // get amplitude of the pulse wave
      thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
      P = thresh;                            // reset these for next time
      T = thresh;
     }
  
  if (N > 2500){                             // if 2.5 seconds go by without a beat
      thresh = 512;                          // set thresh default
      P = 512;                               // set P default
      T = 512;                               // set T default
      lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
      firstBeat = true;                      // set these to avoid noise
      secondBeat = true;                     // when we get the heartbeat back
     }
  
  if (MinCounter<60){
    
     if (SecCounter<500){
     
     SecCounter += 1;//count ms until 1 s
   }
     else{
     MinCounter += 1;//count s until 1 min
    // Serial.println( MinCounter);
     SecCounter = 0;//reload;
   }
  }
   else{
   //MLed = true;//FLAG for minite led
  // beatsCounter = 0;//reset beats
  Serial.print("The beat is:" );
    Serial.println(beatsCounter); 
    digitalWrite(MledPin, HIGH);
    delay(1000);
     digitalWrite(MledPin, LOW);
    beatsCounter = 0;//reset beats
   MinCounter = 0;//reload
   }
  
  sei();                                     // enable interrupts when youre done!
}// end isr







