
/*
Xstream Team Sr. Design
Written by Edward Poore
Spring 2012

SD card information:
 ** CS - pin 8
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13

 SDA -> A4
 SCL -> A5
 
 DI0, DI1 -> UART
 DI3 -> Flow Control for UART
 
 DI6 -> triple sensor 1
 DI7 -> triple sensor 2
 
 
 
*/
#include <SD.h> //sd interface
#include <Wire.h> //I2C communication
#include <RTClib.h>

RTC_DS1307 RTC; //RTC object
int index;
const int triple1 = 6; //triple sensor number 1
const int triple2 = 7; //triple sensor number 2
float triple[2]; //an array to handle data from triple sensors

void setup()
{
  Serial.begin(9600); //start serial comm
  pinMode(10, OUTPUT); //pin 10 needs to be set as an output for the SD library
  pinMode(2, INPUT); //CTS for the xbee if needed
  
  //SETUP FOR TRIPLE SENSOR ONE WIRE DATA LINES
  //Sensor 1
  digitalWrite(triple1, LOW); //initialize it to low
  pinMode(triple1, INPUT);      // sets the digital pin as input (logic 1)
  //Sensor 2
  digitalWrite(triple2, LOW); //initialize it to low
  pinMode(triple2, INPUT);      // sets the digital pin as input (logic 1)
  
  //Look for a SD card
  if (!SD.begin(8)) 
  {
    Serial.println("No SD card detected!");
    return;
  }

  //Get the current date/time from the clock module
  //DS1307 SDA is Ain4, SCL is Ain5
  Wire.begin();
  RTC.begin();
  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
  }
  DateTime now = RTC.now();
  DateTime time (now.unixtime()-2*86400+3*3600);

  //Generate initial file if none found
  if(!SD.exists("mydata.csv"))
  {
    eraseFile();
  }

  index = 1; //index to represent how many measurements have been made since last time power was lost. Set to 1 at startup

  Serial.println("Setup Complete.");
}

void loop()
{
  DateTime now = RTC.now();
  DateTime time (now.unixtime()-2*86400+3*3600);
  byte inByte; //call/reponse variable
  //Call/Response for serial  

    if (Serial.available() > 0)
    {
      inByte = Serial.read(); 
      switch (inByte) {
        case 'd':
            getFile(); //download the file to the host  
          break;
        case 'e':
            eraseFile(); //erase the file and start again
          break;
        case 'h':
            Serial.println("Press d to retrieve data from node. Press e to erase data");
          break;
        default:
          break; //do nothing
      } //switch(inByte)
    } //if serial.available  
  
  if(time.second()%30 == 0) //take measurements every 30 seconds
  {
    takeMeas();
    delay(1100); //wait one second so it will only trigger once
  }

} //loop
  
  
  
void takeMeas(void)
{
  //Gather measurements from sensors
  float temperature = read_temp(); //collect temperature measurement
  
  //triple sensors
  readTriple(triple1);
  float triple1Turb = triple[0];
  float triple1Flow = triple[1];

  readTriple(triple2);
  float triple2Turb = triple[0];
  float triple2Flow = triple[1];

  DateTime now = RTC.now();
  DateTime time (now.unixtime()-2*86400+3*3600);
  
  //Build a string to represent the current time/date
  String timeAndDate = String(time.hour()) + ":" + String(time.minute()) + ":" + String(time.second()); //hh:mm:ss
  timeAndDate = timeAndDate + "|" + String(time.month()) + "/" + String(time.day()) + "/" + String(time.year()); //hh:mm:ss|mm/dd/yyyy
  
  //add an entry to the file
  addEntry( index, temperature, triple1Turb, triple1Flow, triple2Turb, triple2Flow, timeAndDate);
  Serial.println("Measurements taken and added to file.");
  return;
}//takeMeasurements
   
  
/*
eraseFile()
Check if mydata.csv exists on the SD card.
Erase it if it does, then generate a new file.
If it doesn't, just generate the new file.
*/
  
void eraseFile()
{ 
  byte inByte; //call/reponse variable
  int eraseStuff = 0; //variable to kick out of the loop if erase is desired
  Serial.println("Are you sure? y or n?");
  while(!eraseStuff) //if erase stuff gets set to 1 (y is detected)
  {
    if (Serial.available() > 0) //wait for a byte on the serial line
    {
      inByte = Serial.read(); // put it in inByte
      switch (inByte) {
        case 'y': //if inByte = y
          eraseStuff = 1; //kick out of loop and continue  
          break;
        case 'n': //if inByte = n
          return; //dont erase stuff
          break;
        default: //else
          break;
      } //switch(inByte)
    } //if serial.available  
  }//while(1)
    
  //if the file exists, delete it
  if(SD.exists("mydata.csv"))
  {
    Serial.println("File found.");
    SD.remove("mydata.csv");
    Serial.println("File deleted.");
    File myFile = SD.open("mydata.csv", FILE_WRITE); //replace the file
    myFile.close();
    Serial.println("New file created.");
    index = 1; //set index to 1
  }  
  //else just create a blank file
  else
  {
    Serial.println("File not found.");
    Serial.println("Creating file...");
    File myFile = SD.open("mydata.csv", FILE_WRITE);
    myFile.close();
    Serial.println("File created.");
  }
 
 return;   
} //eraseFile()
  
/*
getFile()
Open mydata.csv and dump any information inside to the serial port.

*/
  
void getFile()
{
  
  //whitespace
  Serial.println("Measured Data:");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  
  if(SD.exists("mydata.csv")) //look for the file
  {
    File myFile = SD.open("mydata.csv", FILE_READ);
    //if the file was opened, write it all to the serial port
    if(myFile) //if the file was opened correctly
    {
      while(myFile.available()) //write...ALL THE BYTES!
      {
        Serial.write(myFile.read());
      }
      myFile.close();
    }
  }
  else
  {
    Serial.println("No file found.");
  }
   
 return; 
} //getFile

/*
addEntry()
Add an entry to the end of the file.
Construct the entry from the parameters passed
*/

void addEntry( int entry, float temp, float turb1, float flow1, float turb2, float flow2, String timeDate)
{
  
  //open file
  if(SD.exists("mydata.csv"))
  {
    File myFile = SD.open("mydata.csv", FILE_WRITE);
    //if the file was opened, write it all to the serial port
    if(myFile)
    {
      //add the string to the file
      //myFile.println(entryString);
      myFile.print(entry);
      myFile.print(", ");
      myFile.print(temp);
      myFile.print(", ");
      myFile.print(turb1);
      myFile.print(", ");
      myFile.print(flow1);
      myFile.print(", ");   
      myFile.print(turb2);
      myFile.print(", ");
      myFile.print(flow2);
      myFile.print(", ");
      myFile.println(timeDate);
      
      myFile.close();
      index = index + 1; //only increment the measurement value if a line is added
    }
  }  
  return;
} //addEntry


//Wiring
//white - A0
//black - gnd
//red - vdd
float read_temp(void){                          //the read temperature function 
  float v_out;                                  //voltage output from temp sensor  
  float temp;         //the final temperature is stored here (this is only for code clarity) 

  digitalWrite(A0, LOW);                       //set pull-up on analog pin 0

// writing and resetting d2 not needed when red is connected to vdd
//  digitalWrite(2, HIGH);                       //set pin 2 high, this will turn on temp sensor

  delay(2);                               //wait 1 ms for temp to stabilize

  v_out = analogRead(0);                  //read the input pin
//  digitalWrite(2, LOW);                  //set pin 2 low, this will turn off temp sensor 

  v_out*=.0048;    //convert ADC points to volts (we are using .0048 because this device is running at 5 volts)
  v_out*=1000;                                      //convert volts to millivolts  
  temp= 0.0512 * v_out -20.5128;   //the equation from millivolts to temperature   

  return   temp;     //send back the temp
} //read_temp

//TO READ FROM THE TRIPLE SENSORS


void readTriple(int pin)
{     
	int  _1W_Pin, CRCRead, LowByte, HighByte, AVal, BVal, CVal, DVal;

	_1W_Pin = pin; //select the pin to be used for OneWire as needed

	Configure_2450(_1W_Pin); //use before gathering measurements
	OneWireReset(_1W_Pin);
	OneWireOutByte(_1W_Pin, 0xcc, 0);
	OneWireOutByte(_1W_Pin, 0x3c, 0);  // convert
	OneWireOutByte(_1W_Pin, 0x0f, 0);  // all channels
	OneWireOutByte(_1W_Pin, 0x01, 0);  // preset to all zeros
	CRCRead = FetchCRCBytes(_1W_Pin);
	
	while(1) // wait for conversion to complete
    {
       if(OneWireInByte(_1W_Pin) == 0xff)
        {
			break;
        }
    }

	delay(1);
	
    OneWireReset(_1W_Pin);
    OneWireOutByte(_1W_Pin, 0xcc, 0);
    OneWireOutByte(_1W_Pin, 0xaa, 0);  // read memory
    OneWireOutByte(_1W_Pin, 0x00, 0);  // channel A
    OneWireOutByte(_1W_Pin, 0x00, 0);  // locations 0000 and 0001
	
	//Channel A
    LowByte = OneWireInByte(_1W_Pin);
    HighByte = OneWireInByte(_1W_Pin);
    AVal = HighByte*16 + LowByte/16;
	
	//Channel B
    LowByte = OneWireInByte(_1W_Pin);
    HighByte = OneWireInByte(_1W_Pin);
    BVal = HighByte*16 + LowByte/16;

	//Channel C
    LowByte = OneWireInByte(_1W_Pin);
    HighByte = OneWireInByte(_1W_Pin);
    CVal = HighByte*16 + LowByte/16;

	//Channel D
	LowByte = OneWireInByte(_1W_Pin);
    HighByte = OneWireInByte(_1W_Pin);
    DVal = HighByte*16 + LowByte/16;
  
  //place the values into the global variable for triple sensor readings
  //flow -> triple[1]
  //turbidity -> triple[0]

  int turb = (AVal - BVal); //get a differential measurement
  int flow = (CVal - DVal);
  flow = abs(flow); //we only care about the absolute difference
  turb = abs(turb);

  //return the values
  triple[0] = (float)turb / 4096.0 * 5.12;
  triple[1] = (float)flow / 4096.0 * 5.12;


  //Reference
  //Volts = CSng(ADVal) / 4096.0 * 5.12
}


//BELOW IS THE INTERFACE FOR THE ADC ON THE TRIPLE SENSOR(s)

//Function Prototypes
void Configure_2450(int _1W_Pin);
int FetchCRCBytes(int _1W_Pin);
void OneWireReset(int _1W_Pin);
void OneWireOutByte(int _1W_Pin, byte d, byte strong);
byte OneWireInByte(int _1W_Pin);

void Configure_2450(int _1W_Pin)
{
   int CRCRead, Dummy;

   OneWireReset(_1W_Pin);
   OneWireOutByte(_1W_Pin, 0xcc, 0);
   OneWireOutByte(_1W_Pin, 0x55, 0);
   OneWireOutByte(_1W_Pin, 0x1c, 0);  // write to 001c
   OneWireOutByte(_1W_Pin, 0x00, 0);
   OneWireOutByte(_1W_Pin, 0x40, 0);  // Vcc operation

   CRCRead = FetchCRCBytes(_1W_Pin);
   Dummy = OneWireInByte(_1W_Pin); // readback the data

   OneWireReset(_1W_Pin);
   OneWireOutByte(_1W_Pin, 0xcc, 0);
   OneWireOutByte(_1W_Pin, 0x55, 0);
   OneWireOutByte(_1W_Pin, 0x08, 0);  // write beginning at  0008
   OneWireOutByte(_1W_Pin, 0x00, 0);

   OneWireOutByte(_1W_Pin, 0x0b, 0);  // channel A - 12 bits

   CRCRead = FetchCRCBytes(_1W_Pin);
   Dummy = OneWireInByte(_1W_Pin); // readback the data

   OneWireOutByte(_1W_Pin, 0x01, 0);	// 5.12 VDC range
   CRCRead = FetchCRCBytes(_1W_Pin);
   Dummy = OneWireInByte(_1W_Pin);

   OneWireOutByte(_1W_Pin, 0x0b, 0);	 //set up Channel B for 12 bit
   CRCRead = FetchCRCBytes(_1W_Pin);
   Dummy = OneWireInByte(_1W_Pin);

   OneWireOutByte(_1W_Pin, 0x01, 0);	// 5.12 VDC range
   CRCRead = FetchCRCBytes(_1W_Pin);
   Dummy = OneWireInByte(_1W_Pin);

   OneWireOutByte(_1W_Pin, 0x0b, 0);	 //set up Channel C for 12 bit
   CRCRead = FetchCRCBytes(_1W_Pin);
   Dummy = OneWireInByte(_1W_Pin);

   OneWireOutByte(_1W_Pin, 0x01, 0);	// 5.12 VDC range
   CRCRead = FetchCRCBytes(_1W_Pin);
   Dummy = OneWireInByte(_1W_Pin);

   OneWireOutByte(_1W_Pin, 0x0b, 0);	 //set up Channel D for 12 bit
   CRCRead = FetchCRCBytes(_1W_Pin);
   Dummy = OneWireInByte(_1W_Pin);

   OneWireOutByte(_1W_Pin, 0x01, 0);	// 5.12 VDC range
   CRCRead = FetchCRCBytes(_1W_Pin);
   Dummy = OneWireInByte(_1W_Pin);
}

int FetchCRCBytes(int _1W_Pin)
{
   int x, y, CRC;

   x = OneWireInByte(_1W_Pin);
   y = OneWireInByte(_1W_Pin);
   CRC = CRC * y + x;
   return(CRC);
}

void OneWireReset(int _1W_Pin) // reset.  Should improve to act as a presence pulse
{
     digitalWrite(_1W_Pin, LOW);
     pinMode(_1W_Pin, OUTPUT); // bring low for 500 us
     delayMicroseconds(500);
     pinMode(_1W_Pin, INPUT);
     delayMicroseconds(500);
}

void OneWireOutByte(int _1W_Pin, byte d, byte strong) // output byte d (least sig bit first).
{
   byte n;

   for(n=8; n!=0; n--)
   {
      if ((d & 0x01) == 1)  // test least sig bit
      {
         digitalWrite(_1W_Pin, LOW);
         pinMode(_1W_Pin, OUTPUT);
         delayMicroseconds(5);
         pinMode(_1W_Pin, INPUT);
         delayMicroseconds(60);
      }
      else
      {
         digitalWrite(_1W_Pin, LOW);
         pinMode(_1W_Pin, OUTPUT);
         delayMicroseconds(60);
         pinMode(_1W_Pin, INPUT);
      }

      d=d>>1; // now the next bit is in the least sig bit position.
   }
   if(strong)
   {
       digitalWrite(_1W_Pin, HIGH); // One sec of strong +5 VDC
       pinMode(_1W_Pin, OUTPUT);
       delay(1000);
       pinMode(_1W_Pin, INPUT);
       digitalWrite(_1W_Pin, LOW);
   }
}

byte OneWireInByte(int _1W_Pin) // read byte, least sig byte first
{
    byte d, n, b;

    for (n=0; n<8; n++)
    {
        digitalWrite(_1W_Pin, LOW);
        pinMode(_1W_Pin, OUTPUT);
        delayMicroseconds(5);
        pinMode(_1W_Pin, INPUT);
        delayMicroseconds(5);
        b = digitalRead(_1W_Pin);
        delayMicroseconds(50);
        d = (d >> 1) | (b<<7); // shift d to right and insert b in most sig bit position
    }
    return(d);
}

