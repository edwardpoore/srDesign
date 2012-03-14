
/*
Xstream Team Sr. Design
Written by Edward Poore
Spring 2012

SD card information:
 ** CS - pin 8
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13

*/
#include <SD.h> //sd interface
#include <Arduino.h> //needed for time
#include <Time.h> //timekeeping
#include <TimeAlarms.h>
#include <Wire.h> //I2C communication
#include <DS1307RTC.h> //basic DS1307 library

int index;
//AlarmId measure; //measurement object
byte inByte; //call/reponse variable

void setup()
{
  Serial.begin(9600); //start serial comm
  pinMode(10, OUTPUT); //pin 10 needs to be set as an output for the SD library
  pinMode(2, OUTPUT); //used to drive the temperature sensor


  //Look for a SD card
  if (!SD.begin(8)) 
  {
    Serial.println("No SD card detected!");
    return;
  }

  //Get the current date/time from the clock module
  //Set the time library to automatically sync the time clock from the DS1307
  //DS1307 SDA is Ain4, SCL is Ain5
  setSyncProvider(RTC.get);
  if(timeStatus()!= timeSet) 
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");

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
          Serial.println("Invalid Command. Press h for help.");
          break; //do nothing
      } //switch(inByte)
    } //if serial.available  
  
  if(second()%30 == 0) //take measurements every 30 seconds
  {
    takeMeas();
    Alarm.delay(1100); //wait one second so it will only trigger once
  }

} //loop
  
void takeMeas(void)
{
  //Gather measurements from sensors
  float temperature = read_temp(); //collect temperature measurement
  
  //Gather other measurements
  /* 
  
    OTHER MEASUREMENT FUNCTION CALLS
    
  */
  
  //Build a string to represent the current time/date
  String timeAndDate = String(hour()) + ":" + String(minute()) + ":" + String(second()); //hh:mm:ss
  timeAndDate = timeAndDate + "|" + String(2) + "/" + String(25) + "/" + String(12); //hh:mm:ss|mm/dd/yyyy
  
  //add an entry to the file
  addEntry( index, temperature, 1.0, 1.0, 1.0, 1.0, 1.0, timeAndDate);
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
          Serial.println("Invalid Command. y for yes or n for no.");
          //do nothing
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

void addEntry( int entry, float temp, float disso, float turb, float flow, float pH, float ec, String timeDate)
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
      myFile.print(index);
      myFile.print(", ");
      myFile.print(temp);
      myFile.print(", ");
      myFile.print(disso);
      myFile.print(", ");
      myFile.print(turb);
      myFile.print(", ");
      myFile.print(flow);
      myFile.print(", ");   
      myFile.print(pH);
      myFile.print(", ");
      myFile.print(ec);
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
//red - pin 2 - consider moving to VDD on final design
float read_temp(void){                          //the read temperature function 
  float v_out;                                  //voltage output from temp sensor  
  float temp;         //the final temperature is stored here (this is only for code clarity) 

  digitalWrite(A0, LOW);                       //set pull-up on analog pin 0

  digitalWrite(2, HIGH);                       //set pin 2 high, this will turn on temp sensor

  delay(2);                               //wait 1 ms for temp to stabilize

  v_out = analogRead(0);                  //read the input pin
  digitalWrite(2, LOW);                  //set pin 2 low, this will turn off temp sensor 

  v_out*=.0048;    //convert ADC points to volts (we are using .0048 because this device is running at 5 volts)
  v_out*=1000;                                      //convert volts to millivolts  
  temp= 0.0512 * v_out -20.5128;   //the equation from millivolts to temperature   

  return   temp;     //send back the temp
} //read_temp
