// DS2450_1 (Arduino)
//
// Illustrates an interface with a Dallas DS2450 Quad 16-bit A/D
//
// Configures for VDD = 5V operation by writing &H40 to memory location &H001c.
//
// Configures all four channels for 12 bit, 5.12 full scale by writing to
// locations 0x0008 and 0009 for Channel A, 0x000a and 000b for channel B, etc.
//
// Continually loops, setting mask to 1111 (all channels) with a preset of 0.
// Reads from locations 0x0000 and 0001 for Channel A, 0x0002 and 0003 for
// Channel B, etc.
//
// Displays the A/D value for each channel.
//
// Note that the 16-bit CRC is not implemented.
//
//
//                  +5
//                   |
//                   4.7K
//                   |
// Arduino (term 8) ------------------------ DS2450 (term 3)
//
// Tx ---------------------------- To Serial LCD (LCD #117)
//
// Peter H Anderson, Baltimore, MD, May 15, '07




void Configure_2450(int _1W_Pin);
int FetchCRCBytes(int _1W_Pin);

void OneWireReset(int _1W_Pin);
void OneWireOutByte(int _1W_Pin, byte d, byte strong);
byte OneWireInByte(int _1W_Pin);

void setup()
{
  int n, _1W_Pin;

  _1W_Pin = 8;
  digitalWrite(_1W_Pin, LOW);
  pinMode(_1W_Pin, INPUT);      // sets the digital pin as input (logic 1)

  Serial.begin(9600);
  delay(100);
  Serial.print("?B40"); // set backlight intensity
  delay(100);
  Serial.print("?f"); // clear LCD
  delay(100);
}

void loop()
{
   int  _1W_Pin, CRCRead, LowByte, HighByte, ADVal;

   _1W_Pin = 8;

   Configure_2450(_1W_Pin);

   while(1)
   {
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

      LowByte = OneWireInByte(_1W_Pin);
      HighByte = OneWireInByte(_1W_Pin);
      ADVal = HighByte*16 + LowByte/16;

      Serial.print("?f");
      delay(25);
      Serial.print("Channel A ");
      Serial.print(ADVal);

      LowByte = OneWireInByte(_1W_Pin);
      HighByte = OneWireInByte(_1W_Pin);
      ADVal = HighByte*16 + LowByte/16;

      Serial.print("?n");
      Serial.print("Channel B ");
      Serial.print(ADVal);

      LowByte = OneWireInByte(_1W_Pin);
      HighByte = OneWireInByte(_1W_Pin);
      ADVal = HighByte*16 + LowByte/16;

      Serial.print("?n");
      Serial.print("Channel C ");
      Serial.print(ADVal);

      LowByte = OneWireInByte(_1W_Pin);
      HighByte = OneWireInByte(_1W_Pin);
      ADVal = HighByte*16 + LowByte/16;

      Serial.print("?n");
      Serial.print("Channel D ");
      Serial.print(ADVal);

      //Volts = CSng(ADVal) / 4096.0 * 5.12

      delay(5000);  // as appropriate
   }
}

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

