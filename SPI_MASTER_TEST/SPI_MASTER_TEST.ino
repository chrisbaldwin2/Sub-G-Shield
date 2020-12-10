#include <SPI.h>

#define SLAVE_READY 9

void setup (void) {
   Serial.begin(9600); //set baud rate to 115200 for usart
   digitalWrite(SS, HIGH); // disable Slave Select
   SPI.begin ();
   SPI.setClockDivider(SPI_CLOCK_DIV16);//divide the clock by 16
   SPI.setDataMode(SPI_MODE2);
   pinMode(SLAVE_READY, INPUT);
}

void loop (void) {
   char c;
   char rec[20];
   int i = 0;
   int len = 9;
   digitalWrite(SS, LOW); // enable Slave Select
   // send test string
   for (const char * p = "Hello, world!\r" ; c = *p; p++, i++) {
      rec[i] = SPI.transfer (c);
      Serial.print(c);
   }
   digitalWrite(SS, HIGH); // disable Slave Select
   Serial.print('\n');
   for (i = 0; i < len; i++){
    Serial.print(rec[i]);
   }
   Serial.println('\r');
   delay(2000);
}
