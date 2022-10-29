/*
  SSD1306_minimal.cpp - SSD1306 OLED Driver Library

  Copyright (c) 2012, Adafruit Industries. All rights reserved.
  Copyright (c) 2012 GOF Electronics Co. Ltd ( http://www.geekonfire.com )
  Copyright (c) 2015 CoPiino Electronics All right reserved.
  Copyright (c) 2016 Kirk Northrop (github.com/kirknorthrop)

  Original Author: Limor Fried/Ladyada Adafruit Industries
  Modified by: Jimbo.we (www.geekonfire.com)
  Modified by: CoPiino Electronics (http://copiino.cc)

      CoPiino Electronics invests time and resources providing this open source code,
      please support CoPiino Electronics and open-source hardware by purchasing
      products from CoPiino Electronics!

      What is it?
        This library is derived from OLED library, only for SSD1306 in I2C Mode.
        As the original library only supports Frame Buffered mode which requires to have
        at least 1024bytes of free RAM for a 128x64px display it is too big for smaller devices.

        So this a SSD1306 library that works great with ATTiny85 devices :)

  Modified by: Kirk Northrop (github.com/kirknorthrop)

  It is a free software; you can redistribute it and/or modify it
  under the terms of BSD license, check LICENSE for more information.
  All text above must be included in any redistribution.
*/

#include "SSD1306_minimal.h"
#include <Wire.h>
#include "minimal_font.h"

unsigned char SSD1306_Mini::getFlash( const unsigned char * mem, unsigned int idx  ){
  unsigned char data= pgm_read_byte( &(mem[idx]) );
  return data;

}

void SSD1306_Mini::sendCommand(unsigned char command)
{
  Wire.begin();                       //initialize I2C
  Wire.beginTransmission(_i2cAddr); // begin I2C communication

  Wire.write(OLED_Command_Mode);       // Set OLED Command mode
  Wire.write(command);

  Wire.endTransmission();             // End I2C communication
}

void SSD1306_Mini::sendData(unsigned char Data)
{
  Wire.begin();                    //initialize I2C
  Wire.beginTransmission(_i2cAddr); // begin I2C transmission
  Wire.write(OLED_Data_Mode);            // data mode
  Wire.write(Data);
  Wire.endTransmission();                    // stop I2C transmission
}

void SSD1306_Mini::init()
{

 Wire.begin();

 delay(5);  //wait for OLED hardware init
 // constructor(128, 64);
 //_i2cAddr = address;

 sendCommand(OLED_Display_Off_Cmd);    /*display off*/

 sendCommand(Set_Multiplex_Ratio_Cmd);    /*multiplex ratio*/
 sendCommand(0x3F);    /*duty = 1/64*/

 sendCommand(Set_Display_Offset_Cmd);    /*set display offset*/
 sendCommand(0x00);


 sendCommand(Set_Memory_Addressing_Mode_Cmd);  //set addressing mode
 sendCommand(HORIZONTAL_MODE);       //set horizontal addressing mode

 sendCommand(0xB0);       //set page address
 sendCommand(0x00);   //set column lower address
 sendCommand(0x10);   //set column higher address



 sendCommand(0x40);    /*set display starconstructort line*/

 sendCommand(Set_Contrast_Cmd);    /*contract control*/
 sendCommand(0xcf);    /*128*/

 sendCommand(Segment_Remap_Cmd);    /*set segment remap*/

 sendCommand(COM_Output_Remap_Scan_Cmd);    /*Com scan direction*/

 sendCommand(OLED_Normal_Display_Cmd);    /*normal / reverse*/

 sendCommand(Set_Display_Clock_Divide_Ratio_Cmd);    /*set osc division*/
 sendCommand(0x80);

 sendCommand(Set_Precharge_Period_Cmd);    /*set pre-charge period*/
 sendCommand(0xf1);

 sendCommand(Set_COM_Pins_Hardware_Config_Cmd);    /*set COM pins*/
 sendCommand(0x12);

 sendCommand(Set_VCOMH_Deselect_Level_Cmd);    /*set vcomh*/
 sendCommand(0x30);

 sendCommand(Deactivate_Scroll_Cmd);

 sendCommand(Charge_Pump_Setting_Cmd);    /*set charge pump enable*/
 sendCommand(Charge_Pump_Enable_Cmd);

 sendCommand(OLED_Display_On_Cmd);    /*display ON*/
}

void SSD1306_Mini::clipArea(unsigned char col, unsigned char row, unsigned char w, unsigned char h){

  Wire.begin();                    //initialize I2C
  Wire.beginTransmission(_i2cAddr); // begin I2C transmission
  Wire.write(OLED_Command_Mode);            // data mode
  Wire.write(Set_Column_Address_Cmd);
  Wire.write(0);

  Wire.write(col);
  Wire.write(col+w-1);

  Wire.endTransmission();                    // stop I2C transmission

  Wire.begin();                    //initialize I2C
  Wire.beginTransmission(_i2cAddr); // begin I2C transmission
  Wire.write(OLED_Command_Mode);            // data mode
  Wire.write(Set_Page_Address_Cmd);
  Wire.write(0);

  Wire.write(row);
  Wire.write(row+h-1);

  Wire.endTransmission();                    // stop I2C transmission

}

void SSD1306_Mini::cursorTo(unsigned char col, unsigned char row){
  clipArea(col, row, 128-col, 8-row);
}

void SSD1306_Mini::startScreen(){
  sendCommand(0x00 | 0x0);  // low col = 0
  sendCommand(0x10 | 0x0);  // hi col = 0
  sendCommand(0x40 | 0x0); // line #0
}

void SSD1306_Mini::clear() {

  sendCommand(0x00 | 0x0);  // low col = 0
  sendCommand(0x10 | 0x0);  // hi col = 0
  sendCommand(0x40 | 0x0); // line #0

  clipArea(0,0,128,8);

  for (uint16_t i=0; i<=((128*64/8)/16); i++)
  {
    // send a bunch of data in one xmission
    Wire.beginTransmission(_i2cAddr);
    Wire.write(OLED_Data_Mode);            // data mode
    for (uint8_t k=0;k<16;k++){
      Wire.write( 0 );
    }
    Wire.endTransmission();
  }
}


void SSD1306_Mini::displayX(int off) {
  sendCommand(0x00 | 0x0);  // low col = 0
  sendCommand(0x10 | 0x0);  // hi col = 0
  sendCommand(0x40 | 0x0); // line #0

  for (uint16_t i=0; i<=((128*64/8)/16); i++)
  {
    // send a bunch of data in one xmission
    Wire.beginTransmission(_i2cAddr);
    Wire.write(OLED_Data_Mode);            // data mode
    for (uint8_t k=0;k<16;k++){
      Wire.write((uint8_t) i*16 + k + off);
    }
    Wire.endTransmission();
  }
}



void SSD1306_Mini::printChar( char ch ){

  char data[5];
  unsigned char i = ch;

  data[0] = getFlash(BasicFont, i*5 );
  data[1] = getFlash(BasicFont, i*5 + 1);
  data[2] = getFlash(BasicFont, i*5 + 2);
  data[3] = getFlash(BasicFont, i*5 + 3);
  data[4] = getFlash(BasicFont, i*5 + 4);

  Wire.beginTransmission(_i2cAddr);
  Wire.write(OLED_Data_Mode);            // data mode

  Wire.write( 0x00 );
  Wire.write( data[0] );
  Wire.write( data[1] );
  Wire.write( data[2] );
  Wire.write( data[3] );
  Wire.write( data[4] );
  Wire.write( 0x00 );

  Wire.endTransmission();

}


void SSD1306_Mini::printString(const char * pText ){
  unsigned char i;
  unsigned char len = strlen( pText );

  for (i = 0; i < len; i++){
    printChar( pText[i] );
    // unsigned char chr = pText[i];
    // u8chr_t u8chr = *(uint32_t*)&pText[i];
    // if (u8chrisvalid(u8chr)) {
    //   printChar(specialCharTable(u8chr));
    // if (chr <= 0x20 || chr >= 0x7e) {
    //  printChar( pText[i] );
    // } else {
    //   printChar(0xa8); // ¿
    // }
  }
}

void SSD1306_Mini::drawImage( const unsigned char * img, unsigned char col, unsigned char row, unsigned char w, unsigned char h ){
  unsigned int i,k,data;

  clipArea( col, row, w, h);

  for (i=0;i< (w*h);i++){

      data= getFlash( img, i);

      Wire.beginTransmission(_i2cAddr);
      Wire.write(OLED_Data_Mode); // data mode
      Wire.write((uint8_t) data );
      Wire.endTransmission();

  }

}
