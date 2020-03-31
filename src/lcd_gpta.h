//Library created by Guilherme Pinheiro (up201703867), day 1/12/2019

#define LED 5
#define ClearBit(x,y) x &= ~_BV(y) 
#define SetBit(x,y) x |= _BV(y)
#define LCD_RS 0
#define LCD_E  1
#define DAT4   2
#define DAT5   3
#define DAT6   4
#define DAT7   5
#define CLEARDISPLAY 0x01
#define SETCURSOR    0x80
#define NUMCHARS 64

#include <avr/io.h> 
#include <util/delay.h> 
#include <string.h> 
#include <stdlib.h>

typedef uint8_t byte;
typedef int8_t sbyte;

void SetupPorts();

void msDelay(int delay);
void FlashLED();
void PulseEnableLine ();
void SendNibble(byte data);
void SendByte (byte data);

void LCD_Cmd (byte cmd);
void LCD_Char (byte ch);
void LCD_Init();
void LCD_Clear();
void LCD_Home();
void LCD_Goto(byte x, byte y);
void LCD_Line(byte row);
void LCD_Message(const char *text);
void LCD_Hex(int data);
void LCD_Integer(int data);

void UpdateCursor (byte count);
char GetNextChar(char ch);
void FillScreen ();

//Library created by Guilherme Pinheiro (up201703867), day 1/12/2019

//Code from: http://w8bh.net/avr/AvrLCD1.pdf