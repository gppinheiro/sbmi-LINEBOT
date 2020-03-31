#include <avr/io.h>
#include <util/delay.h>

void initIO (char c, int bit, uint8_t enable);

void setIO (char c, int bit);

void resetIO (char c, int bit);

uint8_t readIO (char c, int bit);

uint8_t get_button (char c, int bit);