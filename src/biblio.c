#include <avr/io.h>
#include <util/delay.h>

void initIO (char c, int bit, uint8_t enable) { //1:output 0:input
    if (((c=='B') || (c=='b')) && enable) {
        DDRB |= (1<<bit);
    }
    else if (((c=='D') || (c=='d')) && enable) {
        DDRD |= (1<<bit);
    }
    else if (((c=='C') || (c=='c')) && enable) {
        DDRC |= (1<<bit);
    }
    else if(((c=='B') || (c=='b')) && (!enable)) {
        DDRB &= ~(1<<bit);
    }
    else if(((c=='D') || (c=='d')) && (!enable)) {
        DDRD &= ~(1<<bit);
    }
    else if(((c=='C') || (c=='c')) && (!enable)) {
        DDRC &= ~(1<<bit);
    }
}

void setIO (char c, int bit) { //LIGAR
    if (c=='B' || c=='b') {
        PORTB |= (1<<bit);
    } 
    else if (c=='D' || c=='d'){
        PORTD |= (1<<bit);
    }
    else if (c=='C' || c=='c') {
        PORTC |= (1<<bit);
    }
}

void resetIO (char c, int bit) { //DESLIGAR
    if (c=='B' || c=='b') {
        PORTB &= ~(1<<bit);
    } 
    else if (c=='D' || c=='d') {
        PORTD &= ~(1<<bit);
    }
    else if (c=='C' || c=='c') {
        PORTC &= ~(1<<bit);
    }
}

uint8_t readIO (char c, int bit) { //LER INPUTS
    if (c=='B' || c=='b') {
        return (PINB & (1<<bit));
    }
    else if (c=='D' || c=='d') {
        return (PIND & (1<<bit));
    }
    else if (c=='C' || c=='c') {
        return (PINC & (1<<bit));
    }
    else return 0;
}

uint8_t get_button (char c, int bit) { //Ler Botões, por terem lógica negativa
    return !readIO(c,bit);
}