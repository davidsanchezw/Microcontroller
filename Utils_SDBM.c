#include "Utils_SDBM.h"
#define MAX_LCD 6

short pow10b(short p);

void espera(int num_ciclos){
int i;
for(i = 0; i < num_ciclos; i ++);
}

short pow10b(short p){
  short i, ret;
  ret = 1;
  for(i = 0; i < p; i ++){
    ret = ret * 10;
  }
  return ret;
}

void Bin2Ascii(short mynumber, unsigned char *mystring){
	short i, last, lastpow;
  if(mynumber < 0){
    mystring[0] = '-';
    mynumber = mynumber * -1;
  }else{
    mystring[0] = ' ';
  }
  last = 0;
  lastpow = 0;
  for(i = 1; i < MAX_LCD; i++){
    mynumber -= last * lastpow;
    lastpow = pow10b(MAX_LCD - 1 - i);
    last = mynumber / lastpow;
    mystring[i] = last + '0';
  }
}

// Devuelve 1 si se pulsa una tecla
unsigned char flanco_bajo(unsigned char m[]){
	if (m[2] == 1 && m[1] == 0 && m[0] == 0) return 1;
	else return 0;
}

// Devuelve 1 si se deja de pulsar una tecla
unsigned char flanco_alto(unsigned char m[]){
	if (m[2] == 0 && m[1] == 1 && m[0] == 1) return 1;
	else return 0;
}
