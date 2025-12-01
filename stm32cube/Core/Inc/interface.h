/*
 * interface.h
 *
 *  Created on: Apr 10, 2022
 *      Author: UMK
 */

#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

#define CMD_SEP ';'

typedef struct{
    void* p;
    char* type;
} pointer;

typedef struct{
    int is;
} ison;

typedef struct{
    int tabsize;
    int tabcount;
    int tabpos;
    double* ptab[2];
}mestab;

typedef struct {
    double min;
    double max;
    double val;
    char* cmdset;
    ison tabon;
    mestab mes;
} value;

typedef struct {
	value raw;
    value volt;
    value avr;
    value coron;
    value corfactor;
}sadcchannel;

typedef struct{
    sadcchannel ch1;
    sadcchannel ch2;
} sadc;

typedef struct {
	value raw;
    value volt;
}sdacchannel;

typedef struct{
	sdacchannel ch1;
	sdacchannel ch2;
} sdac;

typedef struct{
    value on;
    value mcnt;
    value allow;
} slock;

typedef struct{
    value on;
    value f;
    value fset;
    value ok;
    value okdif;
    slock lock;
    value i;
    value maxdif;
    value out;
    value mcnt;
    value ch; // wlm channel
} swlm;

typedef struct{
    value on;
    value allow;
    value last_on;
    value vset;
    value P;
    value I;
    value sign;
    value err;
    value aerr;
    value out;
} sunl;

typedef struct{
    value on;
    value tresh; // transmission threshold
    value cnt; // counter - locked time
    value mcnt; // min cnt to tread that locked
    value locked; // locked flag
} srlc;

typedef struct{
    value on;
    value ampl; // scan amplitude
    value step; // scan step
    value dir; // scan direction
    value out; // scan output (current value)
} sscan;

typedef struct {
    double version;
    value save;
    value load;
    value ver;
    sadc adc;
    sdac dac;
    value out1;
    value out2;
    value rout1;
    value in1;
    value in2;
    value in3;
    value send;
    swlm wlm;
    sunl unl;
    srlc rlc;
    sscan scan;
    value sw_on;
    value last_sw_on;
    value sw_allow;
    value work;
} parameters;

pointer getPointer(pointer,char * );
void initInterface(void);
void setParam(value*, double);
double atofmy(char *str);


#endif /* INC_INTERFACE_H_ */
