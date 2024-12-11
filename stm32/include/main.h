#ifndef _MAIN_H
#define _MAIN_H

#include <stdio.h>
#include <Arduino.h>
#include "bq25730-ststm32.h"

// Choose to use Serial.print() for debugging or not
#if DEBUG == 1
#define debug(x)    Serial.print(x)
#define debugln(x)  Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#ifdef __cplusplus
extern "C" {
#endif
/*
 * function prototypes which don't want cpp compiler to do name-mangling
 *
 *
 */
#ifdef __cplusplus
}
#endif
#endif
