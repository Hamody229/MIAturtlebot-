/* 
 * File:   common.h
 * Author: yoyos
 *
 * Created on September 1, 2024, 11:21 AM
 */

#ifndef COMMON_H
#define	COMMON_H

/** clears the bit number BIT_INDEX of NUM
 * 
 * for example BIT_CLEAR(var_name, 2) will clear
 * the 2nd least significant bit of var_name
 * 
 * clearing means: set to zero
 **/
#define BIT_CLEAR(NUM, BIT_INDEX) (NUM &= ~(1 << (BIT_INDEX)))

/** sets the bit number BIT_INDEX of NUM
 * 
 * for example BIT_SET(var_name, 2) will set
 * the 2nd least significant bit of var_name
 * 
 * setting means: set to 1
 **/
#define BIT_SET(NUM, BIT_INDEX) (NUM |= 1 << (BIT_INDEX))

/** returns the bit number BIT_INDEX of NUM
 * 
 * for example BIT_READ(var_name, 2) will return
 * the 2nd least significant bit of var_name
 **/
#define BIT_READ(NUM, BIT_INDEX) (NUM & (1 << (BIT_INDEX)) != 0)

#endif	/* COMMON_H */
