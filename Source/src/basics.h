#ifndef __BASICS_H__
#define __BASICS_H__

typedef int		bool;
#define true	1
#define false	0

typedef unsigned char BYTE;

#define TEST_BIT(var, bit)		((((var) & (0x01 << (bit))) != 0) ? true : false)

#define MAX(v1, v2)				(((v1) >= (v2)) ? (v1) : (v2))
#define MIN(v1, v2)				(((v1) <= (v2)) ? (v1) : (v2))

#define null					0
#define fnull					((void(*)(void))0)

#endif	// __BASICS_H__
