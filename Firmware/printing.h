#ifdef PRINTING_H__
#define PRINTING_H__

#include <stdio.h>
#define RTT_PRINTF(...) \
do { \
     char str[64];\
     sprintf(str, __VA_ARGS__);\
     SEGGER_RTT_WriteString(0, str);\
 } while(0)

 #define printf RTT_PRINTF
 
 
 
 
 
 
 
 
#endif // PRINTING_H__
