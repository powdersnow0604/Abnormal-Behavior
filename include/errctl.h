#ifndef __ERRCTL_H__
#define __ERRCTL_H__

#include <stdio.h>


#ifdef __cplusplus
extern "C" {
#endif

//exit(failure) with print err to stderr 
void err_quit(const char * msg);

//exit(failure) with print current error message
void err_sys(const char * msg);

//print message with LOGLEVEL, logged time
typedef enum LOGLEVEL_ {
	DEBUG,
	INFO,
	WARNING,
	ERROR,
	CRITICAL
}LOGLEVEL;

void logger(LOGLEVEL level, const char* log, FILE* file); 

#ifdef __cplusplus
}
#endif

#endif