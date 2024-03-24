#include <errctl.h>
#include <stdlib.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

void err_sys(const char * msg)
{
    perror(msg);
    exit(EXIT_FAILURE);
}

void err_quit(const char * msg)
{
    fputs(msg, stderr);
    fputc('\n', stderr);
    exit(EXIT_FAILURE);
}

void logger(LOGLEVEL level, const char* log, FILE* file)
{
    static const char *log_levels[] = { "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL", "UNDEFINED"};
    
	time_t current_time;
    time(&current_time);

    char* timestamp = asctime(localtime(&current_time));
    timestamp[24] = '\0';
	
	fprintf(file, "[%s][%s]\t%s\n\n", timestamp, log_levels[level], log);
}

#ifdef __cplusplus
}
#endif