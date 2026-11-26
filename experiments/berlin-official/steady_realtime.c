// Evaluation-only wall clock for a WSL host with periodic backward time steps.
// All trial processes inherit one realtime-minus-monotonic offset.
#define _GNU_SOURCE
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/time.h>
static long long offset;
__attribute__((constructor)) static void init_clock(void) {
    const char *s=getenv("TRIAL_REALTIME_OFFSET_NS");
    if(s) offset=strtoll(s,0,10);
}
int clock_gettime(clockid_t id, struct timespec *ts) {
    if(id!=CLOCK_REALTIME || !offset)
        return syscall(SYS_clock_gettime,id,ts);
    int err=syscall(SYS_clock_gettime,CLOCK_MONOTONIC,ts);
    if(err) return err;
    long long n=ts->tv_nsec+offset;
    ts->tv_sec+=n/1000000000LL;
    ts->tv_nsec=n%1000000000LL;
    return 0;
}
int gettimeofday(struct timeval *tv,void *tz) {
    if(!offset) return syscall(SYS_gettimeofday,tv,tz);
    struct timespec ts;
    if(clock_gettime(CLOCK_REALTIME,&ts)) return -1;
    tv->tv_sec=ts.tv_sec; tv->tv_usec=ts.tv_nsec/1000;
    return 0;
}
