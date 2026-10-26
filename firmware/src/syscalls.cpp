/**
 * @file    syscalls.cpp
 * @brief   Minimal newlib syscalls: printf goes out USART1 as Log packets (remote.h), nothing else is supported.
 */
#include <errno.h>
#include <sys/stat.h>

#include "remote.h"

extern "C" {

int _write(int fd, char* buf, int len) {
    if (len > 0) remoteWrite(buf, (size_t)len);
    return len;
}

int _read(int fd, char* buf, int len) { return 0; }
int _close(int fd) { return -1; }
int _lseek(int fd, int off, int whence) { return 0; }
int _isatty(int fd) { return 1; }
int _fstat(int fd, struct stat* st) {
    st->st_mode = S_IFCHR;
    return 0;
}
int _getpid(void) { return 1; }
int _kill(int pid, int sig) {
    errno = EINVAL;
    return -1;
}

extern char _end;
void* _sbrk(int incr) {
    static char* heap = &_end;
    char* prev = heap;
    heap += incr;
    return prev;
}

}  // extern "C"
