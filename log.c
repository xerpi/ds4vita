#include "log.h"
#include <psp2kern/io/fcntl.h>

extern int ksceIoMkdir(const char *, int);

#ifndef RELEASE
static unsigned int log_buf_ptr = 0;
static char log_buf[16 * 1024];
#endif

void log_reset()
{
#ifndef RELEASE
	SceUID fd = ksceIoOpen(LOG_FILE,
		SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 6);
	if (fd < 0)
		return;

	ksceIoClose(fd);

	memset(log_buf, 0, sizeof(log_buf));
#endif
}

void log_write(const char *buffer, size_t length)
{
#ifndef RELEASE
	if ((log_buf_ptr + length) >= sizeof(log_buf))
		return;

	memcpy(log_buf + log_buf_ptr, buffer, length);

	log_buf_ptr = log_buf_ptr + length;
#endif
}

void log_flush()
{
#ifndef RELEASE
	ksceIoMkdir(LOG_PATH, 6);

	SceUID fd = ksceIoOpen(LOG_FILE,
		SCE_O_WRONLY | SCE_O_CREAT | SCE_O_APPEND, 6);
	if (fd < 0)
		return;

	ksceIoWrite(fd, log_buf, strlen(log_buf));
	ksceIoClose(fd);
#endif
}
