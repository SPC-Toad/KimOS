#include "../types.h"

#ifndef STDIO_H
#define STDIO_H

size_t strlen(const char* str);

void strncpy(char *dest, const char *src, size_t n);

int strcmp(char *a, char *b);

void itoa (char *buf, int base, int d);

#endif