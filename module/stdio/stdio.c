#include "stdio.h"
/*
    returns the number of the character in string
    Input: str
    Ouput: int
*/
size_t strlen(const char* str)
{
  size_t ret = 0;
  while ( str[ret] != 0 )
    ret++;
  return ret;
}

/*
    Copys string from dest to src by n number.
    Input: destination, source, # of characters to copy
    Ouput: none
*/
void strncpy(char *dest, const char *src, size_t n) 
{
  int i = 0;

  // Copy characters from src to dest up to 14 bytes
  while (i < n && src[i] != '\0') {
    dest[i] = src[i];
    i++;
  }

  // Fill the rest with null terminators to ensure proper padding
  while (i < n) {
    dest[i] = '\0';
    i++;
  }
}

/* 
    Compare two strings.
    Input: str1, str2
    Output: 0 or 1
*/
int strcmp(char *a, char *b) 
{
    int i = 0;
    while (a[i] != '\0' || b[i] != '\0') {
        if (a[i] != b[i]) return 1;  // Mismatch
        i++;
    }
    return 0;  // Strings are equal
}

/* Convert the integer D to a string and save the string in BUF. If
   BASE is equal to 'd', interpret that D is decimal, and if BASE is
   equal to 'x', interpret that D is hexadecimal. */
void itoa (char *buf, int base, int d)
{
  char *p = buf;
  char *p1, *p2;
  unsigned long ud = d;
  int divisor = 10;
     
  /* If %d is specified and D is minus, put `-' in the head. */
  if (base == 'd' && d < 0)
    {
      *p++ = '-';
      buf++;
      ud = -d;
    }
  else if (base == 'x')
    divisor = 16;
     
  /* Divide UD by DIVISOR until UD == 0. */
  do
    {
      int remainder = ud % divisor;
     
      *p++ = (remainder < 10) ? remainder + '0' : remainder + 'a' - 10;
    }
  while (ud /= divisor);
     
  /* Terminate BUF. */
  *p = 0;
     
  /* Reverse BUF. */
  p1 = buf;
  p2 = p - 1;
  while (p1 < p2)
    {
      char tmp = *p1;
      *p1 = *p2;
      *p2 = tmp;
      p1++;
      p2--;
    }
}