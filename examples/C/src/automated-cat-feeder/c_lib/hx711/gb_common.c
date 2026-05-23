//=============================================================================
//
//
// Gertboard Common code
//
// This file is part of the gertboard test suite
//
//
// Copyright (C) Gert Jan van Loo & Myra VanInwegen 2012
// No rights reserved
// You may treat this program as if it was in the public domain
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//
// Notes:
// 1/ In some Linux systems (e.g. Debian) the UART is used by Linux.
//    So for now do not demo the UART.
// 2/ At the moment (16-March-2012) there is no Linux driver for
//    the audio yet so the PWM is free.
//    This is likely to change and in that case the Linux
//    audio/PWM driver must be disabled.
//
// This file contains code use by all the test programs for individual 
// capabilities.

#include "gb_common.h"

#define GPIO_BASE 0x00000000   // gpiomem maps GPIO at offset 0

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <unistd.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
char *clk_mem_orig, *clk_mem, *clk_map;
char *gpio_map;
char *pwm_mem_orig, *pwm_mem, *pwm_map;
char *spi0_mem_orig, *spi0_mem, *spi0_map;
char *uart_mem_orig, *uart_mem, *uart_map;

// I/O access
volatile unsigned *gpio;
volatile unsigned *pwm;
volatile unsigned *clk;
volatile unsigned *spi0;
volatile unsigned *uart;


//
//  GPIO
//

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET0   *(gpio+7)  // Set GPIO high bits 0-31
#define GPIO_SET1   *(gpio+8)  // Set GPIO high bits 32-53

#define GPIO_CLR0   *(gpio+10) // Set GPIO low bits 0-31
#define GPIO_CLR1   *(gpio+11) // Set GPIO low bits 32-53
#define GPIO_PULL   *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock


//
//  UART 0
//

#define UART0_BAUD_HI *(uart+9)
#define UART0_BAUD_LO *(uart+10)


void setup_io();
void restore_io();

//
// This is a software loop to wait
// a short while.
//
void short_wait()
{ int w;
  for (w=0; w<100; w++)
  { w++;
    w--;
  }
} // short_wait


//
// Simple SW wait loop
//
void long_wait(int v)
{ int w;
  while (v--)
    for (w=-800000; w<800000; w++)
    { w++;
      w--;
    }
} // long_wait


//
// Set up memory regions to access the peripherals.
// This is a bit of 'magic' which you should not touch.
// It it also the part of the code which makes that
// you have to use 'sudo' to run this program.
//
void setup_io()
{  
    if ((mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC)) < 0) {
        perror("open /dev/gpiomem");
        exit(-1);
    }

    gpio_map = mmap(
        NULL,
        BLOCK_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        0
    );

    if (gpio_map == MAP_FAILED) {
        perror("gpio mmap failed");
        exit(-1);
    }

    gpio = (volatile unsigned *)gpio_map;
} // setup_io

//
// Undo what we did above
//
void restore_io()
{
  munmap(gpio_map, BLOCK_SIZE);
  close(mem_fd);
} // restore_io

// simple routine to convert the last several bits of an integer to a string 
// showing its binary value
// nbits is the number of bits in i to look at
// i is integer we want to show as a binary number
// we only look at the nbits least significant bits of i and we assume that 
// s is at least nbits+1 characters long
void make_binary_string(int nbits, int i, char *s)
{ char *p;
  int bit;

  p = s;
  for (bit = 1 << (nbits-1); bit > 0; bit = bit >> 1, p++)
    *p = (i & bit) ? '1' : '0';
  *p = '\0';
}
