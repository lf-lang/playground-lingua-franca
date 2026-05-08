// Note: This script has been modified from original library for functionality with a memory barrier. 
// All changed functions sections are denoted by a "changed!" comment and a brief reason why.
#include "hx711.h"
#include <unistd.h>
#include <string.h>
#include <stdio.h>

// added memory barrier to not reorder/rewrite instructions going forward
#define MEM_BARRIER() __sync_synchronize()

int setupGPIO(HX711 *hx)
{
  setup_io();
  INP_GPIO(hx->data_pin);
  INP_GPIO(hx->clock_pin);
  OUT_GPIO(hx->clock_pin);
  // changed! 
  // clock must be low for hx711 to remain active
  // sleep to assure low
  setPinState(hx->clock_pin, 0);
  usleep(10);

  return 0;
}

void cleanGPIO(HX711 *hx)
{
  //unpull pins
  GPIO_PULL = 0;
  GPIO_PULLCLK0 = 1 << hx->data_pin;
  GPIO_PULL = 0;
  GPIO_PULLCLK0 = 0;
  restore_io();
}

//set higher priority for the process
int setPriority(int priority)
{
  struct sched_param sched;
  memset(&sched, 0, sizeof(sched));

  sched.sched_priority = priority;
  if (sched_setscheduler(0, SCHED_FIFO, &sched))
  {
    return 1;
  }
  return 0;
}


void reset(HX711 *hx)
{
  setPinState(hx->clock_pin, 1);
  usleep(100);
  setPinState(hx->clock_pin, 0);
  usleep(100);
}


void setGain(HX711 *hx)
{
  // changed!
  int pulses = 1;
  // simplified variable assignment
  if (hx->wanted_channel == 'B')
    pulses = 2;
  else if (hx->gain_channel_A == 64)
    pulses = 3;

  // usleep instead of continue
  for (int i = 0; i < pulses; i++)
  {
    setPinState(hx->clock_pin, 1);
    usleep(1);
    setPinState(hx->clock_pin, 0);
    usleep(1);
  }
}

// changed!
int getRawData(HX711 *hx)
{
  unsigned int bits = 0;
  // wait until low data
  while (getPinState(hx->data_pin))
  {
    // reduced polling sleep
    usleep(100);
  }

  // read 24 bits of data
  for (int i = 0; i < 24; i++)
  {
    setPinState(hx->clock_pin, 1);
    usleep(1);

    bits = (bits << 1) | (getPinState(hx->data_pin) ? 1 : 0);

    setPinState(hx->clock_pin, 0);
    usleep(1);
  }

  // set gain for next read
  setGain(hx);

  if (bits & 0x800000)
  {
    bits |= ~0xFFFFFF;
  }

  return (int)bits;
}


// changed!
// place mem barrier around direct gpio mem read to read state of curr physical pin, rather than cached val
bool getPinState(unsigned pin_number)
{
  if (pin_number > 31)
  {
    printf("getPinState - wrong pin number: %d\n", pin_number);
    return false;
  }

  MEM_BARRIER();

  int value = (*(gpio + 13) & (1 << pin_number));

  MEM_BARRIER();

  return value != 0;
}

// changed!
// place mem barrier around direct gpio mem write to assure 
//cpu does not reorder other var around hw write
int setPinState(unsigned pin_number, bool state)
{
  if (pin_number > 31)
  {
    printf("setPinState - wrong pin number: %d\n", pin_number);
    return 1;
  }

  MEM_BARRIER();

  if (state)
    *(gpio + 7) = (1 << pin_number);
  else
    *(gpio + 10) = (1 << pin_number);

  MEM_BARRIER();

  return 0;
}


int initHX711(HX711 *hx, unsigned char clock_pin, unsigned char data_pin)
{
  if (clock_pin > 31 || data_pin > 31)
  {
    printf("Invalid GPIO pins\n");
    return 1;
  }

  hx->clock_pin = clock_pin;
  hx->data_pin = data_pin;
  hx->gain_channel_A = 128;
  hx->current_channel = 'A';
  hx->wanted_channel = 'A';

  hx->offset_A_128 = 0;
  hx->offset_A_64 = 0;
  hx->offset_B = 0;

  hx->scale_ratio_A_128 = 0.0;
  hx->scale_ratio_A_64 = 0.0;
  hx->scale_ratio_B = 0.0;

  hx->filterPtr = NULL;

  return 0;
}

//changed!
// simplified- only zero scale if using default channel A @ 128 gain
int zeroScale(HX711 *hx)
{
  double result = getRawDataMean(hx, 50);

  if (hx->current_channel == 'A' && hx->gain_channel_A == 128)
  {
    hx->offset_A_128 = result;
    return 0;
  }

  return 1;
}

// changed!
// tally instead of dynamic allocation to array
int getRawDataMean(HX711 *hx, int samples)
{
  long sum = 0;

  for (int i = 0; i < samples; i++)
  {
    int val = getRawData(hx);
    sum += val;
    usleep(5000);
  }

  return (int)(sum / samples);
}

// changed!
// simplified - only offset for A 128
int getDataMean(HX711 *hx, int samples)
{
  int result = getRawDataMean(hx, samples);
  return result - hx->offset_A_128;
}

// changed!
int getWeightMean(HX711 *hx, int samples)
{
  int result = getRawDataMean(hx, samples);

  if (hx->scale_ratio_A_128 == 0)
    return 0;
  // simplfied - only assuming A 128
  return (int)((result - hx->offset_A_128) / hx->scale_ratio_A_128);
}