#define WIDTH 80      // Width of the terminal screen
#define HEIGHT 6      // Number of rows for the ski lift
#define NUM_CHAIRS 10 // Number of ski lift chairs
#define ANIMATION_HEIGHT 13
#include "../../../include/ScreenPrinter/ScreenPrinter.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

void skiLiftInit(int32_t offset);
void skiLiftUpdate(int32_t offset);
void printMotionStatus(char *str_motion);
void printGateStatus(char *str_gate);

void moveCursor(int32_t row, int32_t col)
{
  // Move cursor to (row, col)
  fprintf(stdout, "\033[%d;%dH", row, col);
  fflush(stdout);
}
void clearScreen()
{
  fprintf(stdout, "\033[H\033[J"); // ANSI escape code to clear the screen
  fflush(stdout);
}
void clearLine()
{
  fprintf(stdout, "\033[2K"); // Clear the entire line
  fflush(stdout);
}

void drawSkiLift(int32_t offset)
{
  char screen[HEIGHT][WIDTH];

  // Clear the screen array
  for (int32_t i = 0; i < HEIGHT; i++)
  {
    memset(screen[i], ' ', WIDTH - 1);
    screen[i][WIDTH - 1] = '\0';
  }

  // Draw the top row of chairs
  for (int32_t i = 0; i < NUM_CHAIRS; i++)
  {
    int32_t pos = (i * 8 + offset) % WIDTH;
    if (pos + 1 > 4 && pos + 1 < WIDTH - 10)
    {
      screen[0][pos] = '-';
      screen[0][pos + 1] = 'o';
      screen[0][pos + 2] = '-';
      screen[1][pos] = '/';
      screen[1][pos + 1] = '|';
      screen[1][pos + 2] = '\\';
    }
  }

  // Draw the bottom row of chairs
  for (int32_t i = 0; i < NUM_CHAIRS; i++)
  {
    int32_t pos = (i * 8 - offset + WIDTH) % WIDTH;
    if (pos + 2 > 4 && pos + 2 < WIDTH - 10)
    {
      screen[HEIGHT - 2][pos] = '-';
      screen[HEIGHT - 2][pos + 1] = 'o';
      screen[HEIGHT - 2][pos + 2] = '-';
      screen[HEIGHT - 1][pos] = '/';
      screen[HEIGHT - 1][pos + 1] = '|';
      screen[HEIGHT - 1][pos + 2] = '\\';
    }
  }

  // Draw the left-most and right-most vertical lines
  for (int32_t i = 0; i < NUM_CHAIRS; i++)
  {
    int32_t pos_top =
        (i * 8 + offset) % WIDTH;
    int32_t pos_bottom =
        (i * 8 - offset + WIDTH) % WIDTH;

    // Left vertical connection (top to bottom)
    if (pos_top + 1 > 5 && pos_top + 1 < WIDTH - 10)
    {
      screen[2][1] = '|';
      screen[3][1] = '|';
    }

    // Right vertical connection (bottom to top)
    if (pos_bottom + 2 > 8 && pos_bottom + 2 < WIDTH - 5)
    {
      screen[2][WIDTH - 6] = '|';
      screen[3][WIDTH - 6] = '|';
    }
  }

  // Print the screen to the terminal
  for (int32_t i = 0; i < HEIGHT; i++)
  {
    fprintf(stdout, "%s\n", screen[i]);
  }
}

void skiLiftInit(int32_t offset)
{
  clearScreen();
  fprintf(stdout, "Ski Lift Animation\n");
  fprintf(stdout,
          "---------------------------------------------------------------------"
          "-------\n\n\n");
  drawSkiLift(offset);
  fprintf(stdout, "\n\n");
  fprintf(stdout,
          "---------------------------------------------------------------------"
          "-------\n");
  // offset = (offset + 1) % WIDTH;
}
void skiLiftUpdate(int32_t offset)
{
  for (int32_t i = HEIGHT + 3 - 1; i > 0; i--)
  {
    moveCursor(i + 4, 1);
    clearLine();
  }
  drawSkiLift(offset);
  fprintf(stdout, "\n\n");
  fprintf(stdout,
          "---------------------------------------------------------------------"
          "-------\n");
  // offset = (offset + 1) % WIDTH;
}
void printMotionStatus(char *str_motion)
{
  moveCursor(ANIMATION_HEIGHT + 1, 1);
  clearLine();
  fprintf(stdout, "Lift Status: %s\n", str_motion);
  fflush(stdout);
}
void printGateStatus(char *str_gate)
{
  moveCursor(ANIMATION_HEIGHT + 2, 1);
  clearLine();
  fprintf(stdout, "Gate Status: %s\n", str_gate);
  fflush(stdout);
}

void printReadyWeight(float ready_weight)
{
  moveCursor(ANIMATION_HEIGHT + 3, 1);
  clearLine();
  fprintf(stdout, "Current weight on the ready area: %.2f\n", ready_weight);
  fflush(stdout);
}

void printLiftWeight(float lift_weight)
{
  moveCursor(ANIMATION_HEIGHT + 4, 1);
  clearLine();
  fprintf(stdout, "The weight will be on the current chair lift: %.2f\n", lift_weight);
  fflush(stdout);
}

void printDebugStr(char *debug_str)
{
  moveCursor(ANIMATION_HEIGHT + 5, 1);
  clearLine();
  fprintf(stdout, "[Debug] String: %s\n", debug_str);
  fflush(stdout);
}
void printDebugInt(int32_t debug_int)
{
  moveCursor(ANIMATION_HEIGHT + 6, 1);
  clearLine();
  fprintf(stdout, "[Debug] Int: %d\n", debug_int);
  fflush(stdout);
}
