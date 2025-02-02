
#define WIDTH 80      // Width of the terminal screen
#define HEIGHT 6      // Number of rows for the ski lift
#define NUM_CHAIRS 10 // Number of ski lift chairs
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include "../../../include/ScreenPrinter/ScreenPrinter.h"
static volatile bool is_running = true;
static volatile int offset;
static void clearScreen() {
  printf("\033[H\033[J"); // ANSI escape code to clear the screen
}

static void drawSkiLift(int offset) {
  char screen[HEIGHT][WIDTH];

  // Clear the screen array
  for (int i = 0; i < HEIGHT; i++) {
    memset(screen[i], ' ', WIDTH - 1);
    screen[i][WIDTH - 1] = '\0'; // Null-terminate each row
  }

  // Draw the top row of chairs
  for (int i = 0; i < NUM_CHAIRS; i++) {
    int pos = (i * 8 + offset) % WIDTH; // Calculate position with wraparound
    if (pos + 1 > 4 && pos + 1 < WIDTH - 10) {
      screen[0][pos] = '-';
      screen[0][pos + 1] = 'o';
      screen[0][pos + 2] = '-';
      screen[1][pos] = '/';
      screen[1][pos + 1] = '|';
      screen[1][pos + 2] = '\\';
    }
  }

  // Draw the bottom row of chairs
  for (int i = 0; i < NUM_CHAIRS; i++) {
    int pos = (i * 8 - offset + WIDTH) %
              WIDTH; // Subtract offset for reverse direction
    if (pos + 2 > 4 &&
        pos + 2 < WIDTH - 10) { // Ensure chair fits within the screen width
      screen[HEIGHT - 2][pos] = '-';
      screen[HEIGHT - 2][pos + 1] = 'o';
      screen[HEIGHT - 2][pos + 2] = '-';
      screen[HEIGHT - 1][pos] = '/';
      screen[HEIGHT - 1][pos + 1] = '|';
      screen[HEIGHT - 1][pos + 2] = '\\';
    }
  }

  // Draw the left-most and right-most vertical lines
  for (int i = 0; i < NUM_CHAIRS; i++) {
    int pos_top = (i * 8 + offset) % WIDTH; // Position in the top row
    int pos_bottom =
        (i * 8 - offset + WIDTH) % WIDTH; // Position in the bottom row

    // Left vertical connection (top to bottom)
    if (pos_top + 1 > 5 && pos_top + 1 < WIDTH - 10) {
      screen[2][1] = '|'; // Left vertical line
      screen[3][1] = '|';
    }

    // Right vertical connection (bottom to top)
    if (pos_bottom + 2 > 8 && pos_bottom + 2 < WIDTH - 5) {
      screen[2][WIDTH - 6] = '|'; // Right vertical line
      screen[3][WIDTH - 6] = '|';
    }
  }

  // Print the screen to the terminal
  for (int i = 0; i < HEIGHT; i++) {
    printf("%s\n", screen[i]);
  }
}

void runSkiLift(screenprinter_self_t* self) {
  clearScreen();
  printf("Ski lift animation\n");
  printf(
      "---------------------------------------------------------------------"
      "-------\n\n\n");
  drawSkiLift(offset);
  printf("\n\n");
  printf(
      "---------------------------------------------------------------------"
      "-------\n");
  offset = (offset + 1) % WIDTH; // Update the offset for the next frame
}

void initSkiLift(screenprinter_self_t* self) {
  offset = 0;
}