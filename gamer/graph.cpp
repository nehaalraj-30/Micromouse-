#include "graph.h"
#include <stdlib.h>
#include <Arduino.h> // Assuming you're using Arduino, include it

// Define global variables
extern int grid[GRID_SIZE][GRID_SIZE];
extern float i_cord;
extern float j_cord;

NODE head = NULL;
NODE tail = NULL;

// Function definitions

void enqueue(int x, int y) {
    NODE new_node = new struct node; // Allocate memory for the new node
    new_node->x = x;
    new_node->y = y;
    new_node->next = NULL;

    if (tail != NULL) {
        tail->next = new_node;
    }

    tail = new_node;

    if (head == NULL) {
        head = new_node;  // Queue was empty
    }
}

void dequeue() {
    if (head != NULL) {
        NODE temp = head;
        head = head->next;
        delete temp;
        if (head == NULL) {
            tail = NULL;  // Queue is empty now
        }
    }
}

void print_queue() {
    NODE temp = head;
    while (temp != NULL) {
        Serial.print("(");
        Serial.print(temp->x);
        Serial.print(",");
        Serial.print(temp->y);
        Serial.print(") ");
        temp = temp->next;
    }
    Serial.println();
}

void flood_fill(int x, int y) {
    enqueue(x, y);
    head = tail;

    int i, j;

    i = head->x;
    j = head->y;

    grid[i][j] = 1;

    while (head != NULL) {
        i = head->x;
        j = head->y;

        if (i > 0 && !grid[i - 1][j]) {  // One step above
            grid[i - 1][j] = grid[i][j] + 1;
            enqueue(i - 1, j);
        }
        if (i < GRID_SIZE - 1 && !grid[i + 1][j]) {  // One step below
            grid[i + 1][j] = grid[i][j] + 1;
            enqueue(i + 1, j);
        }
        if (j < GRID_SIZE - 1 && !grid[i][j + 1]) {  // One step left
            grid[i][j + 1] = grid[i][j] + 1;
            enqueue(i, j + 1);
        }
        if (j > 0 && !grid[i][j - 1]) {  // One step right
            grid[i][j - 1] = grid[i][j] + 1;
            enqueue(i, j - 1);
        }

        dequeue();
    }
}

void initiate() {
    for (int i = 1; i < GRID_SIZE - 1; i++) {
        for (int j = 1; j < GRID_SIZE - 1; j++) {
            grid[i][j] = 0;
        }
    }

    for (int i = 2; i < GRID_SIZE - 1; i += 2) {
        for (int j = 2; j < GRID_SIZE - 1; j += 2) {
            grid[i][j] = 69;  // Mark obstacles
        }
    }

    // Set borders as obstacles
    for (int i = 0; i < GRID_SIZE; i++) {
        grid[0][i] = 69;
        grid[i][0] = 69;
        grid[GRID_SIZE - 1][i] = 69;
        grid[i][GRID_SIZE - 1] = 69;
    }
}

void reset() {
    for (int i = 1; i < GRID_SIZE - 1; i++) {
        for (int j = 1; j < GRID_SIZE - 1; j++) {
            if (grid[i][j] != 69) {
                grid[i][j] = 0;
            }
        }
    }
}

void printGrid() {
  for (int i = 0; i < GRID_SIZE; i++) {
      for (int j = 0; j < GRID_SIZE; j++) {
          Serial.print(grid[i][j]);
          Serial.print("   ");  // Print space between numbers
      }
      Serial.println();  // Print newline after each row
  }
}

int best_choice() {
    int cell_i = (int)i_cord;//TBD
    int cell_j = (int)j_cord;

    int options[4];
    options[0] = grid[cell_i][cell_j + 1];  // East
    options[1] = grid[cell_i + 1][cell_j];  // South
    options[2] = grid[cell_i][cell_j - 1];  // West
    options[3] = grid[cell_i - 1][cell_j];  // North

    int min_index = -1;
    int min_value = 96;  // Initialize with a large value

    for (int i = 0; i < 4; i++) {
        if (options[i] < min_value) {
            min_value = options[i];
            min_index = i;
        }
    }
    return min_index + 1;  // Return direction (1 for East, 2 for South, etc.)
}
