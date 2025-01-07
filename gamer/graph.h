#ifndef GRAPH_H
#define GRAPH_H

#include "config.h"

// Define the node structure
struct node {
    int x;
    int y;
    struct node* next; // Use the full type name here
};

// Define NODE as a pointer to node
typedef struct node* NODE;

// External declarations
extern int grid[GRID_SIZE][GRID_SIZE];
extern float i_cord;
extern float j_cord;

extern NODE head;
extern NODE tail;

// Function declarations
void enqueue(int x, int y);       // Enqueue a node
void dequeue();                   // Dequeue a node
void print_queue();               // Print the queue (for debugging)
void flood_fill(int x, int y);    // Flood fill algorithm
void initiate();                  // Initialize the grid
void reset();                     // Reset the grid except for obstacles
void printGrid();
int best_choice();                // Choose the next best move
void turn_right();
void turn_left();

#endif // GRAPH_H
