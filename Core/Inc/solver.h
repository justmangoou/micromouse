#ifndef SOLVER_H
#define SOLVER_H

#include "stdint.h"
#include "stdbool.h"

#define STACK_SIZE_INDEX 0
#define STACK_POINTER_INDEX 1

#define SIZE 16
#define HALF_SIZE (SIZE / 2)

typedef struct {
  uint8_t x, y, val;
  bool visited;

  struct MazeNode *North;
  struct MazeNode *South;
  struct MazeNode *East;
  struct MazeNode *west;
} MazeNode;

typedef struct {
  MazeNode *Map[SIZE][SIZE];
} Maze;

typedef struct {
  uint_least8_t Properties;
  MazeNode *Array[];
} MazeStack;

MazeStack *MazeStack_New(void);
void MazeStack_Delete(MazeStack **stack);
void MazeStack_Push(MazeStack *stack, MazeNode *node);
MazeNode *MazeStack_Pop(MazeStack *stack);

#endif //SOLVER_H
