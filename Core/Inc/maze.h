#ifndef SOLVER_H
#define SOLVER_H

#include "stdint.h"
#include "stdbool.h"

#include "main.h"

#define SIZE 16
#define HALF_SIZE (SIZE / 2)

#define STACK_SIZE 80
#define STACK_SIZE_INDEX 0
#define STACK_POINTER_INDEX 1

typedef struct MazeNode {
  uint8_t X, Y, Val;
  bool Visited;

  struct MazeNode *North;
  struct MazeNode *South;
  struct MazeNode *East;
  struct MazeNode *West;
} MazeNode;

typedef struct Maze {
  MazeNode *Nodes[SIZE][SIZE];
} Maze;

typedef struct MazeStack {
  uint_least8_t Properties[2];
  MazeNode *Array[];
} MazeStack;

MazeNode *MazeNode_New(uint8_t x, uint8_t y);
void MazeNode_Delete(MazeNode **node);
void MazeNode_SetWall(MazeNode *node, Direction dir);
void MazeNode_SetValue(MazeNode *node, uint8_t val);
void MazeNode_SetVisited(MazeNode *node);

uint8_t MazeNode_GetSmallestNeighbor(MazeNode *node);
uint8_t MazeNode_GetSmallestNeighborDir(MazeNode *node, uint8_t preferred_dir);
bool MazeNode_ValueCheck(MazeNode *node);
void MazeNode_UpdateValue(MazeNode *node);
void MazeNode_PushOpenNeighbors(MazeNode *node, MazeStack *stack);

Maze *Maze_New();
void Maze_Delete(Maze **maze);

MazeStack *MazeStack_New(void);
void MazeStack_Delete(MazeStack **stack);
bool MazeStack_IsEmpty(MazeStack *stack);
void MazeStack_Push(MazeStack *stack, MazeNode *node);
MazeNode *MazeStack_Pop(MazeStack *stack);

#endif //SOLVER_H