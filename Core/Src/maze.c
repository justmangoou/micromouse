#include "stdlib.h"
#include "main.h"
#include "maze.h"

MazeNode* MazeNode_New(const uint8_t x, const uint8_t y)
{
    MazeNode* node = malloc(sizeof(MazeNode));
    if (!node) return NULL;

    node->X = x;
    node->Y = y;

    if (x < HALF_SIZE && y < HALF_SIZE)
        node->Val = (HALF_SIZE - 1 - x) + (HALF_SIZE - 1 - y);
    else if (x < HALF_SIZE && y >= HALF_SIZE)
        node->Val = (HALF_SIZE - 1 - x) + (y - HALF_SIZE);
    else if (x >= HALF_SIZE && y < HALF_SIZE)
        node->Val = (x - HALF_SIZE) + (HALF_SIZE - 1 - y);
    else
        node->Val = (x - HALF_SIZE) + (y - HALF_SIZE);

    return node;
}

Maze* Maze_New()
{
    Maze* maze = malloc(sizeof(Maze));
    uint8_t i, j;

    if (!maze) return NULL;

    for (i = 0; i < SIZE; ++i)
        for (j = 0; j < SIZE; ++j)
        {
            maze->Nodes[i][j] = MazeNode_New(i, j);
        }

    for (i = 0; i < SIZE; ++i)
        for (j = 0; j < SIZE; ++j)
        {
            maze->Nodes[i][j]->North = j == 0 ? NULL : maze->Nodes[i][j + 1];
            maze->Nodes[i][j]->East = j == SIZE - 1 ? NULL : maze->Nodes[i + 1][j];
            maze->Nodes[i][j]->South = j == 0 ? NULL : maze->Nodes[i][j - 1];
            maze->Nodes[i][j]->West = j == SIZE - 1 ? NULL : maze->Nodes[i - 1][j];
        }

    return maze;
}

void MazeNode_Delete(MazeNode** node)
{
    free(*node);
    *node = NULL;
}

void Maze_Delete(Maze** maze)
{
    for (uint8_t i = 0; i < SIZE; ++i)
        for (uint8_t j = 0; j < SIZE; ++j)
            MazeNode_Delete(&(*maze)->Nodes[i][j]);

    free(*maze);
    *maze = NULL;
}

void MazeNode_SetWall(MazeNode* node, const Direction dir)
{
    switch (dir)
    {
    case NORTH:
        if (node->X > 0)
            node->North = NULL;
        break;
    case EAST:
        if (node->Y < SIZE - 1)
            node->East = NULL;
        break;
    case SOUTH:
        if (node->X < SIZE - 1)
            node->South = NULL;
        break;
    case WEST:
        if (node->Y > 0)
            node->West = NULL;
        break;
    }
}

void MazeNode_SetValue(MazeNode* node, const uint8_t val)
{
    node->Val = val;
}

void MazeNode_SetVisited(MazeNode* node)
{
    node->Visited = true;
}

uint8_t MazeNode_GetSmallestNeighbor(MazeNode* node)
{
    uint8_t smallest_neighbor_val = UINT8_MAX;

    if (node->North != NULL && node->North->South != NULL && node->Val < smallest_neighbor_val)
        smallest_neighbor_val = node->North->Val;
    if (node->East != NULL && node->East->West != NULL && node->Val < smallest_neighbor_val)
        smallest_neighbor_val = node->East->Val;
    if (node->South != NULL && node->South->North != NULL && node->Val < smallest_neighbor_val)
        smallest_neighbor_val = node->South->Val;
    if (node->West != NULL && node->West->East != NULL && node->Val < smallest_neighbor_val)
        smallest_neighbor_val = node->West->Val;

    return smallest_neighbor_val;
}

uint8_t MazeNode_GetSmallestNeighborDir(MazeNode* node, const uint8_t preferred_dir)
{
    const uint8_t smallest_neighbor_val = MazeNode_GetSmallestNeighbor(node);

    switch (preferred_dir)
    {
    case NORTH:
        if (node->North != NULL && node->North->South != NULL && node->North->Val == smallest_neighbor_val)
            return NORTH;
        break;
    case EAST:
        if (node->East != NULL && node->East->West != NULL && node->East->Val == smallest_neighbor_val)
            return EAST;
        break;
    case SOUTH:
        if (node->South != NULL && node->South->North != NULL && node->South->Val == smallest_neighbor_val)
            return SOUTH;
        break;
    case WEST:
        if (node->West != NULL && node->West->East != NULL && node->West->Val == smallest_neighbor_val)
            return WEST;
        break;
    default:
        // Do nothing
        break;
    }

    if (node->North && node->North->Val == smallest_neighbor_val && !node->North->Visited)
        return NORTH;
    if (node->East && node->East->Val == smallest_neighbor_val && !node->East->Visited)
        return EAST;
    if (node->South && node->South->Val == smallest_neighbor_val && !node->South->Visited)
        return SOUTH;
    if (node->West && node->West->Val == smallest_neighbor_val && !node->West->Visited)
        return WEST;

    if (node->North && node->North->Val == smallest_neighbor_val)
        return NORTH;
    if (node->East && node->East->Val == smallest_neighbor_val)
        return EAST;
    if (node->South && node->South->Val == smallest_neighbor_val)
        return SOUTH;

    return WEST;
}

bool MazeNode_ValueCheck(MazeNode* node)
{
    return MazeNode_GetSmallestNeighbor(node) + 1 == node->Val;
}

void MazeNode_UpdateValue(MazeNode* node)
{
    node->Val = MazeNode_GetSmallestNeighbor(node) + 1;
}

void MazeNode_PushOpenNeighbors(MazeNode* node, MazeStack* stack)
{
    if (node->North != NULL && node->North->South != NULL)
        MazeStack_Push(stack, node->North);
    if (node->East != NULL && node->East->West != NULL)
        MazeStack_Push(stack, node->East);
    if (node->South != NULL && node->South->North != NULL)
        MazeStack_Push(stack, node->South);
    if (node->West != NULL && node->West->East != NULL)
        MazeStack_Push(stack, node->West);
}
