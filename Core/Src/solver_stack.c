/**
 * @file solver_stack.c
 * @brief Stack implementation for maze solving. ONLY USE FOR FLOODFILL
 */
#include "stdlib.h"
#include "stdbool.h"
#include "solver.h"

MazeStack *MazeStack_New(void) {
    MazeStack *stack = malloc(sizeof(MazeStack));

    if (stack == NULL)
        return NULL;

    stack->Properties[STACK_POINTER_INDEX] = 0;
    stack->Properties[STACK_SIZE_INDEX] = STACK_SIZE;

    return stack;
}

void MazeStack_Delete(MazeStack **stack) {
    if (stack == NULL || *stack == NULL) {
        return;
    }

    free(*stack);
    *stack = NULL;
}

bool MazeStack_IsEmpty(MazeStack *stack) {
    if (stack->Properties[STACK_POINTER_INDEX] == 0)
        return 1;
    return 0;
}

void MazeStack_Push(MazeStack *stack, MazeNode *node) {
    const uint_least8_t index = stack->Properties[STACK_POINTER_INDEX];
    stack->Array[index] = node;
    stack->Properties[STACK_POINTER_INDEX] += 1;
}

MazeNode *MazeStack_Pop(MazeStack *stack) {
    const uint_least8_t index = stack->Properties[STACK_POINTER_INDEX] - 1;
    MazeNode *node = stack->Array[index];
    stack->Properties[STACK_POINTER_INDEX] -= 1;

    return node;
}
