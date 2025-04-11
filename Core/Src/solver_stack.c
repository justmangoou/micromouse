#include "stdlib.h"
#include "solver.h"

MazeStack *MazeStack_New(void) {
    MazeStack *stack = malloc(sizeof(MazeStack));

    if (stack == NULL)
        return NULL;

    for (int i = 0; i < HALF_SIZE; i++)
        stack->Array[i] = NULL;

    return stack;
}

void MazeStack_Delete(MazeStack **stack) {
    free(*stack);
    *stack = NULL;
}

void MazeStack_Push(MazeStack *stack, MazeNode *node) {
    for (int i = 0; i < HALF_SIZE; i++) {
        if (stack->array[i] == NULL) {
            stack->array[i] = node;
            return;
        }
    }
}

MazeNode *MazeStack_Pop(MazeStack *stack) {

}
