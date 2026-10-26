#pragma once

#include <stdint.h>

namespace nav {

// Maze coordinates: x east, y north; (0, 0) is the southwest cell.
enum class Direction : uint8_t { North, East, South, West };
enum class Passage : uint8_t { Unknown, Open, Wall };
enum class MazeMode : uint8_t { Explore, KnownOnly };
struct Cell { int x, y; };

// Cell-level micromouse planner. No allocation; supports up to 16 x 16 cells.
// Feed discrete wall observations at cell centers, then execute nextMove() with
// a cell motion controller. It does not interpret raw ToF readings or drive motors.
class FloodFill {
public:
    static constexpr int MAX_SIZE = 16;
    static constexpr int MAX_CELLS = MAX_SIZE * MAX_SIZE;
    static constexpr uint16_t UNREACHABLE = UINT16_MAX;

    FloodFill() { reset(); }
    // Invalid dimensions leave the existing maze intact. Reset uses center goals.
    bool reset(int width = MAX_SIZE, int height = MAX_SIZE);
    bool contains(Cell c) const;
    int width() const { return width_; }
    int height() const { return height_; }
    Passage passage(Cell c, Direction d) const;
    // Updates both sides of an edge. Outer boundary walls cannot be removed.
    bool setPassage(Cell c, Direction d, Passage p);
    // Replaces the goals atomically; duplicates are allowed.
    bool setGoals(const Cell* goals, int count);
    void setCenterGoals();
    bool isGoal(Cell c) const;
    void setMode(MazeMode mode);
    MazeMode mode() const { return mode_; }
    // Multi-source BFS from goals; edits are also reflooded lazily by queries.
    void flood();
    uint16_t distance(Cell c);
    // False at a goal, outside the maze or when no route exists; outputs untouched.
    // Equal routes prefer straight, left, right, then back relative to heading.
    bool nextMove(Cell from, Direction heading, Cell& next, Direction& direction);

private:
    int index(Cell c) const { return c.y * width_ + c.x; }
    static Cell neighbor(Cell c, Direction d);
    bool traversable(Cell c, Direction d) const;
    int width_ = MAX_SIZE, height_ = MAX_SIZE;
    uint8_t walls_[MAX_CELLS] = {};
    uint8_t known_[MAX_CELLS] = {};
    bool goals_[MAX_CELLS] = {};
    uint16_t distances_[MAX_CELLS] = {};
    uint16_t queue_[MAX_CELLS] = {};
    MazeMode mode_ = MazeMode::Explore;
    bool dirty_ = true;
};

}  // namespace nav
