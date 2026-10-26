#include "floodfill.h"

namespace nav {
namespace {
bool validDirection(Direction d) { return static_cast<unsigned>(d) < 4; }
uint8_t bit(Direction d) { return static_cast<uint8_t>(1u << static_cast<unsigned>(d)); }
Direction opposite(Direction d) { return static_cast<Direction>((static_cast<unsigned>(d) + 2) % 4); }
}

Cell FloodFill::neighbor(Cell c, Direction d) {
    switch (d) {
        case Direction::North: ++c.y; break;
        case Direction::East: ++c.x; break;
        case Direction::South: --c.y; break;
        case Direction::West: --c.x; break;
    }
    return c;
}

bool FloodFill::contains(Cell c) const {
    return c.x >= 0 && c.x < width_ && c.y >= 0 && c.y < height_;
}

bool FloodFill::reset(int width, int height) {
    if (width < 1 || width > MAX_SIZE || height < 1 || height > MAX_SIZE) return false;
    width_ = width;
    height_ = height;
    mode_ = MazeMode::Explore;
    for (int i = 0; i < MAX_CELLS; ++i) {
        walls_[i] = known_[i] = 0;
        goals_[i] = false;
        distances_[i] = UNREACHABLE;
    }
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            Cell c{x, y};
            for (unsigned d = 0; d < 4; ++d) {
                Direction dir = static_cast<Direction>(d);
                if (!contains(neighbor(c, dir))) setPassage(c, dir, Passage::Wall);
            }
        }
    }
    setCenterGoals();
    return true;
}

Passage FloodFill::passage(Cell c, Direction d) const {
    if (!contains(c) || !validDirection(d)) return Passage::Wall;
    int i = index(c);
    if (walls_[i] & bit(d)) return Passage::Wall;
    return (known_[i] & bit(d)) ? Passage::Open : Passage::Unknown;
}

bool FloodFill::setPassage(Cell c, Direction d, Passage p) {
    if (!contains(c) || !validDirection(d) || static_cast<unsigned>(p) > 2) return false;
    Cell other = neighbor(c, d);
    if (!contains(other) && p != Passage::Wall) return false;
    for (int side = 0; side < 2; ++side) {
        int i = index(c);
        uint8_t mask = bit(d);
        walls_[i] = static_cast<uint8_t>((walls_[i] & ~mask) | (p == Passage::Wall ? mask : 0));
        known_[i] = static_cast<uint8_t>((known_[i] & ~mask) | (p != Passage::Unknown ? mask : 0));
        if (!contains(other)) break;
        c = other;
        d = opposite(d);
    }
    dirty_ = true;
    return true;
}

bool FloodFill::setGoals(const Cell* goals, int count) {
    if (!goals || count < 1 || count > MAX_CELLS) return false;
    for (int i = 0; i < count; ++i) if (!contains(goals[i])) return false;
    for (int i = 0; i < MAX_CELLS; ++i) goals_[i] = false;
    for (int i = 0; i < count; ++i) goals_[index(goals[i])] = true;
    dirty_ = true;
    return true;
}

void FloodFill::setCenterGoals() {
    Cell centers[] = {{(width_ - 1) / 2, (height_ - 1) / 2},
                      {width_ / 2, (height_ - 1) / 2},
                      {(width_ - 1) / 2, height_ / 2}, {width_ / 2, height_ / 2}};
    setGoals(centers, 4);
}

bool FloodFill::isGoal(Cell c) const { return contains(c) && goals_[index(c)]; }

void FloodFill::setMode(MazeMode mode) {
    mode_ = mode;
    dirty_ = true;
}

bool FloodFill::traversable(Cell c, Direction d) const {
    Passage p = passage(c, d);
    return contains(neighbor(c, d)) && (p == Passage::Open || (p == Passage::Unknown && mode_ == MazeMode::Explore));
}

void FloodFill::flood() {
    int head = 0, tail = 0;
    for (int i = 0; i < width_ * height_; ++i) {
        distances_[i] = goals_[i] ? 0 : UNREACHABLE;
        if (goals_[i]) queue_[tail++] = static_cast<uint16_t>(i);
    }
    while (head < tail) {
        int i = queue_[head++];
        Cell c{i % width_, i / width_};
        for (unsigned d = 0; d < 4; ++d) {
            Direction dir = static_cast<Direction>(d);
            if (!traversable(c, dir)) continue;
            int n = index(neighbor(c, dir));
            if (distances_[n] != UNREACHABLE) continue;
            distances_[n] = static_cast<uint16_t>(distances_[i] + 1);
            queue_[tail++] = static_cast<uint16_t>(n);
        }
    }
    dirty_ = false;
}

uint16_t FloodFill::distance(Cell c) {
    if (dirty_) flood();
    return contains(c) ? distances_[index(c)] : UNREACHABLE;
}

bool FloodFill::nextMove(Cell from, Direction heading, Cell& next, Direction& direction) {
    if (!validDirection(heading)) return false;
    uint16_t current = distance(from);
    if (current == 0 || current == UNREACHABLE) return false;
    const unsigned turns[] = {0, 3, 1, 2};
    for (unsigned turn : turns) {
        Direction d = static_cast<Direction>((static_cast<unsigned>(heading) + turn) % 4);
        if (!traversable(from, d)) continue;
        Cell n = neighbor(from, d);
        if (distances_[index(n)] == current - 1) {
            next = n;
            direction = d;
            return true;
        }
    }
    return false;
}
}  // namespace nav
