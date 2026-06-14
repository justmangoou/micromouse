use crate::{MAZE_SIZE, utils::Direction};

impl Direction {
	pub const fn bit(self) -> u8 {
		1 << (self as u8)
	}
}

#[derive(Clone, Copy, Debug)]
pub struct Cell {
	pub value: u8,
	pub visited: bool,
	pub walls: u8,
}

impl Cell {
	#[allow(clippy::new_without_default)]
	pub const fn new() -> Self {
		Self {
			value: 0,
			visited: false,
			walls: 0,
		}
	}

	pub fn has_wall(&self, dir: Direction) -> bool {
		(self.walls & dir.bit()) != 0
	}

	pub fn set_wall(&mut self, dir: Direction) {
		self.walls |= dir.bit();
	}

	pub fn clear_wall(&mut self, dir: Direction) {
		self.walls &= !dir.bit();
	}
}

pub struct Maze {
	pub cells: [[Cell; MAZE_SIZE]; MAZE_SIZE],
}

impl Maze {
	#[allow(clippy::new_without_default)]
	pub const fn new() -> Self {
		Self {
			cells: [[Cell::new(); MAZE_SIZE]; MAZE_SIZE],
		}
	}

	pub fn get(&self, x: usize, y: usize) -> Option<&Cell> {
		self.cells.get(y)?.get(x)
	}

	pub fn get_mut(&mut self, x: usize, y: usize) -> Option<&mut Cell> {
		self.cells.get_mut(y)?.get_mut(x)
	}

	pub fn neighbor(x: usize, y: usize, dir: Direction) -> Option<(usize, usize)> {
		match dir {
			Direction::North if y > 0 => Some((x, y - 1)),
			Direction::East if x + 1 < MAZE_SIZE => Some((x + 1, y)),
			Direction::South if y + 1 < MAZE_SIZE => Some((x, y + 1)),
			Direction::West if x > 0 => Some((x - 1, y)),
			_ => None,
		}
	}

	pub fn add_wall(&mut self, x: usize, y: usize, dir: Direction) {
		self.cells[y][x].set_wall(dir);

		if let Some((nx, ny)) = Self::neighbor(x, y, dir) {
			self.cells[ny][nx].set_wall(dir.opposite());
		}
	}
}
