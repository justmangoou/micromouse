#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Direction {
	North = 0,
	East = 1,
	South = 2,
	West = 3,
}

impl Direction {
	pub const fn left(self) -> Self {
		match self {
			Self::North => Self::West,
			Self::West => Self::South,
			Self::South => Self::East,
			Self::East => Self::North,
		}
	}

	pub const fn right(self) -> Self {
		match self {
			Self::North => Self::East,
			Self::East => Self::South,
			Self::South => Self::West,
			Self::West => Self::North,
		}
	}

	pub const fn opposite(self) -> Self {
		match self {
			Self::North => Self::South,
			Self::East => Self::West,
			Self::South => Self::North,
			Self::West => Self::East,
		}
	}
}
