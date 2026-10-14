# Math

Shared `math` helpers: float `PI`, `Vec2`, `Pose`, distance, point interpolation, angle wrapping, and scalar/array degree-to-radian conversion.

Units: mm and radians; heading is counter-clockwise from +X.
`wrap()` returns angles in [-π, π]; non-finite inputs return NaN.

See [math_util.h](include/math_util.h) and [math_util.cpp](src/math_util.cpp).
