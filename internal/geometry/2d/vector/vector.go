// Package vector defines utility functions on vectors in 2D ambient space.
package vector

import (
	"github.com/downflux/go-geometry/2d/vector"
)

// IsNormalOrientation checks the angle between two vectors and returns true if
// the angle is strictly less than π.
//
// A non-negative determinant of two 2D vectors indicates the angle between them
// is 0 <= 𝜃 <= π. We care in orientation-related problems to ensure that
// vectors are not anti-parallel, so an addional check is necessary.
func IsNormalOrientation(v vector.V, u vector.V) bool {
	return vector.Determinant(v, u) > 0 || vector.Within(vector.Unit(v), vector.Unit(u))
}
