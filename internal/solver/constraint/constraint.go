package constraint

import (
	"github.com/downflux/go-geometry/nd/hyperplane"
	"github.com/downflux/go-geometry/nd/vector"
)

// C defines a 2D linear constraint of the (standard) form
//
//   A • X <= B
//
// For two dimensions, this is
//
//   a_x * x + a_y * y <= b
type C struct {
	hp hyperplane.HP
}

func New(hp hyperplane.HP) *C {
	return &C{
		hp: hp,
	}
}

// A returns the A vector of the contraint; returns [a, b] in the 2D case.
func (c C) A() []float64 {
	a := vector.Scale(-1, c.HP().N())

	xs := make([]float64, a.Dimension())
	for i := vector.D(0); i < a.Dimension(); i++ {
		xs[i] = a.X(i)
	}

	return xs
}

// B returns the bound on the constraint.
func (c C) B() float64 {
	a := vector.Scale(-1, c.HP().N())
	return vector.Dot(a, c.HP().P())
}

func (c C) In(v vector.V) bool { return c.HP().In(v) }

func (c C) HP() hyperplane.HP { return c.hp }
