package constraint

import (
	"github.com/downflux/go-geometry/plane"
	"github.com/downflux/go-geometry/vector"
)

// C defines a 2D linear constraint of the (standard) form
//
//   A • X <= B
//
// For two dimensions, this is
//
//   a_x * x + a_y * y <= b
type C struct {
	hp plane.HP
}

func New(hp plane.HP) *C {
	return &C{
		hp: hp,
	}
}

// A returns the A vector of the contraint; returns [a, b] in the 2D case.
func (c C) A() []float64 {
	a := vector.Scale(-1, c.hp.N())
	return []float64{
		a.X(),
		a.Y(),
	}
}

// B returns the bound on the constraint.
func (c C) B() float64 {
	a := vector.Scale(-1, c.hp.N())
	return vector.Dot(a, c.hp.P())
}

// In returns if a point in the space satisfies the constraint.
func (c C) In(p vector.V) bool { return c.hp.In(p) }

func (c C) HP() plane.HP { return c.hp }
