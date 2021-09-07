package standard

import (
	"github.com/downflux/orca/geometry/vector"
)

// C implements constraint.C using the standard form
//
//   Ax <= B
type C struct {
	a []float64
	b float64
}

func New(a []float64, b float64) *C { return &C{a: a, b: b} }

func (c C) Dimension() int { return len(c.a) }
func (c C) A() []float64   { return c.a }
func (c C) B() float64     { return c.b }

func (r C) In(p vector.V) bool {
	a := *vector.New(r.A()[0], r.A()[1])
	return vector.Dot(a, p) <= r.B()
}
