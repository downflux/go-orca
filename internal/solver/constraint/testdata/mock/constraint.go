package standard

import (
	"github.com/downflux/go-geometry/vector"
)

// C implements constraint.C using the standard form
//
//   Ax <= B
type C struct {
	a []float64
	b float64
}

func New(a []float64, b float64) *C { return &C{a: a, b: b} }

func (r C) In(p vector.V) bool {
	a := *vector.New(r.a[0], r.a[1])
	return vector.Dot(a, p) <= r.b
}
