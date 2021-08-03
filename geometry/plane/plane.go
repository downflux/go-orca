// Package plane defines a geometric half-plane.
package plane

import (
	"github.com/downflux/orca/geometry/vector"
)

type HP struct {
	p vector.V
	n vector.V
}

func New(p vector.V, n vector.V) *HP {
	return &HP{n: n, p: p}
}

func (p HP) N() vector.V { return p.n }
func (p HP) P() vector.V { return p.p }

func Within(a HP, b HP, tolerance float64) bool {
	return vector.Within(a.N(), b.N(), tolerance) && vector.Within(a.P(), b.P(), tolerance)
}
