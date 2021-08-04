// Package plane defines a geometric half-plane.
package plane

import (
	"github.com/downflux/orca/geometry/vector"
)

type HP struct {
	p vector.V
	n vector.V

	lCache vector.V
}

func New(p vector.V, n vector.V) *HP {
	return &HP{
		n:      n,
		p:      p,
		lCache: *vector.New(-n.Y(), n.X()),
	}
}

func (p HP) N() vector.V { return p.n }
func (p HP) P() vector.V { return p.p }

// l returns the characteristic line along the plane is bisected. Points to the
// "left" of the line are not permissible.
func (p HP) l() vector.V { return p.lCache }

func (p HP) in(v vector.V) bool {
	// Generate a vector with tail on l and pointing towards the input.
	w := vector.Sub(v, p.P())

	// Check relative orientation between w and l.
	return vector.Determinant(w, p.l()) >= 0
}

func Within(a HP, b HP, tolerance float64) bool {
	return vector.Within(a.N(), b.N(), tolerance) && vector.Within(a.P(), b.P(), tolerance)
}
