// Package plane defines a geometric half-plane.
package plane

import (
	"github.com/downflux/orca/geometry/vector"
)

// LC defines a linear constraint of the form
//
//   A • X <= B
//
// For two dimensions, this is of the form
//
//   ax + by = c
type LC interface {
	A() vector.V
	B() float64
}

// HP defines a half-plane, geometrically consisting of a normal vector n to the
// plane, and a point p which defines the origin of n. Vectors facing away from
// n are not permissible within the half-plane.
type HP struct {
	p vector.V
	n vector.V

	aCache vector.V
	lCache vector.V
	bCache float64
}

func New(p vector.V, n vector.V) *HP {
	aCache := vector.Scale(-1, n)
	return &HP{
		n:      n,
		p:      p,
		aCache: aCache,
		bCache: vector.Dot(aCache, p),
	}
}

func (p HP) N() vector.V { return p.n }
func (p HP) P() vector.V { return p.p }

func (p HP) A() vector.V { return p.aCache }
func (p HP) B() float64  { return p.bCache }

// l returns the characteristic line along the plane is bisected. Points to the
// "left" of the line are not permissible.
func (p HP) l() vector.V { return *vector.New(-p.N().Y(), p.N().X()) }

func (p HP) in(v vector.V) bool {
	// Generate a vector with tail on l and pointing towards the input.
	w := vector.Sub(v, p.P())

	// Check relative orientation between w and l.
	return vector.Determinant(w, p.l()) >= 0
}

func Within(a HP, b HP, tolerance float64) bool {
	return vector.Within(a.N(), b.N(), tolerance) && vector.Within(a.P(), b.P(), tolerance)
}
