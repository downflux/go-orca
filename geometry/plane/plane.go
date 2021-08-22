// Package plane defines a geometric half-plane.
package plane

import (
	"github.com/downflux/orca/geometry/vector"
)

// HP defines a half-plane, geometrically consisting of a normal vector n to the
// plane, and a point p which defines the origin of n. Vectors facing away from
// n are not permissible within the half-plane.
type HP struct {
	p vector.V
	n vector.V

	aCache []float64
	lCache vector.V
	bCache float64
}

func New(p vector.V, n vector.V) *HP {
	a := vector.Scale(-1, n)
	return &HP{
		n:      n,
		p:      p,
		aCache: []float64{a.X(), a.Y()},
		bCache: vector.Dot(a, p),
	}
}

func (p HP) N() vector.V { return p.n }
func (p HP) P() vector.V { return p.p }

func (p HP) Dimension() int { return 2 }
func (p HP) A() []float64   { return p.aCache }
func (p HP) B() float64     { return p.bCache }

// D returns the characteristic line along the plane is bisected. Points to the
// "left" of the line are not permissible.
func (p HP) D() vector.V { return *vector.New(-p.N().Y(), p.N().X()) }

func (p HP) In(v vector.V) bool {
	// Generate a vector with tail on l and pointing towards the input.
	w := vector.Sub(v, p.P())

	// Check relative orientation between w and l.
	return vector.Determinant(w, p.D()) >= 0
}

func Within(a HP, b HP, tolerance float64) bool {
	return vector.Within(a.N(), b.N(), tolerance) && vector.Within(a.P(), b.P(), tolerance)
}
