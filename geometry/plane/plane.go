// Package plane defines a geometric half-plane.
package plane

import (
	"github.com/downflux/orca/geometry/line"
	"github.com/downflux/orca/geometry/vector"
)

// HP defines a half-plane, geometrically consisting of a normal vector n to the
// plane, and a point p which defines the origin of n.
//
// N.B.: By arbitrary convention, vectors pointing away from N are not
// permissible within the half-plane.
type HP struct {
	line.L

	aCache []float64
	bCache float64
}

func New(p vector.V, n vector.V) *HP {
	a := vector.Scale(-1, n)

	// D returns the characteristic line along the plane is bisected. Points
	// to the "left" of the line are not permissible.
	//
	// N.B.: RVO2 defines the "right" side of the line as non-permissible,
	// but we have considered an anti-clockwise rotation of N() (e.g. +X to
	// +Y) to be more natural. See
	// https://github.com/snape/RVO2/blob/57098835aa27dda6d00c43fc0800f621724884cc/src/Agent.cpp#L314
	// for evidence of this distinction.
	d := *vector.New(-n.Y(), n.X())

	return &HP{
		L:      *line.New(p, d),
		aCache: []float64{a.X(), a.Y()},
		bCache: vector.Dot(a, p),
	}
}

// N returns the normal vector of the plane, pointing away from the invalid
// region.
func (p HP) N() vector.V { return *vector.New(p.D().Y(), -p.D().X()) }

func (p HP) Dimension() int { return 2 }
func (p HP) A() []float64   { return p.aCache }
func (p HP) B() float64     { return p.bCache }

func (p HP) In(v vector.V) bool {
	// Generate a vector with tail on D and pointing towards the input.
	w := vector.Sub(v, p.P())

	// Check relative orientation between w and D.
	return vector.Dot(w, p.N()) >= 0
}

func Within(a HP, b HP, tolerance float64) bool {
	return vector.Within(a.N(), b.N(), tolerance) && vector.Within(a.P(), b.P(), tolerance)
}
