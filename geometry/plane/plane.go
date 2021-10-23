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
}

// New constructs a half-plane passing through a point P and with normal N.
func New(p vector.V, n vector.V) *HP {
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
		L: *line.New(p, d),
	}
}

// N returns the normal vector of the plane, pointing away from the invalid
// region.
func (hp HP) N() vector.V { return *vector.New(hp.D().Y(), -hp.D().X()) }

// In checks if a given point in vector space is in valid region of the
// half-plane.
func (hp HP) In(p vector.V) bool {
	// Generate a vector with tail on D and pointing towards the input.
	v := vector.Sub(p, hp.P())

	// Check relative orientation between w and D.
	//
	// Remember that by the right hand rule, if v is on the "left" of the
	// plane,
	//
	//   D x v > 0, and
	//   N • v > 0
	//
	// As the left half of the plane is considered invalid, we are looking
	// instead for the complementary result.
	return vector.Determinant(hp.D(), v) <= 0
}

func Within(a HP, b HP, tolerance float64) bool {
	return vector.Within(a.N(), b.N(), tolerance) && vector.Within(a.P(), b.P(), tolerance)
}
