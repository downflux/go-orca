package rectangle

import (
	"math"

	"github.com/downflux/orca/geometry/vector"
)

type R struct {
	min vector.V
	max vector.V
}

func New(v vector.V, u vector.V) *R {
	return &R{
		min: *vector.New(
			math.Min(v.X(), u.X()),
			math.Min(v.Y(), u.Y()),
		),
		max: *vector.New(
			math.Max(v.X(), u.X()),
			math.Max(v.Y(), u.Y()),
		),
	}
}

func (r R) Min() vector.V { return r.min }
func (r R) Max() vector.V { return r.max }
func (r R) D() vector.V   { return *vector.New(r.Max().X()-r.Min().X(), r.Max().Y()-r.Min().Y()) }
func (r R) In(v vector.V) bool {
	return (r.Min().X() <= v.X()) && (v.X() <= r.Max().X()) && (r.Min().Y() <= v.Y()) && (v.Y() <= r.Max().Y())
}

func (r R) Intersect(s R) (R, bool) {
	min := *vector.New(
		math.Max(r.Min().X(), s.Min().X()),
		math.Max(r.Min().Y(), s.Min().Y()))
	max := *vector.New(
		math.Min(r.Max().X(), s.Max().X()),
		math.Min(r.Max().Y(), s.Max().Y()),
	)

	if min.X() > max.X() || min.Y() > max.Y() {
		return R{}, false
	}
	return *New(min, max), true
}
