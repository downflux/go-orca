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
