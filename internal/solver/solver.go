package solver

import (
	"github.com/downflux/go-geometry/circle"
	"github.com/downflux/go-geometry/vector"
	"github.com/downflux/go-orca/internal/constraint"
)

type S struct {
	cs        []constraint.C
	tolerance float64
}

func New(cs []constraint.C, tolerance float64) *S {
	return &S{
		cs:        cs,
		tolerance: tolerance,
	}
}

// Solve attempts to find a vector which satisfies all constraints and minimizes
// the distance to the input preferred vector v.
func (s *S) Solve(v vector.V, c circle.C) vector.V {
	return vector.V{}
}
