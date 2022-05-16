package unbounded

import (
	"math"

	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
)

type M struct {
}

func (M) Bound(c constraint.C) (segment.S, bool) {
	l := hyperplane.Line(hyperplane.HP(c))
	return *segment.New(l, math.Inf(-1), math.Inf(0)), true
}

func (M) Within(v vector.V) bool { return true }
func (M) V(v vector.V) vector.V  { return v }
