package solver

import (
	"github.com/downflux/orca/geometry/lp/solver/reference/helper"
	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"
)

type S struct{}

func (r S) Solve(cs []plane.HP, a helper.Agent) (vector.V, bool) {
	return vector.V{}, false
}
