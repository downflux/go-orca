package solver

import (
	"math"

	"github.com/downflux/orca/geometry/lp/solver/reference/helper"
	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"
)

type S struct{}

type maxSpeedAgent struct {
	helper.Agent
}

// T ensures the underlying agent preferred velocity does not exceed the maximum
// speed of the agent.
func (a maxSpeedAgent) T() vector.V {
	if vector.SquaredMagnitude(a.Agent.T()) > a.Agent.S()*a.Agent.S() {
		return vector.Scale(a.Agent.S(), vector.Unit(a.Agent.T()))
	}
	return a.Agent.T()
	return vector.Scale(
		math.Min(1, vector.Magnitude(a.Agent.T())/a.Agent.S()),
		vector.Unit(a.Agent.T()),
	)
}

func (r S) Solve(a helper.Agent, cs []plane.HP) (vector.V, bool) {
	msa := maxSpeedAgent{Agent: a}
	h := helper.New(msa)

	solution := msa.T()
	for _, c := range cs {
		// Checks if the existing solution satisfies the new constraint.
		// In our constraint implementation, we have chosen the "left"
		// side of the plane direction D() to be valid, and so
		//
		//   D() x p < 0
		//
		// Implies p does not satisfy the constraint.
		//
		// N.B.: RVO2 chooses the "right" side of D() to be valid, and
		// therefore checks if the determinant is positive instead.
		if vector.Determinant(c.D(), vector.Sub(c.P(), solution)) < 0 {
			if r, ok := h.Add(c); ok {
				solution = r
			} else {
				return vector.V{}, false
			}
		}
	}
	return solution, true
}
