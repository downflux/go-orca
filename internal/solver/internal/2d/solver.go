package solver

import (
	"github.com/downflux/go-geometry/plane"
	"github.com/downflux/go-geometry/vector"
	"github.com/downflux/go-orca/internal/solver/internal/helper"
)

type S struct {
	knownOptimalMagnitude bool
}

func New(knownOptimalMagnitude bool) *S {
	return &S{knownOptimalMagnitude: knownOptimalMagnitude}
}

type maxSpeedAgent struct {
	helper.Agent
	knownOptimalMagnitude bool
}

// T ensures the underlying agent preferred velocity does not exceed the maximum
// speed of the agent.
func (a maxSpeedAgent) T() vector.V {
	if a.knownOptimalMagnitude || vector.SquaredMagnitude(a.Agent.T()) > a.Agent.S()*a.Agent.S() {
		return vector.Scale(a.Agent.S(), vector.Unit(a.Agent.T()))
	}
	return a.Agent.T()
}

func (r S) Solve(a helper.Agent, cs []plane.HP) (vector.V, bool) {
	msa := maxSpeedAgent{Agent: a, knownOptimalMagnitude: r.knownOptimalMagnitude}
	h := helper.New(msa, r.knownOptimalMagnitude)

	solution := msa.T()
	for _, c := range cs {
		// Checks if the existing solution satisfies the new constraint.
		// In our constraint implementation, we have chosen the "left"
		// side of the plane direction D() to be invalid, and so
		//
		//   D() x p > 0
		//
		// Implies p does not satisfy the constraint.
		//
		// N.B.: RVO2 chooses to orient the test vector towards the
		// plane; we are instead using the less confusing orientation of
		// pointing our test vector towards the solution p instead; this
		// causes us to reverse the inequality.
		//
		// N.B.: RVO2 chooses the "right" side of D() to be valid, and
		// therefore checks if the determinant is negative instead.
		if vector.Determinant(c.D(), vector.Sub(solution, c.P())) > 0 {
			if r, ok := h.Add(c); ok {
				solution = r
			} else {
				return vector.V{}, false
			}
		}
	}
	return solution, true
}
