package orca

import (
	"math"

	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/solver"
	"github.com/downflux/go-orca/internal/vo/ball"
)

// TODO(minkezhang): Export this to a seperate package, as this struct is
// necessary for multiple packages.
type P struct {
	a agent.A
}

func New(a agent.A) *P {
	return &P{a: a}
}

func (p P) P() vector.V { return vector.V(p.a.P()) }

type Mutation struct {
	A agent.A
	V vector.V

	// DEBUG
	CS []constraint.C
}

func agents(ps []point.P) []agent.A {
	agents := make([]agent.A, 0, len(ps))
	for _, p := range ps {
		agents = append(agents, p.(P).a)
	}

	return agents
}

func Step(t *kd.T, tau float64, f func(a agent.A) bool) ([]Mutation, error) {
	as := agents(kd.Data(t))
	vs := make([]Mutation, 0, len(as))

	// Experimental results indicate changing agent loop to parallel
	// execution will not significantly alter speeds for N ~ 1k.
	for _, a := range as {
		ps, err := kd.RadialFilter(
			t,
			*hypersphere.New(
				vector.V(a.P()),
				// TODO(minkezhang): Verify this radius is
				// sufficient for finding all neighbors.
				math.Max(0, 4*a.R()), // 100 * tau * a.S(), 4 * a.R()),
			),
			// TODO(minkezhang): Check for interface equality
			// instead of coordinate equality, via adding an
			// Agent.Equal function.
			//
			// This technically may introduce a bug when multiple
			// points are extremely close together.
			func(p point.P) bool {
				return !vector.Within(
					p.P(),
					vector.V(a.P()),
				) && f(p.(P).a)
			},
		)
		// ps, err := kd.KNN(t, vector.V(a.P()), 0)  // DEBUG
		if err != nil {
			return nil, err
		}

		neighbors := make([]agent.A, 0, len(ps))
		for _, p := range ps {
			// if !vector.Within(p.P(), vector.V(a.P())) && f(p.(P).a) {  // DEBUG
			neighbors = append(neighbors, p.(P).a)
			// }
		}

		cs := make([]constraint.C, 0, len(ps))
		for _, p := range ps {
			b, err := ball.New(a, p.(P).a, tau)
			if err != nil {
				return nil, err
			}
			hp, err := b.ORCA()
			if err != nil {
				return nil, err
			}
			cs = append(cs, constraint.C(hp))
		}

		vs = append(vs, Mutation{
			A:  a,
			V:  vector.V(solver.Solve(cs, a.T(), a.S())),
			CS: cs,
		})
	}

	return vs, nil
}
