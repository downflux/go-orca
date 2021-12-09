package orca

import (
	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/solver"
	"github.com/downflux/go-orca/internal/vo/ball"
)

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
}

func Step(t *kd.T, tau float64) ([]Mutation, error) {
	ps := kd.Data(t)
	agents := make([]agent.A, 0, len(ps))
	for _, p := range ps {
		agents = append(agents, p.(P).a)
	}

	vs := make([]Mutation, 0, len(agents))

	// TODO(minkezhang): Make this parallel.
	for _, a := range agents {
		ps, err := kd.RadialFilter(
			t,
			*hypersphere.New(
				vector.V(a.P()),
				// Verify this radius is sufficient for finding
				// all neighbors.
				2*a.S(),
			),
			// TODO(minkezhang): Compare pointers instead to check
			// for self-interaction.
			//
			// TODO(minkezhang): Add user filter function input --
			// user may want to e.g. specify that certain targets
			// should be considered for collision detection
			// (including walls), but not others. This allows us to
			// support e.g. unit squishing.
			func(p point.P) bool { return !vector.Within(p.P(), vector.V(a.P())) },
		)
		if err != nil {
			return nil, err
		}

		neighbors := make([]agent.A, 0, len(ps))
		for _, p := range ps {
			neighbors = append(neighbors, p.(P).a)
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
			A: a,
			V: vector.V(solver.Solve(cs, a.T(), a.S())),
		})
	}
	return vs, nil
}
