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

	v2d "github.com/downflux/go-geometry/2d/vector"
)

type ORCA struct {
	agents []agent.RW
	t      *kd.T
}

type P struct {
	a agent.RW
}

func (p P) P() vector.V { return vector.V(p.a.P()) }

func New(agents []agent.RW) *ORCA {
	ps := make([]point.P, 0, len(agents))
	for _, a := range agents {
		ps = append(ps, P{a: a})
	}
	t, err := kd.New(ps)
	if err != nil {
		return nil
	}
	return &ORCA{
		agents: agents,
		t:      t,
	}
}

func (o *ORCA) Step(tau float64) error {
	// TODO(minkezhang): Refactor this outside of ORCA -- ensure ORCA is
	// state-invariant and leave the simulation details further up the
	// stack.
	o.t.Balance()

	vs := make([]v2d.V, 0, len(o.agents))

	// TODO(minkezhang): Make this parallel.
	for _, a := range o.agents {
		ps, err := kd.RadialFilter(
			o.t,
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
			return err
		}

		neighbors := make([]agent.RW, 0, len(ps))
		for _, p := range ps {
			neighbors = append(neighbors, p.(P).a)
		}

		cs := make([]constraint.C, 0, len(ps))
		for _, p := range ps {
			b, err := ball.New(a, p.(P).a, tau)
			if err != nil {
				return err
			}
			hp, err := b.ORCA()
			if err != nil {
				return err
			}
			cs = append(cs, constraint.C(hp))
		}
		vs = append(vs, solver.Solve(cs, a.T(), a.S()))
	}

	// TODO(minkezhang): Return a set of vectors intead and make this
	// operation state-invariant -- leave the state mutation to further up
	// the stack.
	for i, a := range o.agents {
		a.SetV(vs[i])
	}
	return nil
}
