package orca

import (
	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/orca"
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
}

func agents(ps []point.P) []agent.A {
	agents := make([]agent.A, 0, len(ps))
	for _, p := range ps {
		agents = append(agents, p.(P).a)
	}

	return agents
}

type O struct {
	T   *kd.T
	Tau float64
	F   func(a agent.A) bool
	R   func(tau float64) float64
}

// Step calculates new velocities for a collection of agents such that they will
// avoid collitions within the specified input duration tau.
//
// TODO(minkezhang): Add support for immovable agents, e.g. agents that have
// reached their destination or are on an enemy team. This may take the form of
// adding an A.Immovable() bool to the interface.
//
// TODO(minkezhang): Brainstorm ways to introduce a linear "agent", i.e. wall.
//
// TODO(minkezhang): Add more documentation on what tau represents, i.e. how far
// into the future we are looking to avoid obstacles, and is independent of the
// calling rate, though tau should be "larger" (in distance) than the calling
// rate (i.e. the maximum distance agents may travel in between calls).
func Step(o O) ([]Mutation, error) {
	as := agents(kd.Data(o.T))
	vs := make([]Mutation, 0, len(as))

	// Experimental results indicate changing agent loop to parallel
	// execution will not significantly alter speeds for N ~ 1k.
	for _, a := range as {
		ps, err := kd.RadialFilter(
			o.T,
			*hypersphere.New(
				vector.V(a.P()),
				// TODO(minkezhang): Move tau manipulation to
				// caller instead; this value should be the same
				// as the vaule passed into ball.New().
				//
				// TODO(minkezhang): Change BenchmarkStep() to
				// supply the modified tau value instead, as
				// this influences kd.RadialFilter, and thus
				// influences the runtime.
				//
				// TODO(minkezhang): Verify this radius is
				// sufficient for finding all neighbors.
				orca.R(a, o.Tau),
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
				) && o.F(p.(P).a)
			},
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
			b, err := ball.New(a, p.(P).a, o.Tau)
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
