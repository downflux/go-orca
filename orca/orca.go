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

// O is an options struct passed into the Step function.
type O struct {
	// T is a K-D tree containing all agents.
	T *kd.T

	// Tau is the lookahead time -- Step will avoid agent velocities which
	// will lead to collisions within this time frame. More discussion on a
	// sensible value for this variable can be found in
	// /internal/vo/ball/ball.go.
	Tau float64

	// F is a function which is used during neighbor searching to filter out
	// agents for which collisions are allowed. This is useful for e.g. when
	// we want to support unit squishing.
	F func(a agent.A) bool
}

// Step calculates new velocities for a collection of agents such that they will
// avoid collitions within the specified input duration tau.
//
// TODO(minkezhang): Add support for immovable agents, e.g. agents that have
// reached their destination or are on an enemy team. This may take the form of
// adding an A.Immovable() bool to the interface.
//
// TODO(minkezhang): Brainstorm ways to introduce a linear "agent", i.e. wall.
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
				// N.B.: RVO2 passes in a global state for this
				// radius; see
				// https://github.com/snape/RVO2/blob/a92e8cc858ab1884ee5de5eb3bc4a07f490d247a/src/Agent.cpp#L50
				// for more information.
				o.Tau*a.S()+2*a.R(),
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
