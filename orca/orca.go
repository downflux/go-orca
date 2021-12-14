package orca

import (
	"fmt"
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

// P defines a point used in the K-D tree.
type P interface {
	A() agent.A
	P() vector.V
}

// Mutation pairs an agent with a velocity change calculated by ORCA.
type Mutation struct {
	A agent.A
	V vector.V
}

func agents(ps []point.P) []agent.A {
	agents := make([]agent.A, 0, len(ps))
	for _, p := range ps {
		agents = append(agents, p.(P).A())
	}

	return agents
}

// O is an options struct passed into the Step function.
type O struct {
	// T is a K-D tree containing all agents.
	//
	// TODO(minkezhang): Specify this as a generic T[P].
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

	// PoolSize is the number of workers that will process the the agents in
	// parallel. We want this to be on the order of magnitude of the number
	// of cores on the system for fastest processing times.
	PoolSize int
}

type result struct {
	m   Mutation
	err error
}

// step calculates the ORCA velocity for a single agent.
func step(a agent.A, t *kd.T, f func(a agent.A) bool, tau float64) (Mutation, error) {
	ps, err := kd.RadialFilter(
		t,
		*hypersphere.New(
			vector.V(a.P()),
			// N.B.: RVO2 passes in a global state for this
			// radius; see
			// https://github.com/snape/RVO2/blob/a92e8cc858ab1884ee5de5eb3bc4a07f490d247a/src/Agent.cpp#L50
			// for more information.
			tau*a.S()+2*a.R(),
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
			) && f(p.(P).A())
		},
	)
	if err != nil {
		return Mutation{}, err
	}

	cs := make([]constraint.C, 0, len(ps))
	for _, p := range ps {
		b, err := ball.New(a, p.(P).A(), tau)
		if err != nil {
			return Mutation{}, err
		}
		hp, err := b.ORCA()
		if err != nil {
			return Mutation{}, err
		}
		cs = append(cs, constraint.C(hp))
	}

	return Mutation{
		A: a,
		V: vector.V(solver.Solve(cs, a.T(), a.S())),
	}, nil
}

// Step calculates new velocities for a collection of agents such that they will
// avoid collitions within the specified input duration tau.
//
// Step parallelizes ORCA calculations. Note that while calling Step, the input
// K-D tree and agents must not be mutated.
//
// TODO(minkezhang): Add support for immovable agents, e.g. agents that have
// reached their destination or are on an enemy team. This may take the form of
// adding an A.Immovable() bool to the interface.
//
// TODO(minkezhang): Brainstorm ways to introduce a linear "agent", i.e. wall.
func Step(o O) ([]Mutation, error) {
	if o.PoolSize == 0 {
		panic("must specify Step with non-zero pool size")
	}
	as := agents(kd.Data(o.T))

	// Ensure channel reads aren't blocking due to dispatch or fold
	// operation.
	ach := make(chan agent.A, 8*o.PoolSize)
	rch := make(chan result, 8*o.PoolSize)

	go func(ch chan<- agent.A) {
		defer close(ch)
		for _, a := range as {
			ch <- a
		}
	}(ach)

	n := int(
		math.Min(
			float64(len(as)),
			float64(o.PoolSize)),
	)
	// Start up a number of workers to find the iterative velocity in
	// parallel.
	for i := 0; i < n; i++ {
		go func(jobs <-chan agent.A, results chan<- result) {
			for a := range jobs {
				mutation, err := step(a, o.T, o.F, o.Tau)
				results <- result{
					m:   mutation,
					err: err,
				}
			}
		}(ach, rch)
	}

	mutations := make([]Mutation, 0, len(as))
	var errors []error

	for i := 0; i < len(as); i++ {
		r := <-rch
		if r.err != nil {
			errors = append(errors, r.err)
		} else {
			mutations = append(mutations, r.m)
		}
	}

	if len(errors) > 0 {
		return nil, fmt.Errorf("could not generate ORCA simulation: %v", errors)
	}
	return mutations, nil
}
