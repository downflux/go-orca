package orca

import (
	"fmt"
	"math"

	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/geometry/2d/constraint"
	"github.com/downflux/go-orca/internal/solver"
	"github.com/downflux/go-orca/internal/vo/agent/opt"
	"github.com/downflux/go-orca/internal/vo/wall"
	"github.com/downflux/go-orca/region"

	c2d "github.com/downflux/go-geometry/2d/constraint"
	v2d "github.com/downflux/go-geometry/2d/vector"
	voagent "github.com/downflux/go-orca/internal/vo/agent"
)

// Mutation pairs an agent with a velocity change calculated by ORCA.
type Mutation struct {
	A agent.A
	V v2d.V
}

type P interface {
	point.P
	A() agent.A
}

func agents[T P](ps []T) []agent.A {
	agents := make([]agent.A, 0, len(ps))
	for _, p := range ps {
		agents = append(agents, p.A())
	}

	return agents
}

// O is an options struct passed into the Step function.
type O[T P] struct {
	// T is a K-D tree containing all agents.
	T *kd.KD[T]

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

	// R is a list of map regions.
	R []region.R
}

type result struct {
	m   Mutation
	err error
}

func RadialFilter[T P](t *kd.KD[T], c hypersphere.C, f func(p P) bool) []T {
	offset := vector.M(make([]float64, c.P().Dimension()))
	for i := vector.D(0); i < c.P().Dimension(); i++ {
		offset.SetX(i, c.R())
	}

	r := *hyperrectangle.New(
		vector.Sub(c.P(), offset.V()),
		vector.Add(c.P(), offset.V()),
	)
	return kd.RangeSearch(t, r, func(p T) bool {
		return vector.SquaredMagnitude(vector.Sub(p.P(), c.P())) <= c.R()*c.R() && f(p)
	})
}

// step calculates the ORCA velocity for a single agent.
func step[T P](a agent.A, t *kd.KD[T], rs []region.R, f func(a agent.A) bool, tau float64) (Mutation, error) {
	ps := RadialFilter(
		t,
		// N.B.: RVO2 passes in a global state for this
		// radius; see
		// https://github.com/snape/RVO2/blob/a92e8cc858ab1884ee5de5eb3bc4a07f490d247a/src/Agent.cpp#L50
		// for more information.
		*hypersphere.New(
			vector.V(a.P()),
			tau*a.S()+2*a.R(),
		),
		// TODO(minkezhang): Check for interface equality
		// instead of coordinate equality, via adding an
		// Agent.Equal function.
		//
		// This technically may introduce a bug when multiple
		// points are extremely close together.
		func(p P) bool {
			return !vector.Within(p.P(), vector.V(a.P())) && f(p.(P).A())
		},
	)

	cs := make([]constraint.C, 0, len(ps))
	for _, r := range rs {
		// TODO(minkezhang): Support multi-segment region ORCA.
		if len(r.R()) != 1 {
			panic("UnimplementedError: cannot construct ORCA line for a region not of cardinality 1")
		}
		// TODO(minkezhang): Add to immutable constraints instead, as
		// lines are immovable.
		cs = append(
			cs,
			*constraint.New(
				c2d.C(wall.New(r.R()[0]).ORCA(a, tau)),
				false,
			),
		)
	}

	for _, p := range ps {
		cs = append(
			cs,
			*constraint.New(
				c2d.C(
					voagent.New(
						p.A(),
						opt.O{
							Weight: opt.WeightEqual,
							VOpt:   opt.VOptV,
						},
					).ORCA(a, tau),
				),
				true,
			),
		)
	}

	return Mutation{
		A: a,
		// Find a new velocity for an agent which minimizes the
		// difference to the velocity a.T() which satisifies all
		// constraints.
		//
		// This optimization velocity may be adjusted, per van de Berg
		// et al. (2011), section 5.2; however, setting this velocity to
		// a.V() does not seem very convincing -- agents tend to stop
		// drifting towards the target in packed conditions.
		V: solver.Solve(cs, a.T(), a.S()),
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
func Step[T P](o O[T]) ([]Mutation, error) {
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
				mutation, err := step(a, o.T, o.R, o.F, o.Tau)
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
