package orca

import (
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/orca"

	kdx "github.com/downflux/go-kd/x/kd"
)

// P defines an interface which is passed into the generic version of the K-D
// tree. This is to ensure that we can make certain assumptions about the
// structure of the K-D tree data point when we read from the tree during the
// orca.Step function.
type P[T agent.A] interface {
	point.P

	// A returns the agent.A interface instead of T here. This is to conform
	// to the underlying (un-generic) version of the orca.Step function.
	//
	// TODO(minkezhang): Determine if we want to modify the interface to
	// return type T, but transform this point into an internal-only
	// interface which returns the agent.A interface instead.
	A() agent.A
}

type Mutation[T agent.A] struct {
	A T
	V vector.V
}

type O[T agent.A] struct {
	T        *kdx.T[P[T]]
	Tau      float64
	F        func(a T) bool
	PoolSize int
}

func Step[T agent.A](o O[T]) ([]Mutation[T], error) {
	t := kd.T(*o.T)

	m, err := orca.Step(
		orca.O{
			T:        &t,
			Tau:      o.Tau,
			F:        func(a agent.A) bool { return o.F(a.(T)) },
			PoolSize: o.PoolSize,
		},
	)
	if err != nil {
		return nil, err
	}

	gm := make([]Mutation[T], 0, len(m))
	for _, m := range m {
		gm = append(gm, Mutation[T]{
			A: m.A.(T),
			V: m.V,
		})
	}
	return gm, nil
}
