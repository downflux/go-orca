// Package kd provides a shim to lift and downcast caller K-D tree instances to
// the correct type to be consumed by ORCA.
//
// The alternative is using generics to enforce types, but after
// experimentation, this syntax is very unwieldy.
package kd

import (
	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
)

// P defines the data that is stored in the ORCA K-D tree. Callers to ORCA need
// to implement their K-D trees with this point interface instead of the base
// point.P interface.
type P interface {
	point.P
	A() agent.A
}

// T is the ORCA K-D tree definition.
type T kd.T

// Lift ensures the base K-D tree is explicitly casted as an ORCA K-D tree
// instead.
func Lift(t *kd.T) *T {
	lt := T(*t)
	return &lt
}

// Downcast transforms the ORCA K-D tree back into the base K-D tree.
//
// Note that this function does not normally need to be called -- the ORCA
// caller should have separately kept track of the base K-D tree, and only call
// Lift to pass the tree into orca.Step. The underlying tree data has not
// changed.
func Downcast(t *T) *kd.T {
	dt := kd.T(*t)
	return &dt
}

func (t *T) Balance() {
	kdt := kd.T(*t)
	(&kdt).Balance()
}

func Filter(t *T, r hyperrectangle.R, f func(datum P) bool) ([]P, error) {
	ps, err := kd.Filter(
		Downcast(t),
		r,
		func(datum point.P) bool { return f(datum.(P)) },
	)
	if err != nil {
		return nil, err
	}

	data := make([]P, 0, len(ps))
	for _, p := range ps {
		data = append(data, p.(P))
	}

	return data, nil
}

func RadialFilter(t *T, c hypersphere.C, f func(datum P) bool) ([]P, error) {
	ps, err := kd.RadialFilter(
		Downcast(t),
		c,
		func(datum point.P) bool { return f(datum.(P)) },
	)
	if err != nil {
		return nil, err
	}

	data := make([]P, 0, len(ps))
	for _, p := range ps {
		data = append(data, p.(P))
	}
	return data, nil
}

func KNN(t *T, position vector.V, k int) ([]P, error) {
	ps, err := kd.KNN(Downcast(t), position, k)
	if err != nil {
		return nil, err
	}

	data := make([]P, 0, len(ps))
	for _, p := range ps {
		data = append(data, p.(P))
	}
	return data, nil
}

func Data(t *T) []P {
	ps := kd.Data(Downcast(t))

	data := make([]P, 0, len(ps))
	for _, p := range ps {
		data = append(data, p.(P))
	}

	return data
}
