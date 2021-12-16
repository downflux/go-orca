// Package kd provides a shim to lift and downcast caller K-D tree instances to
// the correct type to be consumed by ORCA.
package kd

import (
	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/downflux/go-geometry/nd/hypersphere"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
)

type P interface {
	point.P
	A() agent.A
}

type T kd.T

func (t *T) Balance() {
	kdt := kd.T(*t)
	(&kdt).Balance()
}

func Filter(t *T, r hyperrectangle.R, f func(datum P) bool) ([]P, error) {
	kdt := kd.T(*t)
	ps, err := kd.Filter(
		&kdt,
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
	kdt := kd.T(*t)
	ps, err := kd.RadialFilter(
		&kdt,
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
	kdt := kd.T(*t)
	ps, err := kd.KNN(&kdt, position, k)
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
	kdt := kd.T(*t)
	ps := kd.Data(&kdt)

	data := make([]P, 0, len(ps))
	for _, p := range ps {
		data = append(data, p.(P))
	}

	return data
}
