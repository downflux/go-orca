package kd

import (
	"github.com/downflux/orca/kd/node"
	"github.com/downflux/orca/kd/point"
)

// T is a K-D tree implementation.
type T struct {
	root *node.N
}

func New(ps []point.P, tolerance float64) *T {
	return &T{
		root: node.New(ps, 0, tolerance),
	}
}

func (t *T) Insert(p point.P, tolerance float64)      { t.root.Insert(p, tolerance) }
func (t *T) Remove(p point.P, tolerance float64) bool { return t.root.Remove(p, tolerance) }

func Balance(t *T, tolerance float64) *T { return New(t.root.Data(), tolerance) }
