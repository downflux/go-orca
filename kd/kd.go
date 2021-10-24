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

func (t *T) Balance(tolerance float64)                { t.root = node.New(t.root.Data(), 0, tolerance) }
func (t *T) Insert(p point.P, tolerance float64)      { t.root.Insert(p, tolerance) }
func (t *T) Remove(p point.P, tolerance float64) bool { return t.root.Remove(p, tolerance) }

func Find(t *T, f func(p point.P) bool) []point.P { return nil }

func Neighbors(t *T, n int) []point.P {
	i := 0
	return Find(t, func(p point.P) bool {
		if i < n {
			i += 1
			return true
		}
		return false
	})
}
