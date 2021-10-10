package kd

import (
	"github.com/kyroy/kdtree/kdtree"
	"github.com/kyroy/kdtree/points/point2d"
	"github.com/downflux/orca/vector/vector"
)

type P interface {
}

type T interface {
	Insert(p P)
	Remove(p P)
	Intersect(t T) []P
	Within(v vector.V, r float64) []P
	Near(p P, n int) []P
}

type p struct {
	v vector.V
}

func (p p) Dimensions() int { return 2 }
func (p p) Dimension(i int) {
	if i == 0 { return p.v.X() }
	return p.v.Y()
}

type T struct {
	t kdtree.Tree
}

func New() *T { return &T }
func Balance(t *T) { return t.Balance() }

func (t *T) Insert(p p) { return t.t.Insert(p) }
func (t *T) Remove(p p) { return t.t.Remove(p) }
func (t *T) Intersect(r []vector.V) []p { ... }

