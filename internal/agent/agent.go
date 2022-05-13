package agent

import (
	"github.com/downflux/go-geometry/2d/hypersphere"
	"github.com/downflux/go-geometry/2d/vector"
)

type O struct {
	P vector.V
	V vector.V
	R float64
	S float64
	T vector.V
}

type A struct {
	c hypersphere.C
	o O
}

func New(o O) *A { return &A{o: o, c: *hypersphere.New(o.P, o.R)} }

func (a A) P() vector.V { return a.c.P() }
func (a A) R() float64  { return a.c.R() }

func (a A) V() vector.V { return a.o.V }
func (a A) T() vector.V { return a.o.T }
func (a A) S() float64  { return a.o.S }
