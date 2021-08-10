package reference

import (
	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/vo"
)

var (
	_ vo.Agent = A{}
)

type O struct {
	P vector.V
	V vector.V
	R float64
	S float64
	G vector.V
}

type A struct {
	o O
}

func New(o O) *A { return &A{o: o} }

func (a A) P() vector.V { return a.o.P }
func (a A) V() vector.V { return a.o.V }
func (a A) G() vector.V { return a.o.G }
func (a A) R() float64  { return a.o.R }
func (a A) S() float64  { return a.o.S }
