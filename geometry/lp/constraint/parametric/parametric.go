package parametric

import (
	"github.com/downflux/go-geometry/plane"
	"github.com/downflux/go-geometry/vector"
	"github.com/downflux/orca/geometry/lp/constraint/standard"
)

type C struct {
	hp plane.HP
	c  standard.C
}

func New(hp plane.HP) *C {
	a := vector.Scale(-1, hp.N())
	return &C{
		hp: hp,
		c: *standard.New(
			[]float64{a.X(), a.Y()},
			vector.Dot(a, hp.P()),
		),
	}
}

func (c C) Dimension() int     { return 2 }
func (c C) A() []float64       { return c.c.A() }
func (c C) B() float64         { return c.c.B() }
func (c C) In(p vector.V) bool { return c.hp.In(p) }
