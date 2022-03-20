package agent

import (
	"github.com/downflux/go-geometry/2d/vector"
)

type A struct {
	p vector.V
	v vector.V
}

func New(p vector.V, v vector.V) *A {
	return &A{
		p: p,
		v: v,
	}
}

func (a A) P() vector.V { return a.p }
func (a A) V() vector.V { return a.v }

// TODO(minkezhang): add VO.s max speed property
func (a A) S() float64  { return 0 }
func (a A) R() float64  { return 0 }
func (a A) T() vector.V { return a.P() }
