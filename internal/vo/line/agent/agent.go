// Package agent implements the ORCA agent interface.
//
// This agent is used to represent the ends of the characteristic line segment
// of the line segment VO, which then is passed into the agent-agent VO.
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
func (a A) S() float64  { return 0 }
func (a A) R() float64  { return 0 }
func (a A) T() vector.V { return a.P() }
