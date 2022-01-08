// Package agent defines a velocity obstacle object which is constructed from a
// line segment.
//
// The line segment obstacle is impermeable from either side.
package agent

import (
	"fmt"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/agent"
)

type VO struct {
	s segment.S
}

func New(s segment.S) *VO {
	if !s.Feasible() {
		panic(
			fmt.Sprintf(
				"cannot construct VO object, line segment %v is infeasible",
				s,
			),
		)
	}
	return &VO{
		s: s,
	}
}

func (vo VO) ORCA(a agent.A, tau float64) hyperplane.HP {
	return hyperplane.HP{}
}

func (vo VO) l(a agent.A) vector.V { return vector.V{} }
func (vo VO) r(a agent.A) vector.V { return vector.V{} }

// s generates a scaled line segment based on the lookahead time and the agent.
func s(s segment.S, a agent.A, tau float64) segment.S {
	return segment.New(
		line.New(
			vector.Scale(1/tau, vector.Sub(vo.s.L().P(), a.P())),
			vector.Scale(1/tau, vo.s.L().D()),
		),
		vo.s.Min(),
		vo.s.Max(),
	)
}
