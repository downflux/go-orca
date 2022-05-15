// Package agent defines a velocity obstacle object which is constructed from
// two agents.
//
// This package is a wrapper around the deprecated ball package, which uses an
// older VO interface definition.
package agent

import (
	"fmt"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/geometry/ball"
	"github.com/downflux/go-orca/internal/vo/opt"
)

type VO struct {
	obstacle agent.A
}

func New(obstacle agent.A) *VO {
	return &VO{obstacle: obstacle}
}

func (vo VO) ORCA(o opt.O) hyperplane.HP {
	b, err := ball.New(
		ball.O{
			Obstacle: vo.obstacle,
			Agent:    o.Agent,
			Tau:      o.Tau,
		},
	)
	if err != nil {
		panic(fmt.Sprintf("cannot construct VO object: %v", err))
	}

	orca, err := b.ORCA()
	if err != nil {
		panic(fmt.Sprintf("cannot construct ORCA constraint: %v", err))
	}

	return orca
}
