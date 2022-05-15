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
	"github.com/downflux/go-orca/internal/vo/agent/opt"
)

type VO struct {
	obstacle agent.A

	weight opt.Weight
	vopt   opt.VOpt
}

func New(obstacle agent.A, o opt.O) *VO {
	if err := opt.Validate(o); err != nil {
		panic(fmt.Sprintf("could not construct velocity obstacle: %v", err))
	}

	return &VO{
		obstacle: obstacle,
		weight:   o.Weight,
		vopt:     o.VOpt,
	}
}

func (vo VO) ORCA(agent agent.A, tau float64) hyperplane.HP {
	b, err := ball.New(
		ball.O{
			Obstacle: vo.obstacle,
			Agent:    agent,
			Tau:      tau,
			Weight:   vo.weight,
			VOpt:     vo.vopt,
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
