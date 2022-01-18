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
	"github.com/downflux/go-orca/internal/vo/agent/internal/ball"
)

type VO struct {
	a agent.A
}

func New(a agent.A) *VO {
	return &VO{a: a}
}

func (vo VO) ORCA(a agent.A, tau float64) hyperplane.HP {
	b, err := ball.New(vo.a, a, tau)
	if err != nil {
		panic(fmt.Sprintf("cannot construct VO object: %v", err))
	}

	orca, err := b.ORCA()
	if err != nil {
		panic(fmt.Sprintf("cannot construct ORCA constraint: %v", err))
	}

	return orca
}
