package agent

import (
	"fmt"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/external/snape/RVO2/vo/agent/cache"
)

type VO struct {
	obstacle agent.A
}

func New(obstacle agent.A) *VO {
	return &VO{
		obstacle: obstacle,
	}
}

func (vo VO) ORCA(a agent.A, tau float64) hyperplane.HP {
	hp, err := cache.New(vo.obstacle, a, tau).ORCA()
	if err != nil {
		panic(fmt.Sprintf("cannot create ORCA plane: %v", err))
	}
	return hp
}
