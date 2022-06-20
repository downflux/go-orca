// Package wall defines a velocity obstacle object which is constructed from a
// line segment.
//
// The line segment obstacle is impermeable from either side.
package wall

import (
	"fmt"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/internal/vo/wall/cache"
)

type VO struct {
	obstacle segment.S
}

func New(obstacle segment.S) *VO {
	if !obstacle.Feasible() {
		panic(
			fmt.Sprintf(
				"cannot construct VO object, line segment %v is infeasible",
				obstacle,
			),
		)
	}

	return &VO{obstacle: obstacle}
}

func (vo VO) ORCA(a agent.A, tau float64) []hyperplane.HP {
	return []hyperplane.HP{cache.New(vo.obstacle, a, tau).ORCA()}
}
