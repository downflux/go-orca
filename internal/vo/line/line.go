// Package line defines a velocity obstacle object which is constructed from a
// line segment.
//
// The line segment obstacle is impermeable from either side.
package line

import (
	"fmt"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/vo/line/cache"
	"github.com/downflux/go-orca/internal/vo/opt"
)

type VO struct {
	s segment.S
	v vector.V
}

func New(s segment.S, v vector.V) *VO {
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
		v: v,
	}
}

func (vo VO) ORCA(o opt.O) hyperplane.HP {
	return cache.New(vo.s, vo.v, o.Agent, o.Tau).ORCA()
}
