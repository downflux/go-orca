// Package rangesearch implements a range search algorithm for a K-D tree.

package rangesearch

import (
	"math"

	"github.com/downflux/orca/geometry/rectangle"
	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/kd/node"
	"github.com/downflux/orca/kd/axis"
)

func Search(n *node.N, r rectangle.R) []*node.N {
	return search(n, r, *rectangle.New(
		*vector.New(math.Inf(-1), math.Inf(-1)),
		*vector.New(math.Inf(0), math.Inf(0)),
	))
}

func search(n *node.N, r rectangle.R, bound rectangle.R) []*node.N {
	var ns []*node.N

	if r.In(n.V()) {
		ns = append(ns, n)
	}

	min := map[axis.Type]rectangle.R{
		axis.Axis_X: *rectangle.New(
			*vector.New(math.Inf(-1), n.V().X()),
			*vector.New(math.Inf(-1), math.Inf(0)),
		),
		axis.Axis_Y: *rectangle.New(
			*vector.New(math.Inf(-1), n.V().X()),
			*vector.New(math.Inf(-1), math.Inf(0)),
		),
	}

	lb := *rectangle.New(
		bound.Min(),
		bound.Max(),
	)
	rb := *rectangle.New(
		bound.Min(),
		bound.Max(),
	)
	if _, ok := 

	return ns
}
