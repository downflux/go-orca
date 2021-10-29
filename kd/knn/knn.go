// Package knn implements k-nearest neighbors search algorithm on a K-D tree node.
package knn

import (
	"math"

	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/kd/axis"
	"github.com/downflux/orca/kd/node"
	"github.com/downflux/orca/kd/point"
)

// queue generates a list of nodes to the root, starting from a leaf node,
// with a node guaranteed to contain the input coordinates.
//
// N.B.: We do not stop the recursion if we reach a node with matching
// coordinates; this is necessary for finding multiple closest neighbors, as we
// care about points in the tree which do not have to coincide with the point
// coordinates.
func queue(n *node.N, v vector.V, tolerance float64) []*node.N {
	if n == nil {
		return nil
	}

	if n.Leaf() {
		return []*node.N{n}
	}

	x := axis.X(v, n.Axis())
	nx := axis.X(n.V(), n.Axis())

	if x < nx {
		return append(queue(n.L(), v, tolerance), n)
	}
	return append(queue(n.R(), v, tolerance), n)
}

// TODO(minkezhang): Replace with KNN instead.
func NNS(n *node.N, v vector.V, tolerance float64) ([]point.P, float64) {
	if n == nil {
		return nil, math.Inf(0)
	}

	q := queue(n, v, tolerance)

	// TODO(minkezhang): Replace with PQ for KNN instead.
	var data []point.P
	dist := math.Inf(0)

	for _, n := range q {
		var d float64
		if d = vector.Magnitude(vector.Sub(v, n.V())); d < dist {
			dist = d
			data = n.Data()
		}

		x := axis.X(v, n.Axis())
		nx := axis.X(n.V(), n.Axis())

		// The minimal distance so far exceeds the current node split
		// plane -- we need to expand into the child nodes.
		if d-math.Abs(nx-x) > 0 {
			if x < nx {
				if newData, newDist := NNS(n.L(), v, tolerance); newDist < dist {
					data, dist = newData, newDist
				}
			} else {
				if newData, newDist := NNS(n.R(), v, tolerance); newDist < dist {
					data, dist = newData, newDist
				}
			}
		}
	}

	return data, dist
}
