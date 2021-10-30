// Package knn implements k-nearest neighbors search algorithm on a K-D tree node.
package knn

import (
	"fmt"
	"math"

	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/kd/axis"
	"github.com/downflux/orca/kd/node"
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

	// Note that we are bypassing the v == n.V() stop condition check -- we
	// are always continuing to the leaf node.

	x := axis.X(v, n.Axis())
	nx := axis.X(n.V(), n.Axis())

	if x < nx {
		return append(queue(n.L(), v, tolerance), n)
	}
	return append(queue(n.R(), v, tolerance), n)
}

func KNN(n *node.N, v vector.V, k int, tolerance float64) []*node.N {
	ns, _ := knn(n, v, k, tolerance)
	return ns
}

func knn(n *node.N, v vector.V, k int, tolerance float64) ([]*node.N, float64) {
	if n == nil {
		return nil, math.Inf(0)
	}
	fmt.Printf("DEBUG: knn(n == %v)\n", n.V())

	q := queue(n, v, tolerance)

	// TODO(minkezhang): Replace with PQ for KNN instead.
	var data []*node.N
	dist := math.Inf(0)

	for _, n := range q {
		if d := vector.Magnitude(vector.Sub(v, n.V())); d < dist {
			fmt.Printf("DEBUG: knn: d == %v < [min]dist == %v\n", vector.Magnitude(vector.Sub(v, n.V())), dist)
			dist = d
			data = []*node.N{n}
		}

		x := axis.X(v, n.Axis())
		nx := axis.X(n.V(), n.Axis())

		// The minimal distance so far exceeds the current node split
		// plane -- we need to expand into the child nodes.
		if dist > math.Abs(nx-x) {
			fmt.Printf("DEBUG: knn distance to node plane == %v < [min]dist == %v; x == %v, nx == %v\n", math.Abs(nx-x), dist, x, nx)

			if c := n.Child(v, tolerance); c != nil {
				if newData, newDist := knn(c, v, k, tolerance); newDist < dist {
					data, dist = newData, newDist
				}
			}
		}
	}

	return data, dist
}
