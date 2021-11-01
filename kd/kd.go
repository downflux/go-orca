// Package kd implements a 2D K-D tree with arbitrary data packing and duplicate
// data coordinate support.
//
// K-D trees are generally a cacheing layer representation of the local state --
// we do not expect to be making frequent mutations to this tree once
// constructed.
//
// The tree will have to be rebalanced as data points move in the simulation.
// For large numbers of points, and for a large number of queries, the time
// taken to build the tree will be offset by the speedup of subsequent reads.
package kd

import (
	"github.com/downflux/orca/geometry/circle"
	"github.com/downflux/orca/geometry/rectangle"
	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/kd/knn"
	"github.com/downflux/orca/kd/node"
	"github.com/downflux/orca/kd/point"
	"github.com/downflux/orca/kd/rangesearch"
)

// T is a K-D tree implementation.
type T struct {
	root      *node.N
	tolerance float64
}

func New(ps []point.P, tolerance float64) *T {
	return &T{
		root:      node.New(ps, 0, tolerance),
		tolerance: tolerance,
	}
}

// Balance rebalances the tree; note that in general, tree node mutations are
// expensive and messy, so much so that it's easier to just redo the tree from
// scratch.
func (t *T) Balance() { t.root = node.New(node.Points(t.root), 0, t.tolerance) }

// Insert adds a new data point into the tree.
//
// N.B.: This is not guaranteed to be a balanced insertion -- adding lots of
// points may unbalance the tree, causing queries to become much slower than in
// theory. If this function must be called a number of times, it's generally
// good practice to call Balance() in order to ensure optimal lookup times.
func (t *T) Insert(p point.P) { t.root.Insert(p, t.tolerance) }

// Remove deletes an existing data point from the tree. This function will
// delete the first matching point with the given coordinates.
//
// TODO(minkezhang): Update API to be of the form
//
//   Remove(v vector.V, check func(p P) bool)
//
// instead.
func (t *T) Remove(p point.P) bool { return t.root.Remove(p, t.tolerance) }

// Filter returns a set of data points in the given bounding box. Data points
// are added to the returned set if they fall inside the bounding box and passes
// the given filter function.
func Filter(t *T, r rectangle.R, f func(p point.P) bool) []point.P {
	var data []point.P
	for _, n := range rangesearch.Search(t.root, r) {
		for _, p := range n.Data() {
			if f(p) {
				data = append(data, p)
			}
		}
	}
	return data
}

// RadialFilter returns a set of data points in the given bounding circle. Data
// points are added to the returned set if they fall inside the bounding circle
// and passes the given filter function.
func RadialFilter(t *T, c circle.C, f func(p point.P) bool) []point.P {
	r := *rectangle.New(
		vector.Sub(c.P(), *vector.New(c.R(), c.R())),
		vector.Add(c.P(), *vector.New(c.R(), c.R())),
	)
	return Filter(t, r, func(p point.P) bool {
		return vector.SquaredMagnitude(vector.Sub(p.V(), c.P())) <= c.R()*c.R() && f(p)
	})
}

// KNN returns the k nearest neighbors of the given search coordinates.
//
// N.B.: KNN will return at max k neighbors; in the degenerate case that
// multiple data points reside at the same spacial coordinate, this function
// will arbitrarily return a subset of these to fulfill the k neighbors
// criteria.
func KNN(t *T, v vector.V, k int) []point.P {
	var data []point.P
	for _, n := range knn.KNN(t.root, v, k, t.tolerance) {
		if len(data) < k {
			data = append(data, n.Data()...)
		} else {
			break
		}
	}
	return data
}
