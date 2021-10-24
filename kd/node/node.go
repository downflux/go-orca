// Package node implements a K-D tree node.
package node

import (
	"math"

	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/kd/axis"
	"github.com/downflux/orca/kd/point"
	"github.com/downflux/orca/kd/point/sorter"
)

// N represents a K-D tree node. Child nodes are sorted along the same axis,
// with coordinates in the left node preceeding right node coordinates (along
// the given axis).
type N struct {
	l *N
	r *N

	// depth represents the current level of the tree; the root node has a
	// depth of 0.
	depth int

	// v represents the coordinate of all data stored in this node.
	v vector.V

	// data is a list of data points stored in this node. All data here are
	// located at the same spacial coordinate.
	data []point.P

	// sizeCache keeps a count of the number of meaningful nodes in the current subtree.
	// A size of 0 or 1 indicates this is a leaf node.
	sizeCache int
}

func (n *N) leaf() bool { return n.size() <= 1 }
func (n *N) size() int {
	if n == nil {
		return 0
	}
	return n.sizeCache
}

func (n *N) setSize() {
	s := n.l.size() + n.r.size()
	if len(n.data) > 0 {
		s += 1
	}
	n.sizeCache = s
}

func (n *N) axis() axis.Type { return axis.A(n.depth) }

// Data returns all data contained in the current and child nodes.
func (n *N) Data() []point.P {
	if n == nil {
		return nil
	}

	var data []point.P

	for _, ps := range [][]point.P{n.l.Data(), n.data, n.r.Data()} {
		data = append(data, ps...)
	}

	return data
}

// Insert inserts a data point into the node. The point may be stored inside a
// child node.
func (n *N) Insert(p point.P, tolerance float64) {
	// The number of meaningful child nodes may increase after this
	// operation, so we need to ensure this cache is updated.
	defer n.setSize()

	if vector.Within(p.V(), n.v, tolerance) {
		n.data = append(n.data, p)
		return
	}

	x := axis.X(p.V(), n.axis())
	nx := axis.X(n.v, n.axis())

	if x < nx {
		if n.l == nil {
			n.l = &N{
				depth: n.depth + 1,
				v:     p.V(),
			}
		}
		n.l.Insert(p, tolerance)
	}

	if n.r == nil {
		n.r = &N{
			depth: n.depth + 1,
			v:     p.V(),
		}
	}
	n.r.Insert(p, tolerance)
}

// Remove deletes a data point from the node or child nodes. A returned value of
// false indicates the given point was not found.
//
// N.B.: Remove does not actually delete the underlying k-d tree node. Manually
// removing and shifting the nodes is a non-trivial task. We generally expect
// k-d trees to be relatively stable once created, and that insert and remove
// operations are kept at a minimum.
func (n *N) Remove(p point.P, tolerance float64) bool {
	// The number of meaningful child nodes may decrease after this
	// operation, so we need to ensure this cache is updated.
	defer n.setSize()

	if vector.Within(p.V(), n.v, tolerance) {
		for i := range n.data {
			if p.Equal(n.data[i]) {
				// Remove the i-th element and set the data to
				// be a shortened slice.
				//
				// N.B.: we can mutate the data slice in this
				// manner only because we are guaranteed to
				// return from the function immediately after,
				// skipping any subsequent iterations.
				n.data[i], n.data[len(n.data)-1] = n.data[len(n.data)-1], nil
				n.data = n.data[:len(n.data)-1]
				return true
			}
		}
	}

	x := axis.X(p.V(), n.axis())
	nx := axis.X(n.v, n.axis())

	if x < nx {
		return n.l != nil && n.l.Remove(p, tolerance)
	}

	return n.r != nil && n.r.Remove(p, tolerance)
}

// New returns a new K-D tree node instance.
func New(data []point.P, depth int, tolerance float64) *N {
	if len(data) == 0 {
		return nil
	}

	// Sort is not stable -- the order may be shuffled, meaning that while
	// the axis coordinates are in order, the complement dimension is not.
	//
	// That is, give we are sorting on the x-axis,
	//
	//   [(1, 3), (1, 1)]
	//
	// is a valid ordering.
	sorter.Sort(data, axis.A(depth))

	m := len(data) / 2
	v := data[m].V()

	// Find adjacent elements in the sorted list that have the same
	// coordinates as the median, as they all should be in the same node.
	var l int
	var r int
	for i := m; i >= 0 && vector.Within(v, data[i].V(), tolerance); i-- {
		l = i
	}
	for i := m; i < len(data) && vector.Within(v, data[i].V(), tolerance); i++ {
		r = i
	}

	l = int(math.Max(0, float64(l)))
	r = int(math.Min(float64(len(data)-1), float64(r)))

	n := &N{
		l:     New(data[:l], depth+1, tolerance),
		r:     New(data[r+1:], depth+1, tolerance),
		depth: depth,

		v:    v,
		data: data[l : r+1],
	}
	n.setSize()

	return n
}
