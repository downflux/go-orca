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

	// sizeCache tracks the current size of the subtree to prevent recursive
	// calculations for every size query.
	sizeCache int
}

func (n *N) Leaf() bool { return n.size() <= 1 }

// size returns the number of meaningful nodes in the current subtree. A size of
// 0 or 1 indicates n is a leaf node.
func (n *N) size() int { return n.sizeCache }

func (n *N) setSize() {
	var s int

	if l := n.L(); l != nil {
		s += l.size()
	}
	if r := n.R(); r != nil {
		s += r.size()
	}

	if len(n.data) > 0 {
		s += 1
	}

	n.sizeCache = s
}

// Axis is the discriminant dimension for this tree node -- if the node is
// split on the X-axis, then all points left of this node in the XY-plane are in
// the left child, and all points on or right of this node are in the right
// child.
func (n *N) Axis() axis.Type { return axis.A(n.depth) }

// V is the point on the XY-plane to which this node is embedded. All data in
// this node are located at the same spacial coordinate, within a small margin
// of error.
func (n *N) V() vector.V { return n.v }

// Data is the data stored in this node.
func (n *N) Data() []point.P { return n.data }

func (n *N) Child(v vector.V, tolerance float64) *N {
	if vector.Within(n.V(), v, tolerance) {
		return nil
	}

	x := axis.X(v, n.Axis())
	nx := axis.X(n.V(), n.Axis())

	if x < nx {
		return n.L()
	}
	return n.R()
}

func (n *N) L() *N {
	if n.l == nil || n.l.size() == 0 {
		return nil
	}
	return n.l
}

func (n *N) R() *N {
	if n.r == nil || n.r.size() == 0 {
		return nil
	}
	return n.r
}

// Insert inserts a data point into the node. The point may be stored inside a
// child node.
func (n *N) Insert(p point.P, tolerance float64) {
	// The number of meaningful child nodes may increase after this
	// operation, so we need to ensure this cache is updated.
	defer n.setSize()

	if vector.Within(p.V(), n.V(), tolerance) {
		n.data = append(n.data, p)
		return
	}

	x := axis.X(p.V(), n.Axis())
	nx := axis.X(n.V(), n.Axis())

	if x < nx {
		if n.l == nil {
			n.l = &N{
				depth: n.depth + 1,
				v:     p.V(),
			}
		}
		n.l.Insert(p, tolerance)
		return
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

	if vector.Within(p.V(), n.V(), tolerance) {
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

	c := n.Child(p.V(), tolerance)

	return c != nil && c.Remove(p, tolerance)
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

// Points returns all data stored in the node and subnodes.
func Points(n *N) []point.P {
	ps := n.Data()
	if n.L() != nil {
		ps = append(ps, Points(n.L())...)
	}
	if n.R() != nil {
		ps = append(ps, Points(n.R())...)
	}
	return ps
}
