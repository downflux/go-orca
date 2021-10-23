// Package node implements a K-D tree node.
package node

import (
	"math"

	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/kd/axis"
	"github.com/downflux/orca/kd/point"
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
func (n *N) Insert(p point.P) {
	x := axis.X(p.V(), n.axis())
	nx := axis.X(n.v, n.axis())

	if x < nx {
		if n.l == nil {
			n.l = &N{
				depth: n.depth + 1,
				v:     p.V(),
			}
		}
		n.l.Insert(p)
	} else if x > nx {
		if n.r == nil {
			n.r = &N{
				depth: n.depth + 1,
				v:     p.V(),
			}
		}
		n.r.Insert(p)
	} else {
		n.data = append(n.data, p)
	}
}

// Remove deletes a data point from the node or child nodes. A returned value of
// false indicates the given point was not found.
func (n *N) Remove(p point.P) bool {
	x := axis.X(p.V(), n.axis())
	nx := axis.X(n.v, n.axis())

	if x < nx {
		return n.l != nil && n.l.Remove(p)
	} else if x > nx {
		return n.r != nil && n.r.Remove(p)
	}

	for i := range n.data {
		if p.Hash() == n.data[i].Hash() {
			n.data[len(n.data)-1], n.data[i] = nil, n.data[len(n.data)-1]
			return true
		}
	}
	return false
}

// New returns a new K-D tree node instance.
func New(data []point.P, depth int, tolerance float64) *N {
	if len(data) == 0 {
		return nil
	}

	point.Sort(data, axis.A(depth))

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

	return &N{
		l:     New(data[:l], depth+1, tolerance),
		r:     New(data[r+1:], depth+1, tolerance),
		depth: depth,

		v:    v,
		data: data[l : r+1],
	}
}
