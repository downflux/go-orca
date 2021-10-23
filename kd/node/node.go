package node

import (
	"math"

	"github.com/downflux/orca/kd/axis"
	"github.com/downflux/orca/kd/point"
	"github.com/downflux/orca/geometry/vector"
)

type N struct {
	l *N
	r *N

	depth int

	v vector.V
	data []point.P
}

func (n *N) axis() axis.Type { return axis.A(n.depth) }

func (n *N) Data() []point.P {
	if n == nil { return nil }

	var data []point.P

	for _, ps := range [][]point.P{ n.l.Data(), n.data, n.r.Data() } {
		data = append(data, ps...)
	}

	return data
}

func (n *N) Insert(p point.P) {
	x := axis.X(p.V(), n.axis())
	nx := axis.X(n.v, n.axis())

	if x < nx {
		if n.l == nil {
			n.l = &N{
				depth: n.depth + 1,
				v: p.V(),
			}
		}
		n.l.Insert(p)
	} else if x > nx {
		if n.r == nil {
			n.r = &N{
				depth: n.depth + 1,
				v: p.V(),
			}
		}
		n.r.Insert(p)
	} else {
		n.data = append(n.data, p)
	}
}

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
			n.data[len(n.data) - 1], n.data[i] = nil, n.data[len(n.data) - 1]
			return true
		}
	}
	return false
}

func New(data []point.P, depth int) *N {
	if len(data) == 0 { return nil }

	point.Sort(data, axis.A(depth))

	m := len(data) / 2
	v := data[m].V()

	l := m
	r := m

	for l := m; m >= 0 && v == data[l].V(); l-- {}
	l = int(math.Max(0, float64(l)))

	for r := m; m < len(data) && v == data[r].V(); r++ {}
	r = int(math.Min(float64(len(data)), float64(r)))

	return &N{
		l: New(data[:l], depth + 1),
		r: New(data[r + 1:], depth + 1),
		depth: depth,

		v: v,
		data: data[l:r + 1],
	}
}
