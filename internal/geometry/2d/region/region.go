package region

import (
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
)

type R []segment.S

func New(ss []segment.S) *R {
	r := R(ss)
	if len(r) > 2 {
		for i, s := range r {
			p := r.prev(i)
			if !vector.Within(p.L().L(p.TMax()), s.L().L(s.TMin())) {
				panic("cannot construct a non-closed region")
			}
		}
	}
	return &r
}

func (r R) prev(i int) segment.S {
	if i == 0 {
		return r[len(r)-1]
	}
	return r[i-1]
}

// Concave returns if the specified vertex turns into the infeasible side of the
// region. We define the angle at this vertex to be greater than π.
// An i-th vertex is defined as the starting point of the i-th segment.
func (r R) Concave(vertex int) bool {
	p := r.prev(vertex)
	s := r[vertex]
	return hyperplane.New(p.L().P(), p.L().N()).In(s.L().L(s.TMax()))
}
