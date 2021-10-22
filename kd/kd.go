// Package kd surfaces a K-D tree implementation with API tailored for DownFlux.
//
// Package wraps github.com/kyroy/kdtree.
package kd

import (
	"github.com/downflux/orca/geometry/vector"
	"github.com/kyroy/kdtree"
)

// P is an exported data point interface that data should implement.
type P interface {
	X() float64
	Y() float64
}

// point represents a wrapped data struct to be passed into the K-D tree
// implementation.
type point struct {
	data P
}

func (p point) Dimensions() int { return 2 }
func (p point) Dimension(i int) float64 {
	return []float64{
		p.data.X(),
		p.data.Y(),
	}[i]
}

// T is a wrapped K-D tree object for DownFlux.
type T struct {
	t *kdtree.KDTree
}

func New() *T { return &T{t: kdtree.New(nil)} }

// Balance rebalances the K-D tree by reconstructing the internal
// representation.
func (t *T) Balance() { t.t.Balance() }

// Insert adds a new data point into the K-D tree.
func (t *T) Insert(p P) { t.t.Insert(point{data: p}) }

// Remove deletes a data point from the K-D tree.
func (t *T) Remove(p P) { t.t.Remove(point{data: p}) }

// Within returns a list of points contained in a ball at the input coordinates
// with the given radius.
func (t *T) Within(v vector.V, r float64) []P {
	var ps []P
	for _, n := range t.t.RangeSearch([][2]float64{
		[2]float64{v.X() - r, v.X() + r},
		[2]float64{v.Y() - r, v.Y() + r},
	}) {
		p := n.(*point).data
		if vector.SquaredMagnitude(
			vector.Sub(v, *vector.New(p.X(), p.Y()))) <= r*r {
			ps = append(ps, p)
		}
	}
	return ps
}

// Neighbors returns the nearest N neighbors centered at the specified point.
func (t *T) Neighbors(p P, n int) []P {
	var ps []P
	for _, n := range t.t.KNN(point{data: p}, n) {
		ps = append(ps, n.(*point).data)
	}

	return ps
}
