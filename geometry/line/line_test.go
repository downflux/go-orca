package line

import (
	"math"
	"testing"

	"github.com/downflux/orca/geometry/vector"
)

const (
	tolerance = 1e-10
)

func TestDistance(t *testing.T) {
	testConfigs := []struct {
		name string
		l    L
		p    vector.V
		want float64
	}{
		{
			name: "Trivial",
			l:    L{p: *vector.New(0, 0), d: *vector.New(0, 1)},
			p:    *vector.New(0, 0),
			want: 0,
		},
		{
			name: "SimpleUnitDirection",
			l:    L{p: *vector.New(0, 0), d: *vector.New(0, 1)},
			p:    *vector.New(1, 1),
			want: 1,
		},
		{
			name: "SimpleLargeDirection",
			l:    L{p: *vector.New(0, 0), d: *vector.New(0, 100)},
			p:    *vector.New(1, 1),
			want: 1,
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.l.Distance(c.p); math.Abs(c.want-got) > tolerance {
				t.Errorf("Distance() = %v, want = %v", got, c.want)
			}
		})
	}
}

func TestT(t *testing.T) {
	testConfigs := []struct {
		name string
		l    L
		t    float64
		want vector.V
	}{
		{
			name: "SimpleHorizontal",
			l:    L{p: *vector.New(1, 0), d: *vector.New(0, 1)},
			t:    1,
			want: *vector.New(1, 1),
		},
		{
			name: "SimpleVertical",
			l:    L{p: *vector.New(0, 1), d: *vector.New(1, 0)},
			t:    1,
			want: *vector.New(1, 1),
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.l.T(c.t); !vector.Within(got, c.want, tolerance) {
				t.Errorf("T() = %v, want = %v", got, c.want)
			}
		})
	}
}

func TestIntersection(t *testing.T) {
	testConfigs := []struct {
		name    string
		l       L
		m       L
		success bool
		want    float64
	}{
		{
			name:    "SimpleOrigin",
			l:       L{p: *vector.New(0, 0), d: *vector.New(1, 0)},
			m:       L{p: *vector.New(0, 0), d: *vector.New(0, 1)},
			success: true,
			want:    0,
		},
		{
			name:    "SimpleYIntercept",
			l:       L{p: *vector.New(0, 0), d: *vector.New(1, 0)},
			m:       L{p: *vector.New(1, 0), d: *vector.New(0, 1)},
			success: true,
			want:    1,
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got, ok := c.l.Intersect(c.m, tolerance); ok != c.success || math.Abs(got-c.want) >= tolerance {
				t.Errorf("Intersection() = %v, %v, want = %v, %v", got, ok, c.want, c.success)
			}
		})
	}
}
