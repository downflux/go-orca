package segment

import (
	"fmt"
	"testing"

	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
)

func TestCollision(t *testing.T) {
	configs := []struct {
		name string

		obstacle segment.S
		p vector.V
		radius float64

		success bool
	}{
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if _, err := New(c.obstacle, c.p, c.radius); c.success && err != nil {
				t.Errorf("New() = _, %v, want = _, nil", err)
			} else if !c.success && err == nil {
				t.Errorf("New() = _, nil, want a non-nil error")
			}
		})
	}
}

func TestTangents(t *testing.T) {
	configs := []struct {
		name string

		s    S
		l    vector.V
		r    vector.V
	}{
	}

	for _, c := range configs {
		t.Run(fmt.Sprintf("%v/L", c.name), func(t *testing.T) {
			if got := c.s.L(); !vector.Within(got, c.l) {
				t.Errorf("L() = %v, want = %v", got, c.l)
			}
		})
		t.Run(fmt.Sprintf("%v/R", c.name), func(t *testing.T) {
			if got := c.s.R(); !vector.Within(got, c.r) {
				t.Errorf("R() = %v, want = %v", got, c.r)
			}
		})
	}
}
