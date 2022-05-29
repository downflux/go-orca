package segment

import (
	"fmt"
	"testing"

	"github.com/downflux/go-geometry/2d/vector"
)

func TestTangents(t *testing.T) {
	configs := []struct {
		name string
		s    S
		l    vector.V
		r    vector.V
	}{}

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
