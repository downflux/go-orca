package reference

import (
	"testing"

	"github.com/downflux/orca/geometry/plane"
	"github.com/downflux/orca/geometry/vector"

	agent "github.com/downflux/orca/agent/reference"
)

func TestAdd(t *testing.T) {
	testConfigs := []struct {
		name       string
		h          Helper
		constraint plane.HP
		success    bool
		want       vector.V
	}{
		{
			name: "Trivial",
			h: Helper{
				a: *agent.New(
					agent.O{
						G: *vector.New(0, 1),
						S: 1,
					},
				),
			},
			constraint: *plane.New(*vector.New(0, 0), *vector.New(0, 1)),
			success:    true,
			want:       *vector.New(0, 1),
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got, success := c.h.Add(c.constraint); success != c.success || !vector.Within(got, c.want, tolerance) {
				t.Fatalf("Add() = %v, %v, want = %v, %v", got, success, c.want, c.success)
			}
		})
	}
}
