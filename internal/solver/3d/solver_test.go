package solver

import (
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/google/go-cmp/cmp"
)

var _ M = Unbounded{}

func TestProject(t *testing.T) {
	testConfigs := []struct {
		name string

		cs []constraint.C
		c  constraint.C

		success bool
		want    []constraint.C
	}{
		{
			name: "Parallel/Override",
			cs: []constraint.C{
				*constraint.New(
					*vector.New(0, 0),
					*vector.New(0, 1),
				),
			},
			c: *constraint.New(
				*vector.New(0, 1),
				*vector.New(0, 1),
			),
			success: true,
			want: []constraint.C{
				*constraint.New(
					*vector.New(0, 1),
					*vector.New(0, 1),
				),
			},
		},
		// A parallel constraint which relaxes a previous constraint
		// is not allowed when calling project(). The caller must make
		// sure this does not happen.
		{
			name: "Parallel/Infeasible",
			cs: []constraint.C{
				*constraint.New(
					*vector.New(0, 1),
					*vector.New(0, 1),
				),
			},
			c: *constraint.New(
				*vector.New(0, 0),
				*vector.New(0, 1),
			),
			success: false,
			want:    nil,
		},
		// Test we can construct the appropriate plane-plane
		// intersection for two constraints with a shared feasible
		// region.
		{
			name: "AntiParallel",
			cs: []constraint.C{
				*constraint.New(
					*vector.New(0, 0),
					*vector.New(0, 1),
				),
			},
			c: *constraint.New(
				*vector.New(0, 1),
				*vector.New(0, -1),
			),
			success: true,
			want: []constraint.C{
				*constraint.New(
					*vector.New(0, 0.5),
					*vector.New(0, -1),
				),
			},
		},
		// Test that a plane-plane intersection may be constructed for
		// two constraints even if the intersection is disjoint.
		{
			name: "AntiParallel/Disjoint",
			cs: []constraint.C{
				*constraint.New(
					*vector.New(0, 1),
					*vector.New(0, 1),
				),
			},
			c: *constraint.New(
				*vector.New(0, 0),
				*vector.New(0, -1),
			),
			success: true,
			want: []constraint.C{
				*constraint.New(
					*vector.New(0, 0.5),
					*vector.New(0, -1),
				),
			},
		},

		{
			name: "Bisect",
			cs: []constraint.C{
				*constraint.New(
					*vector.New(0, 1),
					*vector.New(-1, 1),
				),
			},
			c: *constraint.New(
				*vector.New(0, 1),
				*vector.New(1, 1),
			),
			success: true,
			want: []constraint.C{
				*constraint.New(
					*vector.New(0, 1),
					// The square root is an artifact of
					// normalizing both input constraint
					// normals. The key takeaway is that the
					// result is pointing in the same
					// direction.
					*vector.New(-math.Sqrt(2), 0),
				),
			},
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			r := &region{
				constraints: c.cs,
			}

			got, ok := r.project(c.c)
			if c.success != ok {
				t.Errorf("project() = _, %v, want = _, %v", got, c.success)
			}

			if diff := cmp.Diff(
				c.want,
				got,
				cmp.Comparer(
					func(a, b constraint.C) bool {
						return hyperplane.Within(
							hyperplane.HP(a),
							hyperplane.HP(b),
						)
					},
				),
			); diff != "" {
				t.Errorf("project() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}

func TestAdd(t *testing.T) {
	testConfigs := []struct {
		name    string
		m       M
		cs      []constraint.C
		c       constraint.C
		success bool
		want    vector.V
	}{
		// When no 3D constraints exist, the optimal vector returned
		// should be moving directly into the feasible region of the
		// single constraint.
		{
			name: "Trivial",
			m:    Unbounded{},
			cs:   nil,
			c: *constraint.New(
				*vector.New(1, 2),
				*vector.New(0, 2),
			),
			success: true,
			want:    *vector.New(0, 2),
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			r := region{
				m:           c.m,
				constraints: c.cs,
			}

			if got, ok := r.Add(c.c); ok != c.success || !vector.Within(c.want, got) {
				t.Errorf("Add() = %v, %v, want = %v, %v", got, ok, c.want, c.success)
			}

		})
	}
}
