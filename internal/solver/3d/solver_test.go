package solver

import (
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/hyperplane"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/internal/geometry/2d/constraint"
	"github.com/downflux/go-orca/internal/solver/bounds/unbounded"
	"github.com/google/go-cmp/cmp"

	c2d "github.com/downflux/go-geometry/2d/constraint"
)

func mutable(cs []c2d.C) []constraint.C {
	var ms []constraint.C
	for _, c := range cs {
		ms = append(ms, *constraint.New(c, true))
	}
	return ms
}

func TestProject(t *testing.T) {
	type config struct {
		name string

		cs []constraint.C
		c  constraint.C

		success bool
		want    []constraint.C
	}
	testConfigs := []config{
		{
			name: "Parallel/Override",
			cs: mutable(
				[]c2d.C{
					*c2d.New(
						*vector.New(0, 0),
						*vector.New(0, 1),
					),
				},
			),
			c: *constraint.New(
				*c2d.New(
					*vector.New(0, 1),
					*vector.New(0, 1),
				),
				true,
			),
			success: true,
			want: mutable(
				[]c2d.C{
					*c2d.New(
						*vector.New(0, 1),
						*vector.New(0, 1),
					),
				},
			),
		},
		// A parallel constraint which relaxes a previous constraint
		// is not allowed when calling project(). The caller must make
		// sure this does not happen.
		{
			name: "Parallel/Infeasible",
			cs: mutable(
				[]c2d.C{
					*c2d.New(
						*vector.New(0, 1),
						*vector.New(0, 1),
					),
				},
			),
			c: *constraint.New(
				*c2d.New(
					*vector.New(0, 0),
					*vector.New(0, 1),
				),
				true,
			),
			success: false,
			want:    nil,
		},
		// Test we can construct the appropriate plane-plane
		// intersection for two constraints with a shared feasible
		// region.
		{
			name: "AntiParallel",
			cs: mutable(
				[]c2d.C{
					*c2d.New(
						*vector.New(0, 0),
						*vector.New(0, 1),
					),
				},
			),
			c: *constraint.New(
				*c2d.New(
					*vector.New(0, 1),
					*vector.New(0, -1),
				),
				true,
			),
			success: true,
			want: mutable(
				[]c2d.C{
					*c2d.New(
						*vector.New(0, 0.5),
						*vector.New(0, -1),
					),
				},
			),
		},
		// Test that a plane-plane intersection may be constructed for
		// two constraints even if the intersection is disjoint.
		{
			name: "AntiParallel/Disjoint",
			cs: mutable(
				[]c2d.C{
					*c2d.New(
						*vector.New(0, 1),
						*vector.New(0, 1),
					),
				},
			),
			c: *constraint.New(
				*c2d.New(
					*vector.New(0, 0),
					*vector.New(0, -1),
				),
				true,
			),
			success: true,
			want: mutable(
				[]c2d.C{
					*c2d.New(
						*vector.New(0, 0.5),
						*vector.New(0, -1),
					),
				},
			),
		},

		{
			// We are inspecting the projected contraint generated
			// by the system of inequalities
			//
			//   (a)  x - y <= 1
			//   (b) -x - y <= 1
			//
			// We expect the line of bisection is defined by the
			// inequality
			//
			//   x <= 0
			name: "Bisect",
			cs: mutable(
				[]c2d.C{
					*c2d.New(
						*vector.New(0, 1),
						*vector.New(-1, 1),
					),
				},
			),
			c: *constraint.New(
				*c2d.New(
					*vector.New(0, 1),
					*vector.New(1, 1),
				),
				true,
			),
			success: true,
			want: mutable(
				[]c2d.C{
					*c2d.New(
						*vector.New(0, 1),
						*vector.New(-1, 0),
					),
				},
			),
		},

		func() config {
			// Corresponds to a system of three equations
			//
			//   (a)  2x - y <= -6
			//   (b) -3x - y <= -6
			//   (c)       y <= 1
			a := *c2d.New(
				*vector.New(0, 6),
				*vector.New(-2, 1),
			)
			b := *c2d.New(
				*vector.New(0, 6),
				*vector.New(3, 1),
			)
			c := *c2d.New(
				*vector.New(0, 1),
				*vector.New(0, -1),
			)

			return config{
				name:    "Bisect/Multiple",
				cs:      mutable([]c2d.C{a, b}),
				c:       *constraint.New(c, true),
				success: true,
				want: mutable(
					[]c2d.C{
						*c2d.New(
							*vector.New(-2.5, 1),
							vector.Unit(
								vector.Sub(
									vector.Unit(hyperplane.HP(a).N()),
									vector.Unit(hyperplane.HP(c).N()),
								),
							),
						),
						*c2d.New(
							*vector.New(5./3, 1),
							vector.Unit(
								vector.Sub(
									vector.Unit(hyperplane.HP(b).N()),
									vector.Unit(hyperplane.HP(c).N()),
								),
							),
						),
					},
				),
			}
		}(),
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
							hyperplane.HP(a.C()),
							hyperplane.HP(b.C()),
						)
					},
				),
			); diff != "" {
				t.Errorf("project() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}

func TestSolve(t *testing.T) {
	type config struct {
		name    string
		m       M
		cs      []constraint.C
		c       constraint.C
		success bool
		want    vector.V
	}

	testConfigs := []config{
		// When no 3D constraints exist, the optimal vector returned
		// should be moving directly into the feasible region of the
		// single constraint.
		{
			name: "Trivial",
			m:    unbounded.M{},
			cs:   nil,
			c: *constraint.New(
				*c2d.New(
					*vector.New(1, 2),
					*vector.New(0, 2),
				),
				true,
			),
			success: true,
			want:    *vector.New(0, 2),
		},

		// Ensure that we can find a solution for two parallel
		// intersecting constraints.
		{
			name: "AntiParallel/Intersect",
			m:    unbounded.M{},
			cs: mutable(
				[]c2d.C{
					*c2d.New(
						*vector.New(0, 1),
						*vector.New(0, -1),
					),
				},
			),
			c: *constraint.New(
				*c2d.New(
					*vector.New(0, 0),
					*vector.New(0, 1),
				),
				true,
			),
			success: true,
			// The 2D projection of the single pre-existing
			// constraint is parallel to the input constraint, and
			// covers the input constraint; thus, any solution which
			// fulfills the input constraint also fulfills the
			// projected constraint.
			//
			// The default optimization vector for the 2D problem is
			// a vector pointing to the normal the input constraint.
			want: *vector.New(0, 1),
		},

		// Ensure that we can find a solution for two disjoint
		// constraints.
		{
			name: "AntiParallel/Disjoint",
			m:    unbounded.M{},
			cs: mutable(
				[]c2d.C{
					*c2d.New(
						*vector.New(0, 1),
						*vector.New(0, 1),
					),
				},
			),
			c: *constraint.New(
				*c2d.New(
					*vector.New(0, 0),
					*vector.New(0, -1),
				),
				true,
			),
			success: true,
			// The 2D projection of the single pre-existing
			// constraint is parallel to the input constraint, and
			// covers the input constraint; thus, any solution which
			// fulfills the input constraint also fulfills the
			// projected constraint.
			//
			// The default optimization vector for the 2D problem is
			// a vector pointing to the normal the input constraint.
			want: *vector.New(0, -1),
		},
	}

	testConfigs = append(testConfigs, func() []config {
		// Corresponds to a system of three equations
		//
		//   (a)  2x - y <= -6
		//   (b) -3x - y <= -6
		//   (c)       y <= 1
		//
		// Note that the problem is infeasible in 2D -- there no point
		// in the XY-plane which satisfies all three constraints.
		//
		// We are adding the constraint defined by (c) into the existing
		// system of constraints defined by (a) and (b). This will
		// generate the projected linear equations as defined in
		// TestProject/Bisect/Multiple.
		a := *c2d.New(
			*vector.New(0, 6),
			*vector.New(-2, 1),
		)
		b := *c2d.New(
			*vector.New(0, 6),
			*vector.New(3, 1),
		)
		c := *c2d.New(
			*vector.New(0, 1),
			*vector.New(0, -1),
		)

		return []config{
			{
				name:    "2DInfeasible",
				m:       unbounded.M{},
				cs:      mutable([]c2d.C{a, b}),
				c:       *constraint.New(c, true),
				success: true,
				want:    *vector.New(math.Inf(0), math.Inf(0)),
			},
			// Test that when adding a new constraint into the 3D
			// region, the order of the previous constraints added
			// does not matter.
			{
				name:    "2DInfeasible/OrderInvariance",
				m:       unbounded.M{},
				cs:      mutable([]c2d.C{b, a}),
				c:       *constraint.New(c, true),
				success: true,
				want:    *vector.New(math.Inf(0), math.Inf(0)),
			},
		}
	}()...)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			r := region{
				m:           c.m,
				constraints: c.cs,
			}

			if got, ok := r.Solve(c.c); ok != c.success || !vector.Within(c.want, got) {
				t.Errorf("Add() = %v, %v, want = %v, %v", got, ok, c.want, c.success)
			}

		})
	}
}
