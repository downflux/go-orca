package knn

import (
	"testing"

	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/kd/node"
	"github.com/downflux/orca/kd/point"
	"github.com/google/go-cmp/cmp"

	mock "github.com/downflux/orca/kd/point/mock"
)

const (
	tolerance = 1e-10
)

func TestQueue(t *testing.T) {
	type config struct {
		name string
		n    *node.N
		v    vector.V
		want []*node.N
	}

	testConfigs := []config{
		{
			name: "Null",
			n:    nil,
			v:    *vector.New(1, 2),
			want: nil,
		},

		// Ensure a path to a leaf node is returned if the query
		// point is the same as the leaf.
		func() config {
			n := node.New(
				[]point.P{
					*mock.New(*vector.New(1, 2), ""),
				},
				0,
				tolerance,
			)
			return config{
				name: "Leaf/Match",
				n:    n,
				v:    n.V(),
				want: []*node.N{n},
			}
		}(),

		// Ensure a path to a leaf node is returned even if the
		// query point is the different from the leaf.
		func() config {
			n := node.New(
				[]point.P{
					*mock.New(*vector.New(1, 2), ""),
				},
				0,
				tolerance,
			)
			return config{
				name: "Leaf/NoMatch",
				n:    n,
				v:    *vector.New(0, 2),
				want: []*node.N{n},
			}
		}(),

		// Ensure the generated path starts from the leaf node.
		func() config {
			n := node.New(
				[]point.P{
					*mock.New(*vector.New(1, 2), ""),
					*mock.New(*vector.New(2, 2), ""),
				},
				0,
				tolerance,
			)
			return config{
				name: "AssertLeafFirst",
				n:    n,
				v:    *vector.New(1, 2),
				want: []*node.N{
					n.L(),
					n,
				},
			}
		}(),

		// Ensure that even if a non-leaf node matches the query, the
		// path continues to generate towards a leaf node.
		func() config {
			n := node.New(
				[]point.P{
					*mock.New(*vector.New(1, 2), ""),
					*mock.New(*vector.New(2, 2), ""),
					*mock.New(*vector.New(3, 2), ""),
				},
				0,
				tolerance,
			)
			return config{
				name: "AssertAlwaysLeaf",
				n:    n,
				v:    *vector.New(2, 2),
				want: []*node.N{
					n.R(),
					n,
				},
			}
		}(),
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			got := queue(c.n, c.v, tolerance)
			if diff := cmp.Diff(
				c.want,
				got,
				cmp.AllowUnexported(node.N{}, vector.V{}, mock.P{})); diff != "" {
				t.Errorf("queue() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}
