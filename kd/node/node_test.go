package node

import (
	"fmt"
	"testing"

	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/kd/point"
	"github.com/google/go-cmp/cmp"

	mock "github.com/downflux/orca/kd/point/mock"
)

const (
	tolerance = 1e-10
)

func TestNew(t *testing.T) {
	type config struct {
		name  string
		data  []point.P
		depth int
		want  *N
	}

	testConfigs := []config{
		{
			name:  "Null",
			data:  nil,
			depth: 0,
			want:  nil,
		},
	}

	for i := 0; i < 10; i++ {
		v := *vector.New(1, 2)
		testConfigs = append(
			testConfigs,
			config{
				name: fmt.Sprintf("Trivial/Depth-%v", i),
				data: []point.P{
					*mock.New(v, ""),
				},
				depth: i,
				want: &N{
					depth: i,
					v:     v,
					data: []point.P{
						*mock.New(v, ""),
					},
					sizeCache: 1,
				},
			},
		)
	}

	duplicateCoordinates := []point.P{}
	duplicateCoordinatesWithData := []point.P{}
	for i := 0; i < 10; i++ {
		duplicateCoordinates = append(duplicateCoordinates, *mock.New(*vector.New(1, 2), ""))
		duplicateCoordinatesWithData = append(duplicateCoordinates, *mock.New(*vector.New(1, 2), fmt.Sprintf("hash-%v", i)))
	}
	testConfigs = append(
		testConfigs,
		config{
			name:  "DuplicateCoordinates",
			data:  duplicateCoordinates,
			depth: 0,
			want: &N{
				depth:     0,
				v:         *vector.New(1, 2),
				data:      duplicateCoordinates,
				sizeCache: 1,
			},
		},
		config{
			name:  "DuplicateCoordinatesWithData",
			data:  duplicateCoordinatesWithData,
			depth: 0,
			want: &N{
				depth:     0,
				v:         *vector.New(1, 2),
				data:      duplicateCoordinatesWithData,
				sizeCache: 1,
			},
		},
	)

	testConfigs = append(
		testConfigs,

		config{
			name: "Simple/SortX",
			data: []point.P{
				*mock.New(*vector.New(2, 1), "A"),
				*mock.New(*vector.New(1, 2), "B"),
			},
			depth: 0,
			want: &N{
				l: &N{
					depth: 1,
					v:     *vector.New(1, 2),
					data: []point.P{
						*mock.New(*vector.New(1, 2), "B"),
					},
					sizeCache: 1,
				},
				depth: 0,
				v:     *vector.New(2, 1),
				data: []point.P{
					*mock.New(*vector.New(2, 1), "A"),
				},
				sizeCache: 2,
			},
		},
		config{
			name: "Simple/SortY",
			data: []point.P{
				*mock.New(*vector.New(1, 2), "B"),
				*mock.New(*vector.New(2, 1), "A"),
			},
			depth: 1,
			want: &N{
				l: &N{
					depth: 2,
					v:     *vector.New(2, 1),
					data: []point.P{
						*mock.New(*vector.New(2, 1), "A"),
					},
					sizeCache: 1,
				},
				depth: 1,
				v:     *vector.New(1, 2),
				data: []point.P{
					*mock.New(*vector.New(1, 2), "B"),
				},
				sizeCache: 2,
			},
		},

		// Input data is sorted before creating a node, so the input
		// data order should not matter in the constructor -- left and
		// right nodes are deterministically generated.
		config{
			name: "Simple/SortX/DataOrderInvariance",
			data: []point.P{
				*mock.New(*vector.New(1, 2), "B"),
				*mock.New(*vector.New(2, 1), "A"),
			},
			depth: 0,
			want: &N{
				l: &N{
					depth: 1,
					v:     *vector.New(1, 2),
					data: []point.P{
						*mock.New(*vector.New(1, 2), "B"),
					},
					sizeCache: 1,
				},
				depth: 0,
				v:     *vector.New(2, 1),
				data: []point.P{
					*mock.New(*vector.New(2, 1), "A"),
				},
				sizeCache: 2,
			},
		},

		config{
			name: "LRChild/SortX",
			data: []point.P{
				*mock.New(*vector.New(1, 3), ""),
				*mock.New(*vector.New(2, 2), ""),
				*mock.New(*vector.New(3, 1), ""),
			},
			depth: 0,
			want: &N{
				l: &N{
					depth: 1,
					v:     *vector.New(1, 3),
					data: []point.P{
						*mock.New(*vector.New(1, 3), ""),
					},
					sizeCache: 1,
				},
				r: &N{
					depth: 1,
					v:     *vector.New(3, 1),
					data: []point.P{
						*mock.New(*vector.New(3, 1), ""),
					},
					sizeCache: 1,
				},
				depth: 0,
				v:     *vector.New(2, 2),
				data: []point.P{
					*mock.New(*vector.New(2, 2), ""),
				},
				sizeCache: 3,
			},
		},
		config{
			name: "LRChild/SortY",
			data: []point.P{
				*mock.New(*vector.New(1, 3), ""),
				*mock.New(*vector.New(2, 2), ""),
				*mock.New(*vector.New(3, 1), ""),
			},
			depth: 1,
			want: &N{
				l: &N{
					depth: 2,
					v:     *vector.New(3, 1),
					data: []point.P{
						*mock.New(*vector.New(3, 1), ""),
					},
					sizeCache: 1,
				},
				r: &N{
					depth: 2,
					v:     *vector.New(1, 3),
					data: []point.P{
						*mock.New(*vector.New(1, 3), ""),
					},
					sizeCache: 1,
				},
				depth: 1,
				v:     *vector.New(2, 2),
				data: []point.P{
					*mock.New(*vector.New(2, 2), ""),
				},
				sizeCache: 3,
			},
		},
	)

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			got := New(c.data, c.depth, tolerance)

			if diff := cmp.Diff(
				c.want,
				got,
				cmp.AllowUnexported(N{}, vector.V{}, mock.P{})); diff != "" {
				t.Errorf("New() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}

func TestInsert(t *testing.T) {
	testConfigs := []struct {
		name  string
		data  []point.P
		depth int
		p     []point.P
		want  *N
	}{
		{
			name: "Simple",
			data: []point.P{
				*mock.New(*vector.New(1, 2), ""),
			},
			depth: 0,
			p: []point.P{
				*mock.New(*vector.New(2, 1), ""),
			},
			want: &N{
				r: &N{
					depth: 1,
					v:     *vector.New(2, 1),
					data: []point.P{
						*mock.New(*vector.New(2, 1), ""),
					},
					sizeCache: 1,
				},
				depth: 0,
				v:     *vector.New(1, 2),
				data: []point.P{
					*mock.New(*vector.New(1, 2), ""),
				},
				sizeCache: 2,
			},
		},

		{
			name: "Simple/Recursive",
			data: []point.P{
				*mock.New(*vector.New(1, 2), ""),
			},
			depth: 0,
			p: []point.P{
				*mock.New(*vector.New(2, 1), ""),
				*mock.New(*vector.New(1, 3), ""),
			},
			want: &N{
				depth: 0,
				v:     *vector.New(1, 2),
				data: []point.P{
					*mock.New(*vector.New(1, 2), ""),
				},
				sizeCache: 3,

				r: &N{
					depth: 1,
					v:     *vector.New(2, 1),
					data: []point.P{
						*mock.New(*vector.New(2, 1), ""),
					},
					sizeCache: 2,

					r: &N{
						depth: 2,
						v:     *vector.New(1, 3),
						data: []point.P{
							*mock.New(*vector.New(1, 3), ""),
						},
						sizeCache: 1,
					},
				},
			},
		},

		{
			name: "Degenerate",
			data: []point.P{
				*mock.New(*vector.New(1, 2), "A"),
			},
			depth: 0,
			p: []point.P{
				*mock.New(*vector.New(1, 2), "B"),
			},
			want: &N{
				depth: 0,
				v:     *vector.New(1, 2),
				data: []point.P{
					*mock.New(*vector.New(1, 2), "A"),
					*mock.New(*vector.New(1, 2), "B"),
				},
				sizeCache: 1,
			},
		},

		// Inserting a new point into a node with the same sorted axis
		// should not result in the point added to the current node; by
		// convention, points with a degenerate axis value will be
		// parsed into the right child node.
		{
			name: "Degenerate/SortedAxis",
			data: []point.P{
				*mock.New(*vector.New(1, 2), ""),
			},
			depth: 0,
			p: []point.P{
				*mock.New(*vector.New(1, 3), ""),
			},
			want: &N{
				depth: 0,
				v:     *vector.New(1, 2),
				data: []point.P{
					*mock.New(*vector.New(1, 2), ""),
				},
				r: &N{
					depth: 1,
					v:     *vector.New(1, 3),
					data: []point.P{
						*mock.New(*vector.New(1, 3), ""),
					},
					sizeCache: 1,
				},
				sizeCache: 2,
			},
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			n := New(c.data, c.depth, tolerance)
			for _, p := range c.p {
				n.Insert(p, tolerance)
			}

			if diff := cmp.Diff(
				c.want,
				n,
				cmp.AllowUnexported(N{}, vector.V{}, mock.P{})); diff != "" {
				t.Errorf("New() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}

func TestRemove(t *testing.T) {
	testConfigs := []struct {
		name  string
		data  []point.P
		depth int
		p     []point.P
		want  *N
	}{
		{
			name: "Trivial",
			data: []point.P{
				*mock.New(*vector.New(1, 2), ""),
			},
			depth: 0,
			p: []point.P{
				*mock.New(*vector.New(1, 2), ""),
			},
			want: &N{
				depth:     0,
				v:         *vector.New(1, 2),
				data:      []point.P{},
				sizeCache: 0,
			},
		},

		{
			name: "NotFound",
			data: []point.P{
				*mock.New(*vector.New(1, 2), ""),
			},
			depth: 0,
			p: []point.P{
				*mock.New(*vector.New(2, 2), "does-not-exist"),
			},
			want: &N{
				depth: 0,
				v:     *vector.New(1, 2),
				data: []point.P{
					*mock.New(*vector.New(1, 2), ""),
				},
				sizeCache: 1,
			},
		},

		{
			name: "SinglePoint",
			data: []point.P{
				*mock.New(*vector.New(2, 2), ""),
				*mock.New(*vector.New(1, 2), ""),
			},
			depth: 0,
			p: []point.P{
				*mock.New(*vector.New(1, 2), ""),
			},
			want: &N{
				depth: 0,
				v:     *vector.New(2, 2),
				data: []point.P{
					*mock.New(*vector.New(2, 2), ""),
				},
				l: &N{
					depth: 1,
					v:     *vector.New(1, 2),
					data:  []point.P{},
				},
				sizeCache: 1,
			},
		},

		{
			name: "Simple/Duplicates",
			data: []point.P{
				*mock.New(*vector.New(1, 2), ""),
				*mock.New(*vector.New(1, 2), ""),
			},
			depth: 0,
			p: []point.P{
				*mock.New(*vector.New(1, 2), ""),
			},
			want: &N{
				depth: 0,
				v:     *vector.New(1, 2),
				data: []point.P{
					*mock.New(*vector.New(1, 2), ""),
				},
				sizeCache: 1,
			},
		},
		{
			name: "Simple/DegenerateCoordinates",
			data: []point.P{
				*mock.New(*vector.New(1, 2), "A"),
				*mock.New(*vector.New(1, 2), "B"),
			},
			depth: 0,
			p: []point.P{
				*mock.New(*vector.New(1, 2), "A"),
			},
			want: &N{
				depth: 0,
				v:     *vector.New(1, 2),
				data: []point.P{
					*mock.New(*vector.New(1, 2), "B"),
				},
				sizeCache: 1,
			},
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			n := New(c.data, c.depth, tolerance)

			for _, p := range c.p {
				n.Remove(p, tolerance)
			}

			if diff := cmp.Diff(
				c.want,
				n,
				cmp.AllowUnexported(N{}, vector.V{}, mock.P{})); diff != "" {
				t.Errorf("New() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}
