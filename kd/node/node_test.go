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
				depth: 0,
				v:     *vector.New(1, 2),
				data:  duplicateCoordinates,
			},
		},
		config{
			name:  "DuplicateCoordinatesWithData",
			data:  duplicateCoordinatesWithData,
			depth: 0,
			want: &N{
				depth: 0,
				v:     *vector.New(1, 2),
				data:  duplicateCoordinatesWithData,
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
				},
				depth: 0,
				v:     *vector.New(2, 1),
				data: []point.P{
					*mock.New(*vector.New(2, 1), "A"),
				},
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
				},
				depth: 1,
				v:     *vector.New(1, 2),
				data: []point.P{
					*mock.New(*vector.New(1, 2), "B"),
				},
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
				},
				depth: 0,
				v:     *vector.New(2, 1),
				data: []point.P{
					*mock.New(*vector.New(2, 1), "A"),
				},
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
				},
				r: &N{
					depth: 1,
					v:     *vector.New(3, 1),
					data: []point.P{
						*mock.New(*vector.New(3, 1), ""),
					},
				},
				depth: 0,
				v:     *vector.New(2, 2),
				data: []point.P{
					*mock.New(*vector.New(2, 2), ""),
				},
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
				},
				r: &N{
					depth: 2,
					v:     *vector.New(1, 3),
					data: []point.P{
						*mock.New(*vector.New(1, 3), ""),
					},
				},
				depth: 1,
				v:     *vector.New(2, 2),
				data: []point.P{
					*mock.New(*vector.New(2, 2), ""),
				},
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
