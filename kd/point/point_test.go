package point

import (
	"testing"

	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/kd/axis"
	"github.com/google/go-cmp/cmp"

	mock "github.com/downflux/orca/kd/point/mock"
)

const (
	tolerance = 1e-10
)

var _ P = mock.P{}

func TestSorterLen(t *testing.T) {
	testConfigs := []struct {
		name string
		data []P
		axis axis.Type
		want int
	}{
		{
			name: "Null",
			data: nil,
			axis: axis.Axis_X,
			want: 0,
		},
		{
			name: "Simple",
			data: []P{
				*mock.New(*vector.New(1, 1), ""),
			},
			axis: axis.Axis_X,
			want: 1,
		},
		{
			name: "Duplicate",
			data: []P{
				*mock.New(*vector.New(1, 1), ""),
				*mock.New(*vector.New(1, 1), ""),
			},
			axis: axis.Axis_X,
			want: 2,
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			s := sorter{
				data: c.data,
				axis: c.axis,
			}
			if got := s.Len(); got != c.want {
				t.Errorf("Len() = %v, want = %v", got, c.want)
			}
		})
	}
}

func TestSorterLess(t *testing.T) {
	testConfigs := []struct {
		name string
		s    *sorter
		i    int
		j    int
		want bool
	}{
		{
			name: "OneElement/X",
			s: &sorter{
				data: []P{
					*mock.New(*vector.New(1, 2), ""),
				},
				axis: axis.Axis_X,
			},
			i:    0,
			j:    0,
			want: false,
		},
		{
			name: "OneElement/Y",
			s: &sorter{
				data: []P{
					*mock.New(*vector.New(1, 2), ""),
				},
				axis: axis.Axis_Y,
			},
			i:    0,
			j:    0,
			want: false,
		},
		{
			name: "Simple/X",
			s: &sorter{
				data: []P{
					*mock.New(*vector.New(1, 2), ""),
					*mock.New(*vector.New(2, 1), ""),
				},
				axis: axis.Axis_X,
			},
			i:    0,
			j:    1,
			want: true,
		},
		{
			name: "Simple/Y",
			s: &sorter{
				data: []P{
					*mock.New(*vector.New(1, 2), ""),
					*mock.New(*vector.New(2, 1), ""),
				},
				axis: axis.Axis_Y,
			},
			i:    0,
			j:    1,
			want: false,
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.s.Less(c.i, c.j); got != c.want {
				t.Errorf("Less = %v, want = %v", got, c.want)
			}
		})
	}
}

func TestSorterSwap(t *testing.T) {
	testConfigs := []struct {
		name string
		data []P
		i    int
		j    int
		want []vector.V
	}{
		{
			name: "OneElement",
			data: []P{
				*mock.New(*vector.New(1, 2), ""),
			},
			i: 0,
			j: 0,
			want: []vector.V{
				*vector.New(1, 2),
				*vector.New(1, 2),
			},
		},
		{
			name: "TwoElements",
			data: []P{
				*mock.New(*vector.New(1, 2), ""),
				*mock.New(*vector.New(2, 1), ""),
			},
			i: 0,
			j: 1,
			want: []vector.V{
				*vector.New(2, 1),
				*vector.New(1, 2),
			},
		},
	}
	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			s := &sorter{data: c.data}
			s.Swap(c.i, c.j)

			got := []vector.V{
				c.data[c.i].V(),
				c.data[c.j].V(),
			}

			if diff := cmp.Diff(
				c.want,
				got,
				cmp.AllowUnexported(mock.P{}, vector.V{})); diff != "" {
				t.Errorf("Swap() mismatch (-want +got):\n%s", diff)
			}
		})
	}
}

func TestSort(t *testing.T) {
	testConfigs := []struct {
		name string
		data []P
		axis axis.Type
		want []P
	}{
		{
			name: "Trivial/NoData/X",
			data: []P{*mock.New(*vector.New(1, 2), "")},
			axis: axis.Axis_X,
			want: []P{*mock.New(*vector.New(1, 2), "")},
		},
		{
			name: "Trivial/NoData/Y",
			data: []P{*mock.New(*vector.New(1, 2), "")},
			axis: axis.Axis_Y,
			want: []P{*mock.New(*vector.New(1, 2), "")},
		},

		{
			name: "Trivial/WithData/X",
			data: []P{*mock.New(*vector.New(1, 2), "foo")},
			axis: axis.Axis_X,
			want: []P{*mock.New(*vector.New(1, 2), "foo")},
		},
		{
			name: "Trivial/NoData/Y",
			data: []P{*mock.New(*vector.New(1, 2), "foo")},
			axis: axis.Axis_Y,
			want: []P{*mock.New(*vector.New(1, 2), "foo")},
		},

		{
			name: "Simple/NoData/X",
			data: []P{
				*mock.New(*vector.New(3, 1), ""),
				*mock.New(*vector.New(2, 2), ""),
				*mock.New(*vector.New(1, 3), ""),
			},
			axis: axis.Axis_X,
			want: []P{
				*mock.New(*vector.New(1, 3), ""),
				*mock.New(*vector.New(2, 2), ""),
				*mock.New(*vector.New(3, 1), ""),
			},
		},
		{
			name: "Simple/NoData/Y",
			data: []P{
				*mock.New(*vector.New(1, 3), ""),
				*mock.New(*vector.New(2, 2), ""),
				*mock.New(*vector.New(3, 1), ""),
			},
			axis: axis.Axis_Y,
			want: []P{
				*mock.New(*vector.New(3, 1), ""),
				*mock.New(*vector.New(2, 2), ""),
				*mock.New(*vector.New(1, 3), ""),
			},
		},

		{
			name: "Simple/WithData/X",
			data: []P{
				*mock.New(*vector.New(3, 1), "foo3"),
				*mock.New(*vector.New(2, 2), "foo2"),
				*mock.New(*vector.New(1, 3), "foo1"),
			},
			axis: axis.Axis_X,
			want: []P{
				*mock.New(*vector.New(1, 3), "foo1"),
				*mock.New(*vector.New(2, 2), "foo2"),
				*mock.New(*vector.New(3, 1), "foo3"),
			},
		},
		{
			name: "Simple/WithData/Y",
			data: []P{
				*mock.New(*vector.New(1, 3), "foo1"),
				*mock.New(*vector.New(2, 2), "foo2"),
				*mock.New(*vector.New(3, 1), "foo3"),
			},
			axis: axis.Axis_Y,
			want: []P{
				*mock.New(*vector.New(3, 1), "foo3"),
				*mock.New(*vector.New(2, 2), "foo2"),
				*mock.New(*vector.New(1, 3), "foo1"),
			},
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			Sort(c.data, c.axis)

			if diff := cmp.Diff(
				c.want,
				c.data,
				cmp.AllowUnexported(mock.P{}, vector.V{})); diff != "" {
				t.Errorf("Swap() mismatch (-want +got):\n%s", diff)
			}
		})
	}
}
