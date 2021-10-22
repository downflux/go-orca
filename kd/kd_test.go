package kd

import (
	"testing"

	"github.com/downflux/orca/geometry/vector"
	"github.com/kyroy/kdtree"
)

var _ kdtree.Point = point{}

type data struct {
	vector.V
	d int
}

func TestMemoryLeak(t *testing.T) {
}

func TestInsertRemove(t *testing.T) {
	testConfigs := []struct {
		name   string
		add    []P
		remove []P
		want   int
	}{
		{
			name: "Insert/Simple",
			add: []P{
				vector.New(1, 2),
			},
			want: 1,
		},
		{
			name: "Insert/DuplicateCoordinates",
			add: []P{
				vector.New(1, 2),
				vector.New(1, 2),
			},
			want: 2,
		},
		{
			name: "Remove/NoPoints",
			remove: []P{
				vector.New(1, 2),
			},
			want: 0,
		},
		{
			name: "Remove/Simple",
			add: []P{
				vector.New(1, 2),
			},
			remove: []P{
				vector.New(1, 2),
			},
			want: 0,
		},
		// Removing a point should check the embedded data.
		{
			name: "Remove/SimpleWithData",
			add: []P{
				data{
					V: *vector.New(1, 2),
					d: 1,
				},
			},
			remove: []P{
				data{
					V: *vector.New(1, 2),
					d: 0,
				},
			},
			want: 1,
		},
		{
			name: "Remove/DuplicateCoordinates",
			add: []P{
				vector.New(1, 2),
				vector.New(1, 2),
			},
			remove: []P{
				vector.New(1, 2),
			},
			want: 1,
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			tr := New()
			for _, p := range c.add {
				tr.Insert(p)
			}
			for _, p := range c.remove {
				tr.Remove(p)
			}

			if got := len(tr.t.Points()); got != c.want {
				t.Errorf("len() = %v, want = %v", got, c.want)
			}
		})
	}
}
