package region

import (
	"testing"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
)

func TestIn(t *testing.T) {
	testConfigs := []struct {
		name string
		r    R
		v    int
		want bool
	}{
		{
			name: "Concave",
			// r is a square about the origin; all points about the
			// origin are considered inside the region.
			r: *New([]segment.S{
				*segment.New(
					*line.New(
						/* p = */ *vector.New(-1, 1),
						/* d = */ *vector.New(1, 0),
					), 0, 2),
				*segment.New(
					*line.New(
						*vector.New(1, 1),
						*vector.New(0, -1),
					), 0, 2),
				*segment.New(
					*line.New(
						*vector.New(1, -1),
						*vector.New(-1, 0),
					), 0, 2),
				*segment.New(
					*line.New(
						*vector.New(-1, -1),
						*vector.New(0, 1),
					), 0, 2),
			}),
			v:    0,
			want: true,
		},

		{
			name: "Convex",
			// r is a square about the origin defined in
			// counter-clockwise order. All points about the origin
			// are considered outside of the region.
			r: *New([]segment.S{
				*segment.New(
					*line.New(
						/* p = */ *vector.New(-1, 1),
						/* d = */ *vector.New(0, -1),
					), 0, 2),
				*segment.New(
					*line.New(
						*vector.New(-1, -1),
						*vector.New(1, 0),
					), 0, 2),
				*segment.New(
					*line.New(
						*vector.New(1, -1),
						*vector.New(0, 1),
					), 0, 2),
				*segment.New(
					*line.New(
						*vector.New(1, 1),
						*vector.New(-1, 0),
					), 0, 2),
			}),
			v:    0,
			want: false,
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := c.r.Concave(c.v); got != c.want {
				t.Errorf("In() = %v, want = %v", got, c.want)
			}
		})
	}
}
