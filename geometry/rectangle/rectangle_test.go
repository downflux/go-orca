package rectangle

import (
	"testing"

	"github.com/downflux/orca/geometry/vector"
	"github.com/google/go-cmp/cmp"
)

func TestIntersection(t *testing.T) {
	testConfigs := []struct {
		name        string
		r           R
		s           R
		want        R
		wantSuccess bool
	}{
		{
			name: "Trivial",
			r: *New(
				*vector.New(1, 2),
				*vector.New(2, 3),
			),
			s: *New(
				*vector.New(1, 2),
				*vector.New(2, 3),
			),
			want: *New(
				*vector.New(1, 2),
				*vector.New(2, 3),
			),
			wantSuccess: true,
		},
		{
			name: "Enveloped",
			r: *New(
				*vector.New(1, 2),
				*vector.New(100, 200),
			),
			s: *New(
				*vector.New(2, 3),
				*vector.New(99, 199),
			),
			want: *New(
				*vector.New(2, 3),
				*vector.New(99, 199),
			),
			wantSuccess: true,
		},
		{
			name: "Overlap/Left",
			r: *New(
				*vector.New(0, 0),
				*vector.New(5, 5),
			),
			s: *New(
				*vector.New(-1, 0),
				*vector.New(1, 5),
			),
			want: *New(
				*vector.New(0, 0),
				*vector.New(1, 5),
			),
			wantSuccess: true,
		},
		{
			name: "Overlap/Right",
			r: *New(
				*vector.New(-1, 0),
				*vector.New(1, 5),
			),
			s: *New(
				*vector.New(0, 0),
				*vector.New(5, 5),
			),
			want: *New(
				*vector.New(0, 0),
				*vector.New(1, 5),
			),
			wantSuccess: true,
		},
		{
			name: "Overlap/Top",
			r: *New(
				*vector.New(0, 0),
				*vector.New(5, 5),
			),
			s: *New(
				*vector.New(0, 4),
				*vector.New(5, 6),
			),
			want: *New(
				*vector.New(0, 4),
				*vector.New(5, 5),
			),
			wantSuccess: true,
		},
		{
			name: "Overlap/Bottom",
			r: *New(
				*vector.New(0, 4),
				*vector.New(5, 6),
			),
			s: *New(
				*vector.New(0, 0),
				*vector.New(5, 5),
			),
			want: *New(
				*vector.New(0, 4),
				*vector.New(5, 5),
			),
			wantSuccess: true,
		},
		{
			name: "NoOverlap/Left",
			r: *New(
				*vector.New(0, 0),
				*vector.New(5, 5),
			),
			s: *New(
				*vector.New(-5, 0),
				*vector.New(-1, 5),
			),
			wantSuccess: false,
		},
		{
			name: "NoOverlap/Top",
			r: *New(
				*vector.New(0, 0),
				*vector.New(5, 5),
			),
			s: *New(
				*vector.New(0, 6),
				*vector.New(5, 9),
			),
			wantSuccess: false,
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			got, ok := c.r.Intersect(c.s)
			if ok != c.wantSuccess {
				t.Errorf("Intersect() = _, %v, want = _, %v", ok, c.wantSuccess)
			}
			if diff := cmp.Diff(
				c.want,
				got,
				cmp.AllowUnexported(
					R{},
					vector.V{},
				),
			); diff != "" {
				t.Errorf("Intersect() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}
