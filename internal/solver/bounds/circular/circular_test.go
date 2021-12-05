package circular

import (
	"math"
	"testing"

	"github.com/downflux/go-geometry/2d/constraint"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/line"
	"github.com/downflux/go-orca/internal/solver/3d"
	"github.com/google/go-cmp/cmp"

	l2d "github.com/downflux/go-geometry/2d/line"
)

var _ solver.M = M{}

func TestBound(t *testing.T) {
	testConfigs := []struct {
		name    string
		r       float64
		c       constraint.C
		success bool
		want    segment.S
	}{
		{
			name: "Simple",
			r:    -1,
			c: *constraint.New(
				*vector.New(0, 0),
				*vector.New(1, 0),
			),
			success: true,
			want: *segment.New(
				*l2d.New(
					*vector.New(0, 0),
					*vector.New(0, 1),
				),
				-1,
				1,
			),
		},
		{
			name: "Simple/Reverse",
			r:    1,
			c: *constraint.New(
				*vector.New(0, 0),
				*vector.New(-1, 0),
			),
			success: true,
			want: *segment.New(
				*l2d.New(
					*vector.New(0, 0),
					*vector.New(0, -1),
				),
				-1,
				1,
			),
		},
		{
			name: "Infeasible",
			r:    1,
			c: *constraint.New(
				*vector.New(2, 0),
				*vector.New(1, 0),
			),
			success: false,
		},
		{
			name: "Unbounded",
			r:    math.Inf(0),
			c: *constraint.New(
				*vector.New(0, 0),
				*vector.New(1, 0),
			),
			success: true,
			want: *segment.New(
				*l2d.New(
					*vector.New(0, 0),
					*vector.New(0, 1),
				),
				math.Inf(-1),
				math.Inf(1),
			),
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			m := *New(c.r)
			got, ok := m.Bound(c.c)
			if ok != c.success {
				t.Errorf("Bound() = _, %v, want = _, %v", ok, c.success)
			}

			if diff := cmp.Diff(
				c.want,
				got,
				cmp.AllowUnexported(
					segment.S{},
					line.L{},
				)); diff != "" {
				t.Errorf("Bound() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}
