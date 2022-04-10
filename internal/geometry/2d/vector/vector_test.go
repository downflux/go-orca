package vector

import (
	"testing"

	"github.com/downflux/go-geometry/2d/vector"
)

func TestIsNormalOrientation(t *testing.T) {
	testConfigs := []struct{
		name string
		v vector.V
		u vector.V
		want bool
	}{
		{
			name: "Parallel",
			v: *vector.New(1, 0),
			u: *vector.New(1, 0),
			want: true,
		},
		{
			name: "Parallel/NonUnit",
			v: *vector.New(2, 0),
			u: *vector.New(5, 0),
			want: true,
		},
		{
			name: "AntiParallel",
			v: *vector.New(1, 0),
			u: *vector.New(-1, 0),
			want: false,
		},
		{
			name: "AntiParallel/NonUnit",
			v: *vector.New(2, 0),
			u: *vector.New(-5, 0),
			want: false,
		},
		{
			name: "Normal",
			v: *vector.New(1, 0),
			u: *vector.New(0, 1),
			want: true,
		},
		{
			name: "Normal/LargeTheta",
			v: *vector.New(1, 0),
			u: *vector.New(-1, 1),
			want: true,
		},
		{
			name: "NonNormal",
			v: *vector.New(1, 0),
			u: *vector.New(0, -1),
			want: false,
		},
		{
			name: "NonNormal/LargeTheta",
			v: *vector.New(1, 0),
			u: *vector.New(-1, -1),
			want: false,
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			if got := IsNormalOrientation(c.v, c.u); got != c.want {
				t.Errorf("IsNormalOrientation() = %v, want = %v", got, c.want)
			}
		})
	}
}
