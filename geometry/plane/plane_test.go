package plane

import (
	"testing"

	"github.com/downflux/orca/geometry/vector"
)

const tolerance = 1e-10

func TestIn(t *testing.T) {
	type check struct {
		v    vector.V
		want bool
	}
	testConfigs := []struct {
		name  string
		hp    HP
		l     vector.V
		tests []check
	}{
		{
			name: "Horizontal",
			hp: *New(
				*vector.New(0, 0),
				*vector.New(1, 0),
			),
			l: *vector.New(0, 1),
			tests: []check{
				{v: *vector.New(0, 0), want: true},
				{v: *vector.New(1, 0), want: true},
				{v: *vector.New(-1, 0), want: false},
			},
		},
		{
			name: "Vertical",
			hp: *New(
				*vector.New(0, 0),
				*vector.New(0, 1),
			),
			l: *vector.New(-1, 0),
			tests: []check{
				{v: *vector.New(0, 0), want: true},
				{v: *vector.New(0, 1), want: true},
				{v: *vector.New(0, -1), want: false},
			},
		},
		{
			name: "Sloped",
			hp: *New(
				*vector.New(0, 0),
				*vector.New(1, 1),
			),
			l: *vector.New(-1, 1),
			tests: []check{
				{v: *vector.New(0, 0), want: true},
				{v: *vector.New(1, 1), want: true},
				{v: *vector.New(-1, -1), want: false},
			},
		},
		{
			name: "SlopedOffset",
			hp: *New(
				*vector.New(0, 1),
				*vector.New(1, 1),
			),
			l: *vector.New(-1, 1),
			tests: []check{
				{v: *vector.New(0, 0), want: false},
				{v: *vector.New(1, 1), want: true},
				{v: *vector.New(2, 2), want: true},
			},
		},
	}

	for _, c := range testConfigs {
		t.Run(c.name, func(t *testing.T) {
			for _, test := range c.tests {
				if got := c.hp.l(); !vector.Within(got, c.l, tolerance) {
					t.Fatalf("l() = %v, want = %v", got, c.l)
				}
				if got := c.hp.In(test.v); got != test.want {
					t.Errorf("In(%v) = %v, want = %v", test.v, got, test.want)
				}
			}
		})
	}
}
