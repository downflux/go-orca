package circle

import (
	"testing"

	"github.com/downflux/orca/geometry/vector"
)

func TestIn(t *testing.T) {
	testConfigs := []struct {
		name string
		c    C
		p    vector.V
		want bool
	}{
		{
			name: "Origin/In",
			c:    C{p: *vector.New(0, 0), r: 1},
			p:    *vector.New(0, 0),
			want: true,
		},
		{
			name: "Origin/Out",
			c:    C{p: *vector.New(0, 0), r: 1},
			p:    *vector.New(0, 2),
			want: false,
		},
		{
			name: "Offset/In",
			c:    C{p: *vector.New(100, 100), r: 1},
			p:    *vector.New(100, 99),
			want: true,
		},
		{
			name: "Offset/Out",
			c:    C{p: *vector.New(100, 100), r: 1},
			p:    *vector.New(0, 0),
			want: false,
		},
	}

	for _, c := range testConfigs {
		if got := c.c.In(c.p); got != c.want {
			t.Errorf("In() = %v, want = %v", got, c.want)
		}
	}
}
