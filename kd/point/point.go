package point

import (
	"sort"

	"github.com/downflux/orca/geometry/vector"
	"github.com/downflux/orca/kd/axis"
)

type sorter struct {
	axis axis.Type
	data []P
}

func (s *sorter) Len() int { return len(s.data) }
func (s *sorter) Less(i, j int) bool { return axis.X(s.data[i].V(), s.axis) < axis.X(s.data[j].V(), s.axis) }
func (s *sorter) Swap(i, j int) { s.data[i], s.data[j] = s.data[j], s.data[i] }

type P interface {
	V() vector.V

        // Hash calculates the hash string of the embedded data, if any. Note
        // that the hash may be dynamically generated, and the implementation
        // does not assume this is a static value.
        Hash() string
}

// Sort sorts a list of points in-place by the given axis.
func Sort(ps []P, a axis.Type) { sort.Sort(&sorter{axis: a, data: ps}) }
