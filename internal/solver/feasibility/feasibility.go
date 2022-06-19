package feasibility

type F int

const (
	Feasible F = iota
	Infeasible
	Partial
)

func (f F) String() string {
	s, ok := map[F]string{
		Feasible:   "FEASIBLE",
		Infeasible: "INFEASIBLE",
		Partial:    "PARTIAL",
	}[f]
	if !ok {
		s = "UNKNOWN"
	}
	return s
}
