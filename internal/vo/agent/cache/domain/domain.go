package domain

type D int

const (
	Left D = iota
	Right
	Circle
	Collision
)

func (d D) String() string {
	if s, ok := map[D]string{
		Left: "LEFT",
		Right: "RIGHT",
		Circle: "CIRCLE",
		Collision: "COLLISION",
	}[d]; ok {
		return s
	}
	return "UNKNOWN"
}
