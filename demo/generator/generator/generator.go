package generator

import (
	"encoding/json"
	"fmt"
	"math/rand"

	"github.com/downflux/go-geometry/2d/vector"

	demo "github.com/downflux/go-orca/demo/agent"
)

func rn(min float64, max float64) float64 { return rand.Float64()*(max-min) + min }

func Marshal(agents []demo.O) []byte {
	b, err := json.MarshalIndent(agents, "", " ")
	if err != nil {
		panic(fmt.Sprintf("cannot export agents: %v", err))
	}
	return b
}

func Unmarshal(data []byte) []demo.O {
	var agents []demo.O
	if err := json.Unmarshal(data, &agents); err != nil {
		panic(fmt.Sprintf("cannot import agents: %v", err))
	}
	return agents
}

// C generates colliding points.
func C() []demo.O {
	ps := []vector.V{
		*vector.New(-50, 0),
		*vector.New(50, 0),
	}

	return []demo.O{
		demo.O{
			P: ps[0],
			G: vector.Add(ps[0], *vector.New(100, 0)),
		},
		demo.O{
			P: ps[1],
			G: vector.Add(ps[1], *vector.New(-100, 0)),
		},
	}
}

// R generates n random agents.
func R(w int, h int, s float64, r float64, n int) []demo.O {
	agents := make([]demo.O, 0, n)
	for i := 0; i < n; i++ {
		p := vector.Add(
			*vector.New(rn(-500, 500), rn(-500, 500)),
			*vector.New(float64(w)/2, float64(h)/2),
		)
		g := vector.Add(
			p,
			*vector.New(rn(-100, 100), rn(-100, 100)),
		)

		agents = append(agents, demo.O{
			P: p,
			G: g,
			S: rn(5, s),
			R: rn(5, r),
		},
		)
	}
	return agents
}
