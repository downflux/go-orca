package generator

import (
	"encoding/json"
	"log"
	"math/rand"

	"github.com/downflux/go-geometry/2d/vector"

	demo "github.com/downflux/go-orca/demo/agent"
)

func rn(min float64, max float64) float64 { return rand.Float64()*(max-min) + min }

func Marshal(agents []demo.O) []byte {
	b, err := json.MarshalIndent(agents, "", " ")
	if err != nil {
		log.Fatalf("cannot export agents: %v", err)
	}
	return b
}

func Unmarshal(data []byte) []demo.O {
	var agents []demo.O
	if err := json.Unmarshal(data, &agents); err != nil {
		log.Fatalf("cannot import agents: %v", err)
	}
	return agents
}

// G generates a grid of points.
func G(x int, y int, s float64, r float64) []demo.O {
	var ps []vector.V
	var gs []vector.V

	for i := 0; i < x; i++ {
		for j := 0; j < y; j++ {
			ps = append(ps, *vector.New(float64(i)*50, float64(j)*50))
			gs = append(gs, *vector.New(float64(i)*50, float64(j)*50))
		}
	}

	rand.Shuffle(len(gs), func(i, j int) { gs[i], gs[j] = gs[j], gs[i] })

	var os []demo.O
	for i, p := range ps {
		os = append(os, demo.O{
			P: p,
			G: gs[i],
			R: r,
			S: s,
		})
	}
	return os
}

// C generates colliding points.
func C(s float64, r float64) []demo.O {
	ps := []vector.V{
		*vector.New(-50, 0),
		*vector.New(50, 0),
	}

	return []demo.O{
		demo.O{
			P: ps[0],
			G: vector.Add(ps[0], *vector.New(100, 0)),
			S: s,
			R: r,
		},
		demo.O{
			P: ps[1],
			G: vector.Add(ps[1], *vector.New(-100, 0)),
			S: s,
			R: r,
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
