package generator

import (
	"math/rand"

	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-orca/examples/agent"
	"github.com/downflux/go-orca/examples/config"
	"github.com/downflux/go-orca/examples/segment"
)

func rn(min float64, max float64) float64 { return rand.Float64()*(max-min) + min }

// G generates a grid of points.
func G(x int, y int, s float64, r float64) config.O {
	var ps []vector.V
	var gs []vector.V

	for i := 0; i < x; i++ {
		for j := 0; j < y; j++ {
			ps = append(ps, *vector.New(float64(i)*50, float64(j)*50))
			gs = append(gs, *vector.New(float64(i)*50, float64(j)*50))
		}
	}

	rand.Shuffle(len(gs), func(i, j int) { gs[i], gs[j] = gs[j], gs[i] })

	var os []agent.O
	for i, p := range ps {
		os = append(os, agent.O{
			P: p,
			G: gs[i],
			R: r,
			S: s,
		})
	}
	return config.O{
		Agents: os,
	}
}

// C generates colliding points.
func C(s float64, r float64) config.O {
	ps := []vector.V{
		*vector.New(-50, 0),
		*vector.New(50, 0),
	}

	return config.O{
		Agents: []agent.O{
			agent.O{
				P: ps[0],
				G: vector.Add(ps[0], *vector.New(100, 0)),
				S: s,
				R: r,
			},
			agent.O{
				P: ps[1],
				G: vector.Add(ps[1], *vector.New(-100, 0)),
				S: s,
				R: r,
			},
		},
	}
}

// L generates some random agents with collision lines.
func L(w int, h int, s float64, r float64) config.O {
	segments := []segment.O{
		{
			P:    *vector.New(25, 25),
			D:    *vector.New(0, 1),
			TMin: 0,
			TMax: float64(h) / 4 * 50,
		},
		{
			P: vector.Add(
				*vector.New(75, 25),
				*vector.New(25, 0),
			),
			D:    *vector.New(1, 1),
			TMin: 0,
			TMax: float64(h) / 4 * 50,
		},
	}
	return config.O{
		Agents:   G(w, h, s, r).Agents,
		Segments: segments,
	}
}

// R generates n random agents.
func R(w int, h int, s float64, r float64, n int) config.O {
	agents := make([]agent.O, 0, n)
	for i := 0; i < n; i++ {
		p := vector.Add(
			*vector.New(rn(-500, 500), rn(-500, 500)),
			*vector.New(float64(w)/2, float64(h)/2),
		)
		g := vector.Add(
			p,
			*vector.New(rn(-100, 100), rn(-100, 100)),
		)

		agents = append(agents, agent.O{
			P: p,
			G: g,
			S: rn(5, s),
			R: rn(5, r),
		},
		)
	}
	return config.O{
		Agents: agents,
	}
}

func DebugCanvas() config.O {
	configs := []config.O{
		{
			Agents: []agent.O{
				// Generate embedded agent on left side of line
				// segment.
				{
					P: *vector.New(10, 10),
					G: *vector.New(10, 100),
					S: 10,
					R: 10,
				},
				// Generate embedded agent on the line segment
				// itself.
				{
					P: *vector.New(10.1, 30.1),
					G: *vector.New(10, -400),
					S: 0,
					R: 10, // setting to 1 panics
				},
			},
			Segments: []segment.O{
				{
					P:    *vector.New(10.1, 10.1),
					D:    *vector.New(0, 1),
					TMin: 0,
					TMax: 30,
				},
			},
		},

		// Ensure we have a non-vertical component here.
		{
			Agents: []agent.O{
				{
					P: *vector.New(100, 100),
					G: *vector.New(100, 200),
					S: 10,
					R: 10,
				},
				{
					P: *vector.New(100, 120),
					G: *vector.New(100, 0),
					S: 10,
					R: 10,
				},
			},
		},
	}

	var agents []agent.O
	var segments []segment.O

	for _, c := range configs {
		agents = append(agents, c.Agents...)
		segments = append(segments, c.Segments...)
	}

	return config.O{
		Agents:   agents,
		Segments: segments,
	}
}
