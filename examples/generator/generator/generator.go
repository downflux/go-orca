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
			D:    *vector.New(0, 50),
			TMin: 0,
			TMax: float64(h) / 4,
		},
		{
			P: vector.Add(
				*vector.New(75, 25),
				*vector.New(25, 0),
			),
			D:    *vector.New(50, 50),
			TMin: 0,
			TMax: float64(h) / 4,
		},

		// Generate box obstacle.
		{
			P:    *vector.New(125, 75),
			D:    *vector.New(0, 50),
			TMin: 0,
			TMax: 5,
		},
		{
			P:    *vector.New(125, 325),
			D:    *vector.New(50, 0),
			TMin: 0,
			TMax: 5,
		},
		{
			P:    *vector.New(375, 325),
			D:    *vector.New(0, -50),
			TMin: 0,
			TMax: 5,
		},
		{
			P:    *vector.New(375, 75),
			D:    *vector.New(-50, 0),
			TMin: 0,
			TMax: 5,
		},

		// Generate border.
		{
			P:    *vector.New(-50, -50),
			D:    *vector.New(0, 50),
			TMin: 0,
			TMax: float64(h) + 1,
		},
		{
			P:    *vector.New(-50, float64(h)*50),
			D:    *vector.New(50, 0),
			TMin: 0,
			TMax: float64(w) + 1,
		},
		{
			P:    *vector.New(float64(w)*50, float64(h)*50),
			D:    *vector.New(0, -50),
			TMin: 0,
			TMax: float64(h) + 1,
		},
		{
			P:    *vector.New(float64(w)*50, -50),
			D:    *vector.New(-50, 0),
			TMin: 0,
			TMax: float64(w) + 1,
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
					S: 20,
					R: 10,
				},
				// Generate embedded agent on the line segment
				// itself.
				{
					P: *vector.New(10.1, 30.1),
					G: *vector.New(10, 0),
					S: 50,
					R: 5,
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
		// Ensure we have a non-vertical component here for fast-moving
		// agents.
		{
			Agents: []agent.O{
				{
					P: *vector.New(1000, 10),
					G: *vector.New(1000, 100),
					S: 50,
					R: 10,
				},
				{
					P: *vector.New(1000, 100),
					G: *vector.New(1000, 10),
					S: 50,
					R: 10,
				},
			},
		},
		// One hangup for ORCA is dealing with relatively slow moving
		// agents which are colliding -- here, setting our velocity to a
		// small enough value will always ensure agents will not
		// collide. However, this is not an efficient large-scale
		// solution, as no meaningful progress will be made in actually
		// reaching the agent goals.
		//
		// See github.com/downflux/go-orca/internal/solver for more
		// information on editing the optimization function to avoid
		// this issue.
		{
			Agents: []agent.O{
				{
					P: *vector.New(900, 10),
					G: *vector.New(900, 100),
					S: 10,
					R: 10,
				},
				{
					P: *vector.New(900, 100),
					G: *vector.New(900, 10),
					S: 10,
					R: 10,
				},
			},
		},

		// For very close agents, ORCA will not force agents apart. This
		// is a very rare bug, and should not matter in everyday use.
		{
			Agents: []agent.O{
				{
					P: *vector.New(200+1e-6, 100),
					G: *vector.New(300, 100),
					S: 50,
					R: 10,
				},
				{
					P: *vector.New(200, 100),
					G: *vector.New(300, 100),
					S: 50,
					R: 10,
				},
			},
		},

		// Ensure agents travelling directly into the line segment will
		// veer away.
		{
			Agents: []agent.O{
				{
					P: *vector.New(700, 10),
					G: *vector.New(800, 10),
					S: 10,
					R: 10,
				},
				{
					P: *vector.New(750, 150),
					G: *vector.New(750, 0),
					S: 10,
					R: 10,
				},
				{
					P: *vector.New(750, 0),
					G: *vector.New(750, 150),
					S: 10,
					R: 10,
				},
				{
					P: *vector.New(700, 50),
					G: *vector.New(800, 50),
					S: 10,
					R: 10,
				},
			},
			Segments: []segment.O{
				{
					P:    *vector.New(750, 10),
					D:    *vector.New(0, 1),
					TMin: 0,
					TMax: 100,
				},
			},
		},
		// Demonstrate buggy behavior when attempting to solve ORCA for
		// multiple wall segments results in a forced collision.
		{
			Agents: []agent.O{
				{
					P: *vector.New(390, 210),
					G: *vector.New(250, 210),
					S: 50,
					R: 10,
				},
				{
					P: *vector.New(390, 240),
					G: *vector.New(450, 240),
					S: 50,
					R: 10,
				},
			},
			Segments: []segment.O{
				{
					P:    *vector.New(300, 200),
					D:    *vector.New(0, 1),
					TMin: 0,
					TMax: 50,
				},
				{
					P:    *vector.New(400, 200),
					D:    *vector.New(0, 1),
					TMin: 0,
					TMax: 50,
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
