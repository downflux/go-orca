// Package main executes a short demo of ORCA and outputs a gif of the
// calculated trajectory of the set of agents.
//
// Run with
//   go run github.com/downflux/go-orca/demo
package main

import (
	"image"
	"image/color"
	"image/draw"
	"image/gif"
	"math/rand"
	"os"

	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/orca"

	demo "github.com/downflux/go-orca/demo/agent"
	util "github.com/downflux/go-orca/internal/orca"
)

const (
	N = 250

	S = 50 // m / s
	R = 10
	H = 1000
	W = 1000

	// Simulate ~120 frames.
	D   = 2.      // s
	TAU = 1.67e-2 // ~1/60 s
)

var (
	black = color.Black
	white = color.White
	red   = color.RGBA{255, 0, 0, 255}
	green = color.RGBA{0, 255, 0, 255}
	blue  = color.RGBA{0, 0, 255, 255}
)

func rn(min float64, max float64) float64 { return rand.Float64()*(max-min) + min }

func DrawCircle(img draw.Image, v vector.V, r int, c color.Color) {
	x, y, dx, dy := r-1, 0, 1, 1
	err := dx - (r * 2)

	for x > y {
		img.Set(int(v.X())+x, int(v.Y())+y, c)
		img.Set(int(v.X())+y, int(v.Y())+x, c)
		img.Set(int(v.X())-y, int(v.Y())+x, c)
		img.Set(int(v.X())-x, int(v.Y())+y, c)
		img.Set(int(v.X())-x, int(v.Y())-y, c)
		img.Set(int(v.X())-y, int(v.Y())-x, c)
		img.Set(int(v.X())+y, int(v.Y())-x, c)
		img.Set(int(v.X())+x, int(v.Y())-y, c)

		if err <= 0 {
			y++
			err += dy
			dy += 2
		}
		if err > 0 {
			x--
			dx += 2
			err += dx - (r * 2)
		}
	}
}

func GenerateRandomPoints(n int) []agent.A {
	agents := make([]agent.A, 0, n)
	for i := 0; i < N; i++ {
		p := vector.Add(
			*vector.New(rn(-500, 500), rn(-500, 500)),
			*vector.New(W/2, H/2),
		)
		g := vector.Add(
			p,
			*vector.New(rn(-100, 100), rn(-100, 100)),
		)

		a := demo.New(
			demo.O{
				P: p,
				G: g,
				S: rn(10, S),
				R: rn(5, R),
			},
		)

		agents = append(agents, a)
	}
	return agents
}

func GenerateLineCollision() []agent.A {
	offset := *vector.New(W/2, H/2)
	ps := []vector.V{
		vector.Add(*vector.New(-50, 0), offset),
		vector.Add(*vector.New(50, 0), offset),
	}

	agents := []agent.A{
		demo.New(
			demo.O{
				P: ps[0],
				G: vector.Add(ps[0], *vector.New(100, 0)),
			},
		),
		demo.New(
			demo.O{
				P: ps[1],
				G: vector.Add(ps[1], *vector.New(-100, 0)),
			},
		),
	}
	return agents
}

func main() {
	agents := GenerateRandomPoints(N)
	points := make([]point.P, 0, N)

	for _, a := range agents {
		points = append(points, *orca.New(a))
	}

	tr, err := kd.New(points)
	if err != nil {
		panic("cannot create K-D tree")
	}

	var images []*image.Paletted
	var delay []int

	for t := 0.; t < D; t += TAU {
		res, err := orca.Step(
			tr,
			TAU,
			func(a agent.A) bool { return true },
		)

		if err != nil {
			panic("error while stepping through ORCA")
		}

		img := image.NewPaletted(
			image.Rectangle{
				image.Point{0, 0},
				image.Point{W, H},
			},
			[]color.Color{
				white,
				black,
				red,
				green,
				blue,
			},
		)
		for x := 0; x < W; x++ {
			for y := 0; y < H; y++ {
				img.Set(x, y, white)
			}
		}

		for _, m := range res {
			a := m.A.(*demo.A)

			var c color.Color = black

			// Vector has changed because Step() detected an
			// oncoming collision. Visually indicate this by
			// flashing the circle red.
			if !vector.Within(a.V(), vector.V(m.V)) {
				c = red
			}

			if vector.Within(a.T(), *vector.New(0, 0)) {
				c = blue
			}

			// Draw agent goals.
			DrawCircle(img, a.G(), 2, green)

			// Draw agents.
			DrawCircle(img, a.P(), int(a.R()), c)

			// Draw agent vision radii.
			DrawCircle(img, a.P(), int(util.R(a, TAU)), green)

			a.SetP(
				vector.Add(
					a.P(),
					vector.Scale(TAU, vector.V(m.V)),
				),
			)
			a.SetV(vector.V(m.V))
		}

		images = append(images, img)
		delay = append(delay, 2)

		tr.Balance()
	}

	// Export animation as a gif.
	anim := &gif.GIF{
		Delay: delay,
		Image: images,
	}
	f, _ := os.Create("demo/output/animation.gif")
	gif.EncodeAll(f, anim)
}
