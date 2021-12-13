// Package main executes a short demo of ORCA and outputs a gif of the
// calculated trajectory of the set of agents.
//
// Example:
//
//   go run \
//     demo/generator/main.go --mode=random | go run \
//     demo/main.go > demo/output/animation.gif
package main

import (
	"bufio"
	"flag"
	"image"
	"image/color"
	"image/draw"
	"image/gif"
	"io"
	"log"
	"math"
	"math/rand"
	"os"
	"runtime"

	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/demo/generator/generator"
	"github.com/downflux/go-orca/orca"

	v2d "github.com/downflux/go-geometry/2d/vector"
	demo "github.com/downflux/go-orca/demo/agent"
)

const (
	Framerate = 60

	// ORCAInterval dictates how many frames to skip before calling ORCA.
	//
	// Note that this linearly correlates with time.
	ORCAInterval = 1
)

var (
	// Tau is passed into ORCA and dictates the event horizon of the VO
	// object. We expand the event horizon slightly compared to the
	// ORCAInterval to help smooth out the velocity changes a bit.
	Tau = 50 * float64(ORCAInterval) / Framerate
)

var (
	black = color.Black
	white = color.White
	red   = color.RGBA{255, 0, 0, 255}
	green = color.RGBA{0, 255, 0, 255}
	blue  = color.RGBA{0, 0, 255, 255}
	gray  = color.RGBA{192, 192, 192, 255}
)

var (
	out    = flag.String("o", "/dev/stdout", "output file path, e.g. path/to/output.gif")
	in     = flag.String("i", "/dev/stdin", "input file path, e.g. path/to/config.json")
	frames = flag.Int("frames", 120, "number of frames to render")
)

func rn(min float64, max float64) float64 { return rand.Float64()*(max-min) + min }

func drawCircle(img draw.Image, v v2d.V, r int, c color.Color) {
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

func generate(data []byte) []agent.A {
	opts := generator.Unmarshal(data)
	agents := make([]agent.A, 0, len(opts))

	for _, o := range opts {
		agents = append(agents, demo.New(o))
	}
	return agents
}

var (
	margin = *v2d.New(50, 50)
)

func bound(agents []agent.A) hyperrectangle.R {
	min := *v2d.New(math.Inf(0), math.Inf(0))
	max := *v2d.New(math.Inf(-1), math.Inf(-1))

	for _, a := range agents {
		min = *v2d.New(
			math.Min(min.X(), a.P().X()),
			math.Min(min.Y(), a.P().Y()),
		)
		max = *v2d.New(
			math.Max(max.X(), a.P().X()),
			math.Max(max.Y(), a.P().Y()),
		)
	}

	return *hyperrectangle.New(
		*vector.New(0, 0),
		vector.V(
			v2d.Add(
				v2d.Scale(2, margin),
				v2d.Sub(max, min),
			),
		),
	)
}

func main() {
	flag.Parse()

	r, err := os.Open(*in)
	if err != nil {
		log.Fatalf("cannot open file %v: %v", *in, err)
	}

	data, err := bufio.NewReader(r).ReadBytes(byte(0))
	if err != io.EOF {
		log.Fatalf("could not read from file %v: %v", *in, err)
	}

	agents := generate(data)
	points := make([]point.P, 0, len(agents))

	for _, a := range agents {
		points = append(points, *orca.New(a))
	}

	// Construct a new K-D tree for neighbor queries. Note the scope of this
	// variable -- in real applications, this tree is very useful for
	// operations not strictly limited to ORCA, e.g. for calculating nearest
	// neighbors for fog-of-war calculations.
	tr, err := kd.New(points)
	if err != nil {
		log.Fatalf("cannot create K-D tree")
	}

	var images []*image.Paletted
	var delay []int

	b := bound(agents)

	// trail is a buffer of the last N positions of agents.
	var trail [50][]v2d.V

	// Run the simulator for some steps.
	for i := 0; i < *frames; i++ {
		trail[i%len(trail)] = nil
		for _, a := range agents {
			trail[i%len(trail)] = append(trail[i%len(trail)], a.P())
		}

		img := image.NewPaletted(
			image.Rectangle{
				image.Point{
					int(b.Min().X(vector.AXIS_X)),
					int(b.Min().X(vector.AXIS_Y)),
				},
				image.Point{
					int(b.Max().X(vector.AXIS_X)),
					int(b.Max().X(vector.AXIS_Y)),
				},
			},
			[]color.Color{
				white,
				black,
				red,
				green,
				blue,
				gray,
			},
		)

		// Draw historical agent paths.
		for _, buf := range trail {
			for _, p := range buf {
				p := v2d.Add(margin, p)
				img.Set(int(p.X()), int(p.Y()), gray)
			}
		}

		if i%ORCAInterval == 0 {
			res, err := orca.Step(orca.O{
				T:   tr,
				Tau: Tau,
				F:   func(a agent.A) bool { return true },
				// We found this is the fastest configuration
				// via benchmarking.
				PoolSize: 8 * runtime.GOMAXPROCS(0),
			})
			if err != nil {
				log.Fatalf("error while stepping through ORCA: %v", err)
			}
			for _, m := range res {
				a := m.A.(*demo.A)
				a.SetV(v2d.V(m.V))
			}
		}

		for _, a := range agents {
			a := a.(*demo.A)

			// Draw agent goals.
			drawCircle(img, v2d.Add(margin, a.G()), 2, green)

			// Draw agents.
			drawCircle(img, v2d.Add(margin, a.P()), int(a.R()), black)

			a.SetP(
				v2d.Add(
					a.P(),
					v2d.Scale(1/float64(Framerate), a.V()),
				),
			)
		}

		images = append(images, img)

		// Render with approximately 2/100 s delay, i.e. at 50Hz.
		delay = append(delay, 2)

		tr.Balance()
	}

	// Export animation as a gif.
	anim := &gif.GIF{
		Delay: delay,
		Image: images,
	}

	w, err := os.Create(*out)
	if err != nil {
		log.Fatalf("cannot write to file %v: %v", *out, err)
	}

	gif.EncodeAll(w, anim)
}
