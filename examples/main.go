// Package main executes a short demo of ORCA and outputs a gif of the
// calculated trajectory of the set of agents.
//
// Example:
//
//	go run \
//	  examples/generator/main.go --mode=random | go run \
//	  examples/main.go > demo.gif
package main

import (
	"bufio"
	"flag"
	"image"
	"image/color"
	"image/gif"
	"io"
	"log"
	"math"
	"math/rand"
	"os"
	"runtime"

	"github.com/downflux/go-geometry/2d/line"
	"github.com/downflux/go-geometry/2d/segment"
	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/examples/config"
	"github.com/downflux/go-orca/orca"
	"github.com/downflux/go-orca/region"

	v2d "github.com/downflux/go-geometry/2d/vector"
	exampleagent "github.com/downflux/go-orca/examples/agent"
	examplesdraw "github.com/downflux/go-orca/examples/draw"
	examplesegment "github.com/downflux/go-orca/examples/segment"
)

const (
	Framerate = 60

	// ORCAInterval dictates how many frames to skip before calling ORCA.
	//
	// Note that this linearly correlates with time.
	ORCAInterval = 1
)

var (
	// Tau is a time interval passed into the ORCA simulation; this scalar
	// is used in conjunction with the agent speed to help determine an
	// event horizon for the simulated agents. Agents beyond this event
	// horizon are not considered for collision detection.
	//
	// If a neighbor of an agent is moving at a much greater max speed, it
	// is possible with a small enough event horizon for a collision event
	// to occur. In order to guarantee these types of collisions do not
	// occur, we need to set the event horizon radius to the maximum
	// traversable distance in between ORCA steps, but in reality we can set
	// the actual event horizon radius to be a bit smaller than this.
	//
	// The time in between ORCA steps can be calculated via
	//
	//   ORCAInterval / Framerate
	//
	// If we set 𝜏 too small however, agent-agent collision detection occurs
	// at the very last moment, which will result in unnatural simulations.
	// The value here was determined experimentally to look good.
	Tau = (0.9 * Framerate) * float64(ORCAInterval) / Framerate
)

var (
	// Color palette for drawing.

	black = color.Black
	white = color.White
	red   = color.RGBA{255, 0, 0, 255}
	green = color.RGBA{0, 255, 0, 255}
	blue  = color.RGBA{0, 0, 255, 255}
	gray  = color.RGBA{192, 192, 192, 255}

	// Various binary flags.

	out    = flag.String("o", "/dev/stdout", "output file path, e.g. path/to/output.gif")
	in     = flag.String("i", "/dev/stdin", "input file path, e.g. path/to/config.json")
	frames = flag.Int("frames", 120, "number of frames to render")

	// Interface checks to demonstrate the functionality P is fulfilling in
	// the demo.
	_ point.P = &P{}

	// margin is the minimum size of the rectangle drawn on screen.
	margin = *v2d.New(50, 50)
)

type P exampleagent.A

func (p *P) A() agent.A  { return (*exampleagent.A)(p) }
func (p *P) P() vector.V { return vector.V((*exampleagent.A)(p).P()) }

func rn(min float64, max float64) float64 { return rand.Float64()*(max-min) + min }

type Env struct {
	ps []*P
	ss []region.R
}

func New(data []byte) *Env {
	c := config.Unmarshal(data)
	points := make([]*P, 0, len(c.Agents))
	segments := make([]region.R, 0, len(c.Segments))

	for _, o := range c.Agents {
		a := *exampleagent.New(o)
		p := P(a)
		points = append(points, &p)
	}

	for _, o := range c.Segments {
		segments = append(segments, *examplesegment.New(o))
	}

	return &Env{
		ps: points,
		ss: segments,
	}
}

func (e *Env) Points() []*P         { return e.ps }
func (e *Env) Segments() []region.R { return e.ss }

// bound calculates the bounding rectangle around all agents.
func bound(e *Env) hyperrectangle.R {
	min := *v2d.New(math.Inf(0), math.Inf(0))
	max := *v2d.New(math.Inf(-1), math.Inf(-1))

	for _, p := range e.Points() {
		p := v2d.V(p.P())
		min = *v2d.New(
			math.Min(min.X(), p.X()),
			math.Min(min.Y(), p.Y()),
		)
		max = *v2d.New(
			math.Max(max.X(), p.X()),
			math.Max(max.Y(), p.Y()),
		)
	}

	for _, r := range e.Segments() {
		for _, s := range r.R() {
			min = *v2d.New(
				math.Min(
					min.X(),
					math.Min(
						s.L().L(s.TMin()).X(),
						s.L().L(s.TMax()).X(),
					),
				),
				math.Min(
					min.Y(),
					math.Min(
						s.L().L(s.TMin()).Y(),
						s.L().L(s.TMax()).Y(),
					),
				),
			)
			max = *v2d.New(
				math.Max(
					max.X(),
					math.Max(
						s.L().L(s.TMin()).X(),
						s.L().L(s.TMax()).X(),
					),
				),
				math.Max(
					max.Y(),
					math.Max(
						s.L().L(s.TMin()).Y(),
						s.L().L(s.TMax()).Y(),
					),
				),
			)
		}
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

	// Read agent configuration from specified input file.
	r, err := os.Open(*in)
	if err != nil {
		log.Fatalf("cannot open file %v: %v", *in, err)
	}
	data, err := bufio.NewReader(r).ReadBytes(byte(0))
	if err != io.EOF {
		log.Fatalf("could not read from file %v: %v", *in, err)
	}
	env := New(data)

	// Construct a new K-D tree for neighbor queries. Note the scope of this
	// variable -- in real applications, this tree is very useful for
	// operations not strictly limited to ORCA, e.g. for calculating nearest
	// neighbors for fog-of-war calculations.
	tr := kd.New(kd.O[*P]{
		Data: env.Points(),
		K:    2,
		N:    16,
	})
	if err != nil {
		log.Fatalf("cannot create K-D tree")
	}

	var images []*image.Paletted
	var delay []int

	b := bound(env)

	// trailbuf keeps the last N positions of agents in memory for
	// visualization.
	var trailbuf [50][]v2d.V

	// Run the simulator for some steps.
	for i := 0; i < *frames; i++ {
		// Overwrite trail buffer
		trailbuf[i%len(trailbuf)] = nil
		for _, p := range env.Points() {
			trailbuf[i%len(trailbuf)] = append(trailbuf[i%len(trailbuf)], p.A().P())
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
		for _, buf := range trailbuf {
			examplesdraw.Trail(img, margin, buf, gray)
		}

		// Draw agents.
		for _, p := range env.Points() {
			a := p.A().(*exampleagent.A)

			// Draw agent goal positions.
			examplesdraw.Circle(img, v2d.Add(margin, a.G()), 2, green)

			// Draw circle.
			examplesdraw.Circle(img, v2d.Add(margin, a.P()), int(a.R()), black)
		}

		// Draw lines.
		for _, r := range env.Segments() {
			for _, s := range r.R() {
				examplesdraw.Line(
					img,
					*segment.New(
						*line.New(
							v2d.Add(margin, s.L().P()),
							s.L().D(),
						),
						s.TMin(),
						s.TMax(),
					),
					black,
				)
			}
		}

		// ORCA may be run at a slower rate than the tick rate.
		if i%ORCAInterval == 0 {
			res, err := orca.Step(orca.O[*P]{
				T:   tr,
				R:   env.Segments(),
				Tau: Tau,
				F:   func(a agent.A) bool { return true },
				// We found this is the fastest configuration
				// via benchmarking.
				PoolSize: 4 * runtime.GOMAXPROCS(0),
			})
			if err != nil {
				log.Fatalf("error while stepping through ORCA: %v", err)
			}
			for _, m := range res {
				a := m.A.(*exampleagent.A)
				a.SetV(m.V)
			}
		}

		// Run simulation for the current server tick.
		for _, p := range env.Points() {
			a := p.A().(*exampleagent.A)
			a.SetP(
				v2d.Add(
					a.P(),
					v2d.Scale(1/float64(Framerate), a.V()),
				),
			)
		}

		// Render with approximately 2/100 s delay, i.e. at 50Hz.
		images = append(images, img)
		delay = append(delay, 2)

		// Update the K-D tree, as positions have changed in the
		// interim. Note that the K-D tree is mutated outside of calling
		// ORCA -- this allows the tree to be used elsewhere, as it's a
		// very useful struct for other range queries.
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
