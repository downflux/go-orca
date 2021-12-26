# go-orca
Golang implementation of the Optimal Reciprocal Collision Avoidance (ORCA)
algorithm

## Disclaimer

This project is under active development and is not yet feature complete, and
may contain bugs. We welcome contributions in the form of new issues and pull
requests.

This project was created after endlessly consulting the canonical implementation
[github.com/snape/RVO2](https://github.com/snape/RVO2) and follows the general
shape of the reference. General improvements lie in abstracting away some code
and documenting a number of assumptions the reference implementation makes.

## Background

ORCA is useful for local collision avoidance in large systems.  This repository
aims to be an implementation of the ORCA algorithm with much improved
documentation and API.

More prosaic documentation of this library will be made available at
[blog.downflux.com](https://blog.downflux.com) soon.

## Installation

```bash
$ go version

go version go1.17.4 linux/amd64
```

## Updating

```bash
$ go get -u ./...
$ go mod tidy
```

## Demo

```bash
$ go run \
  examples/generator/main.go --mode=grid | go run \
  examples/main.go --frames=1250 > demo.gif
```

![ORCA demo](examples/output/animation.gif)

## Profiling

**N.B.**: WSL does not profile correctly. See
[golang/go#22366](https://github.com/golang/go/issues/22366).

```bash
$ go test -v \
  github.com/downflux/go-orca/... \
  -bench . \
  -benchmem -cpu 1,2,4,8,16,32,64

$ go test -v \
  github.com/downflux/go-orca/orca \
  -bench BenchmarkStep/N=1000000 \
  -benchmem \
  -cpuprofile cpu.out
  -memprofile mem.out

$ go tool pprof -tree -nodecount=10 cpu.out
```

See [pprof](https://github.com/google/pprof/blob/master/README.md) for more
information.

### Sample Metrics

```bash
$ go test github.com/downflux/go-orca/orca -bench .

goos: linux
goarch: amd64
pkg: github.com/downflux/go-orca/orca
cpu: Intel(R) Core(TM) i7-6700K CPU @ 4.00GHz
BenchmarkStep/PoolSize=1/N=1000-8                    201           5874149 ns/op
BenchmarkStep/PoolSize=2/N=1000-8                    304           3818542 ns/op
BenchmarkStep/PoolSize=4/N=1000-8                    469           2760155 ns/op
BenchmarkStep/PoolSize=8/N=1000-8                    638           2163011 ns/op
BenchmarkStep/PoolSize=16/N=1000-8                   778           1790053 ns/op
BenchmarkStep/PoolSize=32/N=1000-8                   759           1797966 ns/op
BenchmarkStep/PoolSize=64/N=1000-8                   758           1837833 ns/op
BenchmarkStep/PoolSize=1/N=10000-8                     7         164411857 ns/op
BenchmarkStep/PoolSize=2/N=10000-8                    13          96502769 ns/op
BenchmarkStep/PoolSize=4/N=10000-8                    20          64978580 ns/op
BenchmarkStep/PoolSize=8/N=10000-8                    26          54633023 ns/op
BenchmarkStep/PoolSize=16/N=10000-8                   27          54048937 ns/op
BenchmarkStep/PoolSize=32/N=10000-8                   30          50821777 ns/op
BenchmarkStep/PoolSize=64/N=10000-8                   28          52647196 ns/op
BenchmarkStep/PoolSize=1/N=100000-8                    1        29744473100 ns/op
BenchmarkStep/PoolSize=2/N=100000-8                    1        17573546400 ns/op
BenchmarkStep/PoolSize=4/N=100000-8                    1        12624980100 ns/op
BenchmarkStep/PoolSize=8/N=100000-8                    1        10821498000 ns/op
BenchmarkStep/PoolSize=16/N=100000-8                   1        10191115200 ns/op
BenchmarkStep/PoolSize=32/N=100000-8                   1        10799581500 ns/op
BenchmarkStep/PoolSize=64/N=100000-8                   1        11008062500 ns/op
PASS
ok      github.com/downflux/go-orca/orca        134.500s
```

### Performance

Performance metrics shoud be compared against [Granberg][1], [Snape et al.][2],
and [van den Berg et al.][3]. We estimate that there is about another 50%
optimization achievable in the current implementation of the ORCA algorithm.

## Example

```golang
package main

import (
	"fmt"

	"github.com/downflux/go-geometry/nd/vector"
	"github.com/downflux/go-kd/kd"
	"github.com/downflux/go-kd/point"
	"github.com/downflux/go-orca/agent"
	"github.com/downflux/go-orca/orca"

	v2d "github.com/downflux/go-geometry/2d/vector"
	okd "github.com/downflux/go-orca/kd"
)

// Define a K-D tree point which satisfies ORCA's P interface as well.
//
// Ensure the point is mutable, as we will be updating the point velocities.
type a struct {
	// r is the collision radius of the agent.
	r float64

	// s is the max speed of the agent.
	s float64

	// p is the current position of the agent.
	p v2d.V

	// v is the current veloicty of the agent.
	v v2d.V

	// t is the target velocity of the agent.
	t v2d.V
}

func (a *a) P() v2d.V   { return a.p }
func (a *a) V() v2d.V   { return a.v }
func (a *a) T() v2d.V   { return a.t }
func (a *a) R() float64 { return a.r }
func (a *a) S() float64 { return a.s }

type p a

func (p *p) A() agent.A  { return (*a)(p) }
func (p *p) P() vector.V { return vector.V((*a)(p).P()) }

// Check interface fulfilment.
//
// Ensure that our point fulfills the ORCA K-D tree data point interface.
var (
	_ okd.P = &p{}

	// These checks are technically unnecessary (okd.P is a superset of
	// point.P), but we're adding the check here to facilitate the reader's
	// understanding of the underlying types being used in this example.
	_ point.P = &p{}
	_ agent.A = &a{}
)

func main() {
	// Construct some agents.
	agents := []point.P{
		&p{
			r: 1,
			// Max speed just means the agent has the capability to
			// move this fast, but the goal of ORCA is to minimize
			// the difference to the target velocity instead.
			s: 10,
			p: *v2d.New(0, 0),
			v: *v2d.New(0, 1),
			t: *v2d.New(0, 1),
		},
		&p{
			r: 1,
			s: 10,
			p: *v2d.New(0, 5),
			v: *v2d.New(0, -1),
			t: *v2d.New(0, -1),
		},
	}

	// Create a K-D tree which tracks agent-agent proximity.
	//
	// N.B.: This tree will need to be rebalanced if velocities are updated,
	// which takes a non-trivial amount of time. This additional time is not
	// accounted for in the ORCA performance tests. In the course of a
	// simulation, we would expect multiple other (user-defined) steps to
	// occur, and so have exposed the tree rebalance call.
	t, _ := kd.New(agents)

	// Simulate one loop. Step() is a pure function and does not mutate any
	// state -- the caller will need to manually set the position and
	// velocity vectors. Or not, im_a_sign_not_a_cop.jpg.
	//
	// Note that we are manually casting the base K-D tree into the
	// ORCA-specific K-D tree.
	mutations, _ := orca.Step(orca.O{
		T: okd.Lift(t),
		// Pick a sensible value for the lookhead -- this depends on how
		// fast the agents are travelling per tick.
		Tau:      10,
		F:        func(a agent.A) bool { return true },
		PoolSize: 2,
	})
	for _, m := range mutations {
		a := m.A.(*a)
		fmt.Printf(
			"input velocity for agent at position %v is %v, but output velocity is %v\n",
			a.P(),
			a.V(),
			m.V,
		)
	}
}
```

## TODO

We have not yet implemented generating velocity objects for polygonal obstacles.
The current implementation only adjusts trajectory for other circular agents.

[1]: https://arongranberg.com/astar/docs_beta/local-avoidance.html
[2]: https://www.intel.com/content/www/us/en/developer/articles/technical/reciprocal-collision-avoidance-and-navigation-for-video-games.html
[3]: http://emotion.inrialpes.fr/fraichard/safety2010/10-vandenberg-etal-icraw.pdf
[4]: https://github.com/snape/RVO2
