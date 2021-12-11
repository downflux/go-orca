# go-orca
Golang implementation of the Optimal Reciprocal Collision Avoidance (ORCA)
algorithm

## Disclaimer

This project is under active development and is not yet feature complete, and
may contain bugs. We welcome contributions in the form of new issues and pull
requests.

## Background

ORCA is useful for local collision avoidance in large systems. The current
["canonical" implementation][4] lacks documentation, and is rather opaque.
go-orca aims to be a re-implementation of the ORCA algorithm with better
documentation and API.

More prosaic documentation of this library will be made available at
[blog.downflux.com](https://blog.downflux.com) soon.

## Installation

```bash
go version
> go version go1.17.4 linux/amd64
```

## Updating

```bash
go get -u ./...
go mod tidy
```

## Demo

```bash
go run \
  demo/generator/main.go --mode=random | go run \
  demo/main.go > demo.gif
```

![ORCA demo](demo/output/animation.gif)

Here, we have 250 agents of random size and speeds travelling in 2D ambient
space to some random nearby destination. Green circles indicate agent vision
radius, whereas an agent (in black) flashing red indicates the velocity has
changed due to ORCA.

## Profiling

**N.B.**: WSL does not profile correctly. See
[golang/go#22366](https://github.com/golang/go/issues/22366).

```bash
go test -v \
  github.com/downflux/go-orca/... \
  -bench . \
  -benchmem -cpu 1,2,4,8,16,32,64

go test -v \
  github.com/downflux/go-orca/orca \
  -bench BenchmarkStep/N=1000000 \
  -benchmem \
  -cpuprofile cpu.out
  -memprofile mem.out

go tool pprof -tree -nodecount=10 cpu.out
```

See [pprof](https://github.com/google/pprof/blob/master/README.md) for more
information.

### Sample Metrics

```bash
go test github.com/downflux/go-orca/orca -bench .

goos: linux
goarch: amd64
pkg: github.com/downflux/go-orca/orca
cpu: Intel(R) Core(TM) i7-6700K CPU @ 4.00GHz
BenchmarkStep/N=1-8               965041              1229 ns/op
BenchmarkStep/N=10-8               69074             18382 ns/op
BenchmarkStep/N=100-8               4786            260644 ns/op
BenchmarkStep/N=1000-8               373           3262935 ns/op
BenchmarkStep/N=10000-8               28          41041814 ns/op
BenchmarkStep/N=100000-8               3         518004500 ns/op
BenchmarkStep/N=1000000-8              1        5991109700 ns/op
PASS
ok      github.com/downflux/go-orca/orca        38.640s
```

## TODO

We have not yet implemented generating velocity objects for polygonal obstacles.
The current implementation only adjusts trajectory for other circular agents.

## Performance

Performance metrics shoud be compared against [Granberg][1], [Snape et al.][2],
and [van den Berg et al.][3]. We estimate that there is about another 50%
optimization achievable in the current implementation of the ORCA algorithm.

[1]: https://arongranberg.com/astar/docs_beta/local-avoidance.html
[2]: https://www.intel.com/content/www/us/en/developer/articles/technical/reciprocal-collision-avoidance-and-navigation-for-video-games.html
[3]: http://emotion.inrialpes.fr/fraichard/safety2010/10-vandenberg-etal-icraw.pdf
[4]: https://github.com/snape/RVO2
