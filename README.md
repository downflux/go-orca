# orca
Golang implementation of the Optimal Reciprocal Collision Avoidance (ORCA) algorithm

## Installation

```
go version
> go version go1.16.6 linux/amd64
```

## Testing

```
go test ./... -bench . -cpu 1,2,4,8
```

### Profiling

```
go test -v github.com/downflux/orca/vo/ball -count=1 \
  -run '^BenchmarkORCA/VO$' \
  -bench '^BenchmarkORCA/VO$' \
  -cpuprofile cpu.out
go tool pprof cpu.out
```

```
(pprof) top10
...
```
