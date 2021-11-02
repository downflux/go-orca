# orca
Golang implementation of the Optimal Reciprocal Collision Avoidance (ORCA)
algorithm

## Installation

```
go version
> go version go1.17 linux/amd64
```

## Updating

```
go get -u ./...
go mod tidy
```

## Testing

```
 go test -v github.com/downflux/orca/... -bench . -cpu 1,2,4,8,16,32,64
```

### Profiling

**N.B.**: WSL does not profile correctly. See
[golang/go#22366](https://github.com/golang/go/issues/22366).

```
go test -v github.com/downflux/orca/vo/ball -count=1 \
  -run '^BenchmarkORCA/VO$' \
  -bench '^BenchmarkORCA/VO$' \
  -benchmem \
  -cpuprofile cpu.out \
  -memprofile mem.out
go tool pprof cpu.out
go tool pprof mem.out
```

```
(pprof) top10
...
```

Alternatively print out the profile directly in the CLI, e.g.

```
go tool pprof -tree -nodecount=10 cpu.out
```

See [pprof](https://github.com/google/pprof/blob/master/README.md) for more
information.
