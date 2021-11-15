module github.com/downflux/go-orca

go 1.17

require (
	github.com/downflux/go-geometry v0.2.1-0.20211107235139-65ac6d75a4cf
	google.golang.org/grpc v1.42.0
)

require (
	github.com/golang/protobuf v1.5.2 // indirect
	google.golang.org/genproto v0.0.0-20211104193956-4c6863e31247 // indirect
	google.golang.org/protobuf v1.27.1 // indirect
)

replace github.com/downflux/go-geometry => ../go-geometry
