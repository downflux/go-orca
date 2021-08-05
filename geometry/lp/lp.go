package lp

// C defines a linear constraint of the form
//
//   A • X <= B
//
// For two dimensions, this is
//
//   a_x * x + a_y * y <= b
type C interface {
	// D refers to the dimension of the contraint, e.g. return 2 if the
	// contraint is 2D example above.
	D() int

	// A returns the A vector of the contraint; returns [a, b] in the 2D
	// case.
	A() []float64

	// B returns the bound on the constraint.
	B() float64
}
