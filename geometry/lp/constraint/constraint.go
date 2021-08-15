package constraint

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

type CImpl struct {
	a []float64
	b float64
}

func New(a []float64, b float64) *CImpl { return &CImpl{a: a, b: b} }

func (c CImpl) D() int       { return len(c.a) }
func (c CImpl) A() []float64 { return c.a }
func (c CImpl) B() float64   { return c.b }
