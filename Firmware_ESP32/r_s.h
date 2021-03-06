// Height offset of sensor chip, found by optimisation
#define DDT 0.000224739682F

// Sensor position within the square 3x3 module matrix
// x         y         z       module nr. and field component
const float r_s[] = {
0.00508F, 0.00508F, 0.000500F-DDT, // Module-1-z
0.00508F, 0.00939F-DDT, -0.00100F, // Module-1-y
0.00939F-DDT, 0.00508F, -0.00100F, // Module-1-x
0.01778F, 0.00508F, 0.000500F-DDT, // Module-2-z
0.01778F, 0.00939F-DDT, -0.00100F, // Module-2-y
0.02209F-DDT, 0.00508F, -0.00100F, // Module-2-x
0.03048F, 0.00508F, 0.000500F-DDT, // Module-3-z
0.03048F, 0.00939F-DDT, -0.00100F, // Module-3-y
0.03479F-DDT, 0.00508F, -0.00100F, // Module-3-x
0.00508F, 0.01778F, 0.000500F-DDT, // Module-4-z
0.00508F, 0.02209F-DDT, -0.00100F, // Module-4-y
0.00939F-DDT, 0.01778F, -0.00100F, // Module-4-x
0.01778F, 0.01778F, 0.000500F-DDT, // Module-5-z
0.01778F, 0.02209F-DDT, -0.00100F, // Module-5-y
0.02209F-DDT, 0.01778F, -0.00100F, // Module-5-x
0.03048F, 0.01778F, 0.000500F-DDT, // Module-6-z
0.03048F, 0.02209F-DDT, -0.00100F, // Module-6-y
0.03479F-DDT, 0.01778F, -0.00100F, // Module-6-x
0.00508F, 0.03048F, 0.000500F-DDT, // Module-7-z
0.00508F, 0.03479F-DDT, -0.00100F, // Module-7-y
0.00939F-DDT, 0.03048F, -0.00100F, // Module-7-x
0.01778F, 0.03048F, 0.000500F-DDT, // Module-8-z
0.01778F, 0.03479F-DDT, -0.00100F, // Module-8-y
0.02209F-DDT, 0.03048F, -0.00100F, // Module-8-x
0.03048F, 0.03048F, 0.000500F-DDT, // Module-9-z
0.03048F, 0.03479F-DDT, -0.00100F, // Module-9-y
0.03479F-DDT, 0.03048F, -0.00100F  // Module-9-x
};
