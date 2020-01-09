// ------------------------------
// Written by Suleyman TARSUSLU
// ------------------------------
#ifndef HOMOGRAPHY_HPP
#define HOMOGRAPHY_HPP

namespace ceng391 {

// Matches should contain 4 x <-> x' pairs in the following order:
// matches = { x0, y0, x'0 y'0, x1, y1, x'1 y'1,
//             x2, y2, x'2 y'2, x3, y3, x'3 y'3  }
void fit_homography4(const double *matches, double *h);

}

#endif
