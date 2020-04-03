// -*-c++-*-

/*!
  \file angle_deg.cpp
  \brief Degree wrapper class Source File.
*/

/*
 *Copyright:

 Copyright (C) 2004 Hidehisa Akiyama

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "parsian_util/geom/angle_deg.h"

#include <algorithm>
#include <vector>

#ifndef M_PI
//! PI value macro
#define M_PI 3.14159265358979323846
#endif

#ifndef EPSILON
#define EPSILON 0.0001
#endif
namespace rcsc {

const double AngleDeg::EPSILOON = 1.0e-5;

const double AngleDeg::DEG2RAD = M_PI / 180.0;
const double AngleDeg::RAD2DEG = 180.0 / M_PI;

/*-------------------------------------------------------------------*/
/*!

 */
bool
AngleDeg::isWithin(const AngleDeg & left,
                   const AngleDeg & right) const {
    // left to right arc angle is less than 180 degree.
    if (left.isLeftEqualOf(right)) {
        if (left.isLeftEqualOf(*this) && this->isLeftEqualOf(right)) {
            return true;
        }
    }
    // arc angle is more than 180 degree.
    else {
        // check out reverse side
        //if ( *this <= right || left <= *this )
        // == !(right < *this && *this < left)
        if (this->isLeftEqualOf(right) || left.isLeftEqualOf(*this)) {
            return true;
        }
    }
    return false;
}

/*-------------------------------------------------------------------*/
/*!

 */
void
AngleDeg::sinMinMax(const double & angle_err,
                    double * minsin,
                    double * maxsin) const {
    if (angle_err < 0.0 || 180.0 < angle_err) {
        std::cerr << "AngleDeg::sinMinMax() invalid error range. "
                  << angle_err << std::endl;
        *minsin = -1.0;
        *maxsin = 1.0;
        return;
    }

    double mindir = this->degree() - angle_err;
    double maxdir = this->degree() + angle_err;

    std::vector< double > sol;
    sol.reserve(4);

    if ((mindir < -90.0 && -90.0 < maxdir)
            || (mindir < 270.0 && 270.0 < maxdir)
       ) {
        sol.push_back(-1.0);
    }

    if ((mindir < 90.0 && 90.0 < maxdir)
            || (mindir < -270.0 && -270.0 < maxdir)
       ) {
        sol.push_back(1.0);
    }

    sol.push_back(AngleDeg::sin_deg(mindir));
    sol.push_back(AngleDeg::sin_deg(maxdir));

    *minsin = *std::min_element(sol.begin(), sol.end());
    *maxsin = *std::max_element(sol.begin(), sol.end());
}

/*-------------------------------------------------------------------*/
/*!

*/
void
AngleDeg::cosMinMax(const double & angle_err,
                    double * mincos,
                    double * maxcos) const {
    if (angle_err < 0.0 || 180.0 < angle_err) {
        std::cerr << "AngleDeg::cosMinMax() invalid error range. "
                  << angle_err << std::endl;
        *mincos = -1.0;
        *maxcos = 1.0;
        return;
    }

    double mindir = this->degree() - angle_err;
    double maxdir = this->degree() + angle_err;

    std::vector< double > sol;
    sol.reserve(4);

    if (mindir < -180.0 && -180.0 < maxdir) {
        sol.push_back(-1.0);
    }

    if (mindir < 0.0 && 0.0 < maxdir) {
        sol.push_back(1.0);
    }

    sol.push_back(AngleDeg::cos_deg(mindir));
    sol.push_back(AngleDeg::cos_deg(maxdir));

    *mincos = *std::min_element(sol.begin(), sol.end());
    *maxcos = *std::max_element(sol.begin(), sol.end());
}


/*-------------------------------------------------------------------*/
/*!

 */
AngleDeg
AngleDeg::bisect(const AngleDeg & left,
                 const AngleDeg & right) {
    AngleDeg result(left);
    AngleDeg rel(right - left);
    double half_deg = rel.degree() * 0.5;
    result += half_deg;

    if (left.isLeftOf(right)) {
        return result;
    } else {
        return result += 180.0;
    }
}

} // end of namespace





// -*-c++-*-

/*!
  \file circle_2d.cpp
  \brief 2D circle region Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa Akiyama

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "parsian_util/geom/circle_2d.h"
#include "parsian_util/geom/triangle_2d.h"
#include "parsian_util/geom/ray_2d.h"
#include "parsian_util/geom/line_2d.h"
#include "parsian_util/geom/segment_2d.h"

#include <iostream>
#include <cmath>

// available only this file
namespace {

/*-------------------------------------------------------------------*/
/*!
  \brief get squared value
  \param val input value
  \return squared value
 */
inline
double
SQUARE(const double & val) {
    return val * val;
}

/*-------------------------------------------------------------------*/
/*!
  \brief solve quadratic fomula
  \param a fomula constant A
  \param b fomula constant B
  \param c fomula constant C
  \param sol1 reference to the result variable
  \param sol2 reference to the result variable
  \return number of solution
 */
inline
int
QUADRATIC_FOMULA(const double & a,
                 const double & b,
                 const double & c,
                 double & sol1,
                 double & sol2) {
    double d = SQUARE(b) - 4.0 * a * c;
    // ignore small noise
    if (std::fabs(d) < 0.001) {
        sol1 = -b / (2.0 * a);
        return 1;
    } else if (d < 0.0) {
        return 0;
    } else {
        d = std::sqrt(d);
        sol1 = (-b + d) / (2.0 * a);
        sol2 = (-b - d) / (2.0 * a);
        return 2;
    }
}

} // end of namespace


namespace rcsc {

/*-------------------------------------------------------------------*/


const double Circle2D::EPSILOON = 1.0e-5;

/*-------------------------------------------------------------------*/
/*!

 */
int
Circle2D::intersection(const Line2D & line,
                       Vector2D * sol1,
                       Vector2D * sol2) const {
    if (std::fabs(line.a()) < EPSILOON) {
        if (std::fabs(line.b()) < EPSILOON) {
//            std::cerr << "Circle2D::intersection() illegal line."
//                      << std::endl;
            return 0;
        }

        // Line:    By + C = 0  ---> y = -C/B
        // Circle:  (x - cx)^2 + (y - cy)^2 = r^2
        // --->
        double x1 = 0.0, x2 = 0.0;
        int n_sol
            = QUADRATIC_FOMULA(1.0,
                               -2.0 * center().x,
                               (SQUARE(center().x)
                                + SQUARE(line.c() / line.b() + center().y)
                                - SQUARE(radius())),
                               x1,
                               x2);

        if (n_sol > 0) {
            double y1 = -line.c() / line.b();

            if (sol1) {
                sol1->assign(x1, y1);
            }

            if (n_sol > 1 && sol2) {
                sol2->assign(x2, y1);
            }
        }
        return n_sol;
    } else {
        // include (fabs(l.b()) < EPSILOON) case
        // use line & circle formula
        //   Ax + By + C = 0
        //   (x - cx)^2 + (y - cy)^2 = r^2
        // make y's quadratic formula using these fomula.
        double m = line.b() / line.a();
        double d = line.c() / line.a();

        double a = 1.0 + m * m;
        double b = 2.0 * (-center().y + (d + center().x) * m);
        double c = SQUARE(d + center().x)
                   + SQUARE(center().y)
                   - SQUARE(radius());

        double y1 = 0.0, y2 = 0.0;
        int n_sol = QUADRATIC_FOMULA(a, b, c,
                                     y1, y2);

        if (n_sol > 0 && sol1) {
            sol1->assign(line.getX(y1), y1);
        }

        if (n_sol > 1 && sol2) {
            sol2->assign(line.getX(y2), y2);
        }

        return n_sol;
    }
}

int Circle2D::tangent(Vector2D p, Vector2D * sol1, Vector2D * sol2) {
    double s = p.dist2(M_center);
    double r = M_radius * M_radius;
    if (s < r) {
        return 0;
    }
    if (s == r) {
        sol1->assign(p.x, p.y);
        return 1;
    }
    return intersection(Circle2D(p, sqrt(s - r)), sol1, sol2);
}

/*-------------------------------------------------------------------*/
/*!

 */
int
Circle2D::intersection(const Ray2D & ray,
                       Vector2D * sol1,
                       Vector2D * sol2) const {
    Line2D line(ray.origin(), ray.dir());
    Vector2D tsol1, tsol2;

    int n_sol = intersection(line, &tsol1, &tsol2);

    if (n_sol > 1
            && ! ray.inRightDir(tsol2, 1.0)) {
        --n_sol;
    }

    if (n_sol > 0
            && ! ray.inRightDir(tsol1, 1.0)) {
        tsol1 = tsol2; // substituted by second solution
        --n_sol;
    }

    if (n_sol > 0 && sol1) {
        *sol1 = tsol1;
    }

    if (n_sol > 1 && sol2) {
        *sol2 = tsol2;
    }

    return n_sol;
}


int
Circle2D::intersection(const Segment2D & seg,
                       Vector2D * sol1,
                       Vector2D * sol2) const {
    Line2D line = seg.line();
    Vector2D tsol1(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
    Vector2D tsol2(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);

    int n_sol = intersection(line, &tsol1, &tsol2);
    if (n_sol > 1) {
        if (seg.contains(tsol1) && seg.contains(tsol2)) {
            *sol1 = tsol1;
            *sol2 = tsol2;
            return 2;
        } else if (seg.contains(tsol1) && !seg.contains(tsol2)) {
            *sol1 = tsol1;
            *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
            return 1;
        } else if (seg.contains(tsol2) && !seg.contains(tsol1)) {
            *sol1 = tsol2;
            *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
            return 1;
        } else {
            *sol1 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
            *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
            return 0;
        }
    } else if (n_sol > 0) {
        if (tsol1.valid()) {
            *sol1 = tsol1;
            *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
            return 1;
        } else {
            *sol1 = tsol2;
            *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
            return 1;
        }
    } else {
        *sol1 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
        *sol2 = Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);
        return 0;
    }
}

/*-------------------------------------------------------------------*/
/*!

 */
int
Circle2D::intersection(const Circle2D & circle,
                       Vector2D * sol1,
                       Vector2D * sol2) const {
    double rel_x = circle.center().x - this->center().x;
    double rel_y = circle.center().y - this->center().y;

    double center_dist2 = rel_x * rel_x + rel_y * rel_y;
    double center_dist = std::sqrt(center_dist2);

    if (center_dist < std::fabs(this->radius() - circle.radius())
            || this->radius() + circle.radius() < center_dist) {
        return 0;
    }

    //std::cerr << "must exist intersection C1: " << this->center() << this->radius()
    //        << " C2: " << circle.center() << circle.radius()
    //        << std::endl;
    // line that passes through the intersection points
    Line2D line(-2.0 * rel_x,
                -2.0 * rel_y,
                circle.center().r2()
                - circle.radius() * circle.radius()
                - this->center().r2()
                + this->radius() * this->radius());

    return this->intersection(line, sol1, sol2);
}

/*-------------------------------------------------------------------*/
/*!

 */
Circle2D
Circle2D::circumcircle(const Vector2D & a,
                       const Vector2D & b,
                       const Vector2D & c) {
    Vector2D center = Triangle2D::circumcenter(a, b, c);

    if (! center.valid()) {
        std::cerr << "Circle2D::circumcircle()"
                  << " ***ERROR*** failed to get circumcenter from "
                  << a << b << c
                  << std::endl;
        return Circle2D();
    }

    return Circle2D(center, center.dist(a));
}

}








// -*-c++-*-

/*!
  \file line_2d.cpp
  \brief 2D straight line class Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa Akiyama

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "parsian_util/geom/line_2d.h"

#include <iostream>
#include <limits>

namespace rcsc {

const double Line2D::EPSILOON = 1.0e-5;
const double Line2D::ERROR_VALUE = std::numeric_limits< double >::max();

/*-------------------------------------------------------------------*/
/*!

 */
Vector2D
Line2D::intersection(const Line2D & line1,
                     const Line2D & line2) {
    double tmp = line1.a() * line2.b() - line1.b() * line2.a();
    if (std::fabs(tmp) < EPSILOON) {
        return Vector2D::INVALIDATED;
    }

    return Vector2D((line1.b() * line2.c() - line2.b() * line1.c()) / tmp,
                    (line2.a() * line1.c() - line1.a() * line2.c()) / tmp);
}

/*-------------------------------------------------------------------*/
/*!

 */
Line2D
Line2D::perpendicular_bisector(const Vector2D & p1,
                               const Vector2D & p2) {
    if (std::fabs(p2.x - p1.x) < EPSILOON
            && std::fabs(p2.y - p1.y) < EPSILOON) {
        // input points have same coordiate values.
        std::cerr << "Line2D::perpendicular_bisector."
                  << " ***ERROR*** input points have same coordinate values "
                  << p1 << p2
                  << std::endl;
    }

    double tmp = (p2.x * p2.x - p1.x * p1.x
                  + p2.y * p2.y - p1.y * p1.y) * -0.5 ;
    return Line2D(p2.x - p1.x,
                  p2.y - p1.y,
                  tmp);
}

}








// -*-c++-*-

/*!
  \file matrix_2d.cpp
  \brief 2D transform matrix class Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa Akiyama

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "parsian_util/geom/matrix_2d.h"

namespace rcsc {

/*-------------------------------------------------------------------*/
/*!

 */
Matrix2D
Matrix2D::inverted() const {
    double determinant = det();
    if (determinant == 0.0) {
        // never invertible
        return Matrix2D(); // default matrix
    }

    double dinv = 1.0 / determinant;
    return Matrix2D(M_22 * dinv, -M_12 * dinv,
                    -M_21 * dinv, M_11 * dinv,
                    (M_12 * M_dy - M_dx * M_22) * dinv,
                    (M_dx * M_21 - M_11 * M_dy) * dinv);
}

/*-------------------------------------------------------------------*/
/*!

 */
const
Matrix2D &
Matrix2D::rotate(const AngleDeg & angle) {
    // rotate matrix
    // R = ( cona, -sina, 0 )
    //     ( sina,  cosa, 0 )
    //     (    0,     0, 1 )

    // this = R * this
    // *this = create_rotation(angle) * *this;

    double sina = angle.sin();
    double cosa = angle.cos();

    double tm11 = M_11 * cosa - M_21 * sina;
    double tm12 = M_12 * cosa - M_22 * sina;
    double tm21 = M_11 * sina + M_21 * cosa;
    double tm22 = M_12 * sina + M_22 * cosa;
    double tdx = M_dx * cosa - M_dy * sina;
    double tdy = M_dx * sina + M_dy * cosa;
    M_11 = tm11;
    M_12 = tm12;
    M_dx = tdx;
    M_21 = tm21;
    M_22 = tm22;
    M_dy = tdy;
    return *this;
}


}







// -*-c++-*-

/*!
  \file polygon_2d.cpp
  \brief 2D polygon region Source File.
*/

/*
 *Copyright:

 Copyright (C) Hiroki Shimora

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

#ifdef HAVE_CONFIG
#include <config.h>
#endif

#include "parsian_util/geom/polygon_2d.h"
#include <parsian_util/geom/vector_2d.h>
#include <parsian_util/geom/segment_2d.h>
#include <parsian_util/geom/rect_2d.h>
#include <parsian_util/geom/line_2d.h>

#include <cmath>
#include <cfloat>
#include <cstdlib>
#include <iostream>



namespace rcsc {

/*-------------------------------------------------------------------*/
/*!

*/
Polygon2D::Polygon2D()
    : M_vertex() {
}

/*-------------------------------------------------------------------*/
/*!

*/
Polygon2D::Polygon2D(const std::vector< Vector2D > & v)
    : M_vertex(v) {
}

/*-------------------------------------------------------------------*/
/*!

*/
const Polygon2D &
Polygon2D::assign() {
    M_vertex.clear();
    return *this;
}

/*-------------------------------------------------------------------*/
/*!

*/
const Polygon2D &
Polygon2D::assign(const std::vector< Vector2D > & v) {
    M_vertex = v;
    return *this;
}

/*-------------------------------------------------------------------*/
/*!

*/
void
Polygon2D::addVertex(const Vector2D & p) {
    M_vertex.push_back(p);
}

/*-------------------------------------------------------------------*/
/*!

*/
const std::vector< Vector2D > &
Polygon2D::vertex() const {
    return M_vertex;
}

/*-------------------------------------------------------------------*/
/*!

*/
std::vector< Vector2D > &
Polygon2D::vertex() {
    return M_vertex;
}

/*-------------------------------------------------------------------*/
/*!

*/
Rect2D
Polygon2D::getBoundingBox() const {
    if (M_vertex.empty()) {
        return Rect2D();
    }

    double x_min = +DBL_MAX;
    double x_max = -DBL_MAX;
    double y_min = +DBL_MAX;
    double y_max = -DBL_MAX;

    for (size_t i = 0; i < M_vertex.size(); ++i) {
        const Vector2D & p = M_vertex[i];

        if (p.x > x_max) {
            x_max = p.x;
        }

        if (p.x < x_min) {
            x_min = p.x;
        }

        if (p.y > y_max) {
            y_max = p.y;
        }

        if (p.y < y_min) {
            y_min = p.y;
        }
    }

    return (Rect2D(x_min, y_min, (x_max - x_min), (y_max - y_min)));
}

/*-------------------------------------------------------------------*/
/*!

*/
Vector2D
Polygon2D::xyCenter() const {
    return this -> getBoundingBox().center();
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
Polygon2D::contains(const Vector2D & p,
                    bool allow_on_segment) const {
    if (M_vertex.empty()) {
        return false;
    } else if (M_vertex.size() == 1) {
        return allow_on_segment && (M_vertex[0] == p);
    }


    Rect2D r = this -> getBoundingBox();

    /*if ( ! r.contains( p ) )
    {
        return false;
    }*/


    //
    // make virtual half line
    //
    Segment2D line(p, Vector2D(p.x + ((r.maxX() - r.minX()
                                       + r.maxY() - r.minY())
                                      + (M_vertex[0] - p).r()) * 3.0,
                               p.y));

    //
    // check intersection with all segments
    //
    bool inside = false;
    double min_line_x = r.maxX() + 1.0;

    for (size_t i = 0; i < M_vertex.size(); ++i) {
        size_t p1_index = i + 1;

        if (p1_index >= M_vertex.size()) {
            p1_index = 0;
        }

        const Vector2D p0 = M_vertex[i];
        const Vector2D p1 = M_vertex[p1_index];

        if (! allow_on_segment) {
            if (Segment2D(p0, p1).onSegment(p)) {
                return false;
            }
        }

        if (allow_on_segment && p == p0) {
            return true;
        }

        if (line.existIntersection(Segment2D(p0, p1))) {
            if (p0.y == p.y
                    || p1.y == p.y) {
                if (p0.y == p.y) {
                    if (p0.x < min_line_x) {
                        min_line_x = p0.x;
                    }
                }

                if (p1.y == p.y) {
                    if (p1.x < min_line_x) {
                        min_line_x = p1.x;
                    }
                }


                if (p0.y == p1.y) {
                    continue;
                } else if (p0.y < p.y
                           || p1.y < p.y) {
                    continue;
                } else { // bottom point on the line
                    // no operation, don't skip
                }
            }

            inside = (! inside);
        }
    }

    return inside;
}


/*-------------------------------------------------------------------*/
/*!

*/
double
Polygon2D::dist(const Vector2D & p,
                bool check_as_plane) const {
    if (this -> vertex().size() == 1) {
        return (this -> vertex()[0] - p).r();
    }

    if (check_as_plane && this -> contains(p)) {
        return 0.0;
    }

    double min_dist = +DBL_MAX;

    for (size_t i = 0; i + 1 < this -> vertex().size(); ++i) {
        Segment2D seg(this -> vertex()[i],
                      this -> vertex()[i + 1]);

        double d = seg.dist(p);

        if (d < min_dist) {
            min_dist = d;
        }
    }

    if (this -> vertex().size() >= 3) {
        Segment2D seg(*(this -> vertex().rbegin()),
                      *(this -> vertex().begin()));

        double d = seg.dist(p);

        if (d < min_dist) {
            min_dist = d;
        }
    }

    // if this -> vertex().size() == 0, returns huge value

    return min_dist;
}

/*-------------------------------------------------------------------*/
/*!

*/
double
Polygon2D::area() const {
    return std::fabs(this -> signedArea2() / 2.0);
}

/*-------------------------------------------------------------------*/
/*!

*/
double
Polygon2D::signedArea2() const {
    if (M_vertex.size() < 3) {
        return 0.0;
    }

    double sum = 0.0;
    for (size_t i = 0; i < M_vertex.size(); ++i) {
        size_t n = i + 1;
        if (n == M_vertex.size()) {
            n = 0;
        }

        sum += (M_vertex[i].x * M_vertex[n].y - M_vertex[n].x * M_vertex[i].y);
    }

    return sum;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
Polygon2D::isCounterclockwise() const {
    return this -> signedArea2() > 0.0;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
Polygon2D::isClockwise() const {
    return this -> signedArea2() < 0.0;
}


/*-------------------------------------------------------------------*/
/*!

*/
template< class Predicate >
void
scissorWithLine(const Predicate & in_region,
                const std::vector< Vector2D > & points,
                std::vector< Vector2D > * new_points,
                const Line2D & line) {
    new_points -> clear();

    std::vector< bool > in_rectangle(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        in_rectangle[i] = in_region(points[i]);
    }

    for (size_t i = 0; i < points.size(); ++i) {
        size_t index_0 = i;
        size_t index_1 = i + 1;

        if (index_1 >= points.size()) {
            index_1 = 0;
        }

        const Vector2D & p0 = points[index_0];
        const Vector2D & p1 = points[index_1];

        if (in_rectangle[index_0]) {
            if (in_rectangle[index_1]) {
                new_points -> push_back(p1);
            } else {
                Vector2D c = line.intersection(Line2D(p0, p1));

                if (! c.valid()) {
                    std::cerr << "internal error:"
                              << " in rcsc::Polygon2D::scissorWithLine()"
                              << std::endl;

                    std::abort();
                }

                new_points -> push_back(c);
            }
        } else {
            if (in_rectangle[index_1]) {
                Vector2D c = line.intersection(Line2D(p0, p1));

                if (! c.valid()) {
                    std::cerr << "internal error:"
                              << " in rcsc::Polygon2D::scissorWithLine()"
                              << std::endl;

                    std::abort();
                }

                new_points -> push_back(c);
                new_points -> push_back(p1);
            } else {
                // noting to do
            }
        }
    }
}

class XLessEqual {
private:
    double threshold;

public:
    XLessEqual(double threshold)
        : threshold(threshold) {
    }

    bool operator()(const Vector2D & p) const {
        return p.x <= threshold;
    }
};

class XMoreEqual {
private:
    double threshold;

public:
    XMoreEqual(double threshold)
        : threshold(threshold) {
    }

    bool operator()(const Vector2D & p) const {
        return p.x >= threshold;
    }
};

class YLessEqual {
private:
    double threshold;

public:
    YLessEqual(double threshold)
        : threshold(threshold) {
    }

    bool operator()(const Vector2D & p) const {
        return p.y <= threshold;
    }
};

class YMoreEqual {
private:
    double threshold;

public:
    YMoreEqual(double threshold)
        : threshold(threshold) {
    }

    bool operator()(const Vector2D & p) const {
        return p.y >= threshold;
    }
};


/*-------------------------------------------------------------------*/
/*!

*/
Polygon2D
Polygon2D::getScissoredConnectedPolygon(const Rect2D & r) const {
    if (M_vertex.empty()) {
        return Polygon2D();
    }

    std::vector< Vector2D > p = M_vertex;
    std::vector< Vector2D > clipped_p_1;
    std::vector< Vector2D > clipped_p_2;
    std::vector< Vector2D > clipped_p_3;
    std::vector< Vector2D > clipped_p_4;

    scissorWithLine< XLessEqual >(XLessEqual(r.maxX()),
                                  p, &clipped_p_1,
                                  Line2D(Vector2D(r.maxX(), 0.0), 90.0));

    scissorWithLine< YLessEqual >(YLessEqual(r.maxY()),
                                  clipped_p_1, &clipped_p_2,
                                  Line2D(Vector2D(0.0, r.maxY()), 0.0));

    scissorWithLine< XMoreEqual >(XMoreEqual(r.minX()),
                                  clipped_p_2, &clipped_p_3,
                                  Line2D(Vector2D(r.minX(), 0.0), 90.0));

    scissorWithLine< YMoreEqual >(YMoreEqual(r.minY()),
                                  clipped_p_3, &clipped_p_4,
                                  Line2D(Vector2D(0.0, r.minY()), 0.0));

    return Polygon2D(clipped_p_4);
}

}





// -*-c++-*-

/*!
  \file ray_2d.cpp
  \brief 2D ray line class Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa Akiyama

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "parsian_util/geom/ray_2d.h"

namespace rcsc {

/*-------------------------------------------------------------------*/
/*!

 */
Vector2D
Ray2D::intersection(const Line2D & other) const {
    Line2D my_line = this->line();

    Vector2D tmp_sol = my_line.intersection(other);

    if (! tmp_sol.valid()) {
        return Vector2D::INVALIDATED;
    }

    if (! inRightDir(tmp_sol)) {
        return Vector2D::INVALIDATED;
    }

    return tmp_sol;
}

/*-------------------------------------------------------------------*/
/*!

 */
Vector2D
Ray2D::intersection(const Ray2D & other) const {
    Vector2D tmp_sol = this->line().intersection(other.line());

    if (! tmp_sol.valid()) {
        return Vector2D::INVALIDATED;
    }

    if (! this->inRightDir(tmp_sol)
            || ! other.inRightDir(tmp_sol)) {
        return Vector2D::INVALIDATED;
    }

    return tmp_sol;

}

}









// -*-c++-*-

/*!
  \file rect_2d.cpp
  \brief 2D rectangle region Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa Akiyama

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "parsian_util/geom/rect_2d.h"

#include "parsian_util/geom/segment_2d.h"
#include "parsian_util/geom/ray_2d.h"

#include <iostream>

namespace rcsc {

/*-------------------------------------------------------------------*/
/*!

 */
int
Rect2D::intersection(const Line2D & line,
                     Vector2D * sol1,
                     Vector2D * sol2) const {
    int n_sol = 0;
    Vector2D tsol[2];

    const double left_x = left();
    const double right_x = right();
    const double top_y = top();
    const double bottom_y = bottom();

    if (n_sol < 2
            && (tsol[n_sol] = leftEdge().intersection(line)).valid()
            && top_y >= tsol[n_sol].y && tsol[n_sol].y >= bottom_y) {
        ++n_sol;
    }

    if (n_sol < 2
            && (tsol[n_sol] = rightEdge().intersection(line)).valid()
            && top_y >= tsol[n_sol].y && tsol[n_sol].y >= bottom_y) {
        ++n_sol;
    }

    if (n_sol < 2
            && (tsol[n_sol] = topEdge().intersection(line)).valid()
            && left_x <= tsol[n_sol].x && tsol[n_sol].x <= right_x) {
        ++n_sol;
    }

    if (n_sol < 2
            && (tsol[n_sol] = bottomEdge().intersection(line)).valid()
            && left_x <= tsol[n_sol].x && tsol[n_sol].x <= right_x) {
        ++n_sol;
    }

    if (n_sol > 0
            && sol1) {
        *sol1 = tsol[0];
    }

    if (n_sol > 1
            && sol2) {
        *sol2 = tsol[1];
    }

    return n_sol;
}

/*-------------------------------------------------------------------*/
/*!

 */
int
Rect2D::intersection(const Ray2D & ray,
                     Vector2D * sol1,
                     Vector2D * sol2) const {
    Vector2D tsol1, tsol2;
    int n_sol = intersection(ray.line(), &tsol1, &tsol2);

    if (n_sol > 1
            && ! ray.inRightDir(tsol2, 1.0)) {
        --n_sol;
    }

    if (n_sol > 0
            && ! ray.inRightDir(tsol1, 1.0)) {
        tsol1 = tsol2;
        --n_sol;
    }

    if (n_sol > 0
            && sol1) {
        *sol1 = tsol1;
    }

    if (n_sol > 1
            && sol2) {
        *sol2 = tsol2;
    }

    return n_sol;
}

/*-------------------------------------------------------------------*/
/*!

 */
int
Rect2D::intersection(const Segment2D & segment,
                     Vector2D * sol1,
                     Vector2D * sol2) const {
    Vector2D tsol1, tsol2;
    int n_sol = intersection(segment.line(), &tsol1, &tsol2);

    if (n_sol > 1
            && ! segment.contains(tsol2)) {
        --n_sol;
    }

    if (n_sol > 0
            && ! segment.contains(tsol1)) {
        tsol1 = tsol2;
        --n_sol;
    }

    if (n_sol > 0
            && sol1) {
        *sol1 = tsol1;
    }

    if (n_sol > 1
            && sol2) {
        *sol2 = tsol2;
    }

    return n_sol;
}


int
Rect2D::intersection(const Circle2D & circle,
                     Vector2D * sol1, Vector2D * sol2,
                     Vector2D * sol3, Vector2D * sol4) const {
    int ni = 0;
    Vector2D sols[4] = {Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE),
                        Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE),
                        Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE),
                        Vector2D(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE)
                       };

    Segment2D segs[4];
    segs[0] = Segment2D(topLeft(), topRight());
    segs[1] = Segment2D(bottomLeft(), bottomRight());
    segs[2] = Segment2D(bottomLeft(), topLeft());
    segs[3] = Segment2D(bottomRight(), topRight());
    for (int i = 0; i < 4 && ni < 4; i++) {
        ni += circle.intersection(segs[i], &sols[ni], &sols[ni + 1]);
    }
    *sol1 = sols[0];
    *sol2 = sols[1];
    *sol3 = sols[2];
    *sol4 = sols[3];
    return ni;

}

int
Rect2D::rotateAndintersect(const Segment2D & segment, Vector2D center, float angle ,
                           Vector2D * sol1,
                           Vector2D * sol2) const {
    Vector2D a = segment.a() - center;
    Vector2D b = segment.b() - center;
    a.rotate(-angle);
    b.rotate(-angle);
    a += center;
    b += center;
    int res = intersection(Segment2D(a, b), sol1, sol2);
    sol1->rotate(angle);
    sol2->rotate(angle);

    return res;
}


int
Rect2D::rotateAndintersect(const Circle2D & circle, Vector2D center, float angle ,
                           Vector2D * sol1,
                           Vector2D * sol2,
                           Vector2D * sol3,
                           Vector2D * sol4) const {

    Vector2D cirCenter = circle.center() - center;
    cirCenter.rotate(-angle);
    cirCenter += center;
    int res = intersection(Circle2D(cirCenter, circle.radius()) , sol1, sol2, sol3, sol4);

    sol1->rotate(angle);
    sol2->rotate(angle);
    sol3->rotate(angle);
    sol4->rotate(angle);

    return res;
}

}







// -*-c++-*-

/*!
  \file sector_2d.cpp
  \brief 2D sector region Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa Akiyama

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "parsian_util/geom/sector_2d.h"

namespace rcsc {

/*-------------------------------------------------------------------*/
/*!

*/
Sector2D::Sector2D(const Vector2D & c,
                   const double & min_r,
                   const double & max_r,
                   const AngleDeg & start,
                   const AngleDeg & end)
    : M_center(c)
    , M_min_radius(min_r), M_max_radius(max_r)
    , M_angle_left_start(start), M_angle_right_end(end) {
    if (min_r < 0.0) {
        std::cerr << "Sector2D::Sector2D() radius must be positive value."
                  << std::endl;
        M_min_radius = 0.0;
    }
    if (M_min_radius > M_max_radius) {
        std::cerr << "Sector2D::Sector2D(): max radius must be bigger than min radius."
                  << std::endl;
        M_max_radius = M_min_radius;
    }
}

/*-------------------------------------------------------------------*/
/*!

*/
const
Sector2D &
Sector2D::assign(const Vector2D & c,
                 const double & min_r,
                 const double & max_r,
                 const AngleDeg & start,
                 const AngleDeg & end) {
    M_center = c;
    M_min_radius = min_r;
    M_max_radius = max_r;
    M_angle_left_start = start;
    M_angle_right_end = end;

    if (min_r < 0.0) {
        std::cerr << "Sector2D::assign() radius must be positive value."
                  << std::endl;
        M_min_radius = 0.0;
    }
    if (min_r > max_r) {
        std::cerr << "Sector2D::assign() max radius must be bigger than min radius."
                  << std::endl;
        M_max_radius = M_min_radius;
    }

    return *this;
}

/*-------------------------------------------------------------------*/
/*!

*/
double
Sector2D::area() const {
    double circle_area
        = (radiusMax() * radiusMax() * M_PI)
          - (radiusMin() * radiusMin() * M_PI);
    double angle_width
        = (angleRightEnd() - angleLeftStart()).degree();
    if (angle_width < 0.0) {
        angle_width += 360.0;
    }

    circle_area *= (angle_width / 360.0);
    return circle_area;
}

}








// -*-c++-*-

/*!
  \file segment_2d.cpp
  \brief 2D segment line class Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa Akiyama, Hiroki Shimora

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "parsian_util/geom/segment_2d.h"
#include "parsian_util/geom/triangle_2d.h"

#include <algorithm>
#include <iostream>

namespace rcsc {

/*-------------------------------------------------------------------*/
/*!

*/
Vector2D
Segment2D::intersection(const Segment2D & other) const {
    Line2D my_line = this->line();
    Line2D other_line = other.line();

    Vector2D tmp_sol = my_line.intersection(other_line);

    if (! tmp_sol.valid()) {
        return Vector2D::INVALIDATED;
    }

    // check if intersection point is on the line segment
    if (! this->contains(tmp_sol)
            || ! other.contains(tmp_sol)) {
        return Vector2D::INVALIDATED;
    }

    return tmp_sol;

#if 0
    // Following algorithm seems faster ther abover method.
    // In fact, following algorithm slower...

    Vector2D ab = b() - a();
    Vector2D dc = other.a() - other.b();
    Vector2D ad = other.b() - a();

    double det = dc.outerProduct(ab);

    if (std::fabs(det) < 0.001) {
        // area size is 0.
        // segments has same slope.
        std::cerr << "Segment2D::intersection()"
                  << " ***ERROR*** parallel segments"
                  << std::endl;
        return Vector2D::INVALIDATED;
    }

    double s = (dc.x * ad.y - dc.y * ad.x) / det;
    double t = (ab.x * ad.y - ab.y * ad.x) / det;

    if (s < 0.0 || 1.0 < s || t < 0.0 || 1.0 < t) {
        return Vector2D::INVALIDATED;
    }

    return Vector2D(a().x + ab.x * s, a().y + ab.y * s);
#endif
}

/*-------------------------------------------------------------------*/
/*!

*/
Vector2D
Segment2D::intersection(const Line2D & other) const {
    Line2D my_line = this->line();

    Vector2D tmp_sol = my_line.intersection(other);

    if (! tmp_sol.valid()) {
        return Vector2D::INVALIDATED;
    }

    // check if intersection point is on the line segment
    if (! this->contains(tmp_sol)) {
        return Vector2D::INVALIDATED;
    }

    return tmp_sol;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
Segment2D::existIntersectionExceptEndpoint(const Segment2D & other) const {
    return    Triangle2D(*this, other.a()).signedArea2()
              * Triangle2D(*this, other.b()).signedArea2() < 0.0
              &&   Triangle2D(other, this -> a()).signedArea2()
              * Triangle2D(other, this -> b()).signedArea2() < 0.0;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
Segment2D::existIntersection(const Segment2D & other) const {
    double a0 = Triangle2D(*this, other.a()).signedArea2();
    double a1 = Triangle2D(*this, other.b()).signedArea2();
    double b0 = Triangle2D(other, this -> a()).signedArea2();
    double b1 = Triangle2D(other, this -> b()).signedArea2();

    if (a0 * a1 < 0.0 && b0 * b1 < 0.0) {
        return true;
    }

    if (this -> a() == this -> b()) {
        if (other.a() == other.b()) {
            return this -> a() == other.a();
        }

        return b0 == 0.0 && other.checkIntersectsOnLine(this -> a());
    } else if (other.a() == other.b()) {
        return a0 == 0.0 && this -> checkIntersectsOnLine(other.a());
    }


    if ((a0 == 0.0 && this -> checkIntersectsOnLine(other.a()))
            || (a1 == 0.0 && this -> checkIntersectsOnLine(other.b()))
            || (b0 == 0.0 && other.checkIntersectsOnLine(this -> a()))
            || (b1 == 0.0 && other.checkIntersectsOnLine(this -> b()))) {
        return true;
    }

    return false;
}

/*-------------------------------------------------------------------*/
/*!

*/
// private
bool
Segment2D::checkIntersectsOnLine(const Vector2D & p) const {
    if (a().x == b().x) {
        return ((a().y <= p.y && p.y <= b().y)
                || (b().y <= p.y && p.y <= a().y));
    } else {
        return ((a().x <= p.x && p.x <= b().x)
                || (b().x <= p.x && p.x <= a().x));
    }
}

/*-------------------------------------------------------------------*/
/*!

*/
Vector2D
Segment2D::nearestPoint(const Vector2D & p) const {
    const Vector2D vec = b() - a();

    const double len_square = vec.r2();

    if (len_square == 0.0) {
        return a();
    }


    double inner_product = vec.innerProduct((p - a()));

    //
    // A: p1 - p0
    // B: p - p0
    //
    // check if 0 <= |B|cos(theta) <= |A|
    //       -> 0 <= |A||B|cos(theta) <= |A|^2
    //       -> 0 <= A.B <= |A|^2  // A.B = |A||B|cos(theta)
    //
    if (inner_product <= 0.0) {
        return a();
    } else if (inner_product >= len_square) {
        return b();
    }

    return a() + vec * inner_product / len_square;
}

/*-------------------------------------------------------------------*/
/*!

*/
double
Segment2D::dist(const Vector2D & p) const {
    double len = this -> length();

    if (len == 0.0) {
        return (p - a()).r();
    }

    const Vector2D vec = b() - a();
    const double prod = vec.innerProduct(p - a());

    //
    // A: p1 - p0
    // A: p - p0
    //
    // check if 0 <= |B|cos(theta) <= |A|
    //       -> 0 <= |A||b|cos(theta) <= |A|^2
    //       -> 0 <= A.B <= |A|^2  // A.B = |A||B|cos(theta)
    //
    if (0.0 <= prod && prod <= len * len) {
        // return perpendicular distance
        return std::fabs(Triangle2D(*this, p).signedArea2() / len);
    }

    return std::min((p - a()).r(),
                    (p - b()).r());
}

/*-------------------------------------------------------------------*/
/*!

*/
double
Segment2D::dist(const Segment2D &  seg) const {
    if (this -> existIntersection(seg)) {
        return 0.0;
    }

    return std::min(std::min(this -> dist(seg.a()),
                             this -> dist(seg.b())),
                    std::min(seg.dist(a()),
                             seg.dist(b())));
}

/*-------------------------------------------------------------------*/
/*!

*/
double
Segment2D::farthestDist(const Vector2D & p) const {
    return std::max((a() - p).r(), (b() - p).r());
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
Segment2D::onSegment(const Vector2D & p) const {
    return Triangle2D(*this, p).signedArea2() == 0.0
           && this -> checkIntersectsOnLine(p);
}


Vector2D
Segment2D::projection(const Vector2D & p) const {
    Vector2D dir = terminal() - origin();
    double len = dir.r();

    if (len < EPSILON) {
        return origin();
    }

    dir /= len; // normalize

    double d = dir.innerProduct(p - origin());
    if (-EPSILON < d && d < len + EPSILON) {
        dir *= d;
        return Vector2D(origin()) += dir;
    }

    return Vector2D::INVALIDATED;

#if 0
    Line2D my_line = this->line();
    Vector2D sol = my_line.projection(p);

    if (! sol.isValid()
            || ! this->contains(sol)) {
        return Vector2D::INVALIDATED;
    }

    return sol;
#endif
}


bool
Segment2D::onSegmentWeakly(const Vector2D & p) const {
    Vector2D proj = projection(p);

    return (proj.isValid()
            && p.equalsWeakly(proj));

#if 0
    Vector2D o = origin();
    Vector2D t = terminal();

    const double buf = (allow_on_terminal
                        ? EPSILON
                        : 0.0);

    if (std::fabs((t - o).outerProduct(p - o)) < EPSILON) {
        if (std::fabs(o.x - t.x) < EPSILON) {
            return ((o.y - buf < p.y && p.y < t.y + buf)
                    || (t.y - buf < p.y && p.y < o.y + buf));
        } else {
            return ((o.x - buf < p.x && p.x < t.x + buf)
                    || (t.x - buf < p.x && p.x < o.x + buf));
        }
    }

    return false;
#endif
}

}





// -*-c++-*-

/*!
  \file triangle_2d.cpp
  \brief 2D triangle class Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa Akiyama

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "parsian_util/geom/triangle_2d.h"

#include "parsian_util/geom/line_2d.h"

#include <cmath>

namespace rcsc {

/*-------------------------------------------------------------------*/
/*!

 */
bool
Triangle2D::contains(const Vector2D & point) const {
    Vector2D rel1(M_a - point);
    Vector2D rel2(M_b - point);
    Vector2D rel3(M_c - point);

    double outer1 = rel1.outerProduct(rel2);
    double outer2 = rel2.outerProduct(rel3);
    double outer3 = rel3.outerProduct(rel1);

    if ((outer1 >= 0.0 && outer2 >= 0.0 && outer3 >= 0.0)
            || (outer1 <= 0.0 && outer2 <= 0.0 && outer3 <= 0.0)) {
        return true;
    }

    return false;
}


/*-------------------------------------------------------------------*/
/*!

 */
bool
Triangle2D::contains(const Vector2D & a,
                     const Vector2D & b,
                     const Vector2D & c,
                     const Vector2D & point) {
    Vector2D rel1(a - point);
    Vector2D rel2(b - point);
    Vector2D rel3(c - point);

    double outer1 = rel1.outerProduct(rel2);
    double outer2 = rel2.outerProduct(rel3);
    double outer3 = rel3.outerProduct(rel1);

    if ((outer1 >= 0.0 && outer2 >= 0.0 && outer3 >= 0.0)
            || (outer1 <= 0.0 && outer2 <= 0.0 && outer3 <= 0.0)) {
        return true;
    }

    return false;
}
/*-------------------------------------------------------------------*/
/*!

 */
Vector2D
Triangle2D::incenter(const Vector2D & a,
                     const Vector2D & b,
                     const Vector2D & c) {
    Vector2D ab = b - a;
    Vector2D ac = c - a;
    Line2D bisect_a(a,
                    AngleDeg::bisect(ab.th(), ac.th()));

    Vector2D ba = a - b;
    Vector2D bc = c - b;
    Line2D bisect_b(b,
                    AngleDeg::bisect(ba.th(), bc.th()));

    return bisect_a.intersection(bisect_b);
}

/*-------------------------------------------------------------------*/
/*!

 */
Vector2D
Triangle2D::circumcenter(const Vector2D & a,
                         const Vector2D & b,
                         const Vector2D & c) {
    Line2D perpendicular_ab
        = Line2D::perpendicular_bisector(a, b);

    Line2D perpendicular_bc
        = Line2D::perpendicular_bisector(b, c);

    return perpendicular_ab.intersection(perpendicular_bc);

#if 0
    // Following algorithm seems faster than above method.
    // However, result is as:
    //   above method     10000000times 730 [ms]
    //   following method 10000000times 934 [ms]
    // So, I choose above method.

    ////////////////////////////////////////////////////////////////
    // Q : curcumcenter
    // M : center of AB
    // N : center of AC
    // s, t : parameter
    // <,> : inner product operator
    // S : area of triangle
    // a = |BC|, b = |CA|, c = |AB|

    // AQ = s*AB + t*AC

    // <MQ, AB> = <AQ - AM, AB>
    //          = <s*AB + t*AC - AB/2, AB >
    //          = <(s-1/2)*AB^2 + tAB, AC>
    //          = (s-1/2)*c^2 + t*b*c*cosA
    //          = 0
    // <NQ, AC> = s*b*c*cosA + (t-1/2)*b^2 = 0

    // c^2 * s + (b*c*cosA)*t = c^2 / 2
    // (b*c*cosA)*s + b^2 * t = b^2 / 2

    // s = b^2 * (c^2 + a^2 - b^2) / (16S^2)
    // t = c^2 * (a^2 + b^2 - c^2) / (16S^2)

    // AQ = {b^2 * (c^2 + a^2 - b^2) * AB + c^2 * (a^2 + b^2 - c^2)) * AC} /(16S^2)

    Vector2D ab = b - a;
    Vector2D ac = c - a;

    double tmp = ab.outerProduct(ac);
    if (std::fabs(tmp) < 0.001) {
        // The area of parallelogram is 0.
        std::cerr << "Triangle2D::getCircumCenter()"
                  << " ***ERROR*** at least, two vertex points have same coordiante"
                  << std::endl;
        return Vector2D(Vector2D::INVALID);
    }

    double inv = 0.5 / tmp;
    double ab_len2 = ab.r2();
    double ac_len2 = ac.r2();
    double xcc = inv * (ab_len2 * ac.y - ac_len2 * ab.y);
    double ycc = inv * (ab.x * ac_len2 - ac.x * ab_len2);
    // circle radius = xcc*xcc + ycc*ycc
    return Vector2D(a.x + xcc, a.y + ycc);
#endif
}

/*-------------------------------------------------------------------*/
/*!

 */
Vector2D
Triangle2D::orthocenter(const Vector2D & a,
                        const Vector2D & b,
                        const Vector2D & c) {
    Line2D perpend_a = Line2D(b, c).perpendicular(a);
    Line2D perpend_b = Line2D(c, a).perpendicular(b);

    return perpend_a.intersection(perpend_b);
}

}






// -*-c++-*-

/*!
  \file vector_2d.cpp
  \brief 2D vector class Source File.
*/

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "parsian_util/geom/vector_2d.h"

#include <limits>

namespace rcsc {

//const double Vector2D::ERROR_VALUE = std::numeric_limits< double >::max();
const double Vector2D::ERROR_VALUE = 5000.0;
const Vector2D Vector2D::INVALIDATED(Vector2D::ERROR_VALUE, Vector2D::ERROR_VALUE);

}

int sign(double d1) {
    return (d1 > 0) ? 1 : -1;
}

double max(double d1, double d2) {
    return (d1 > d2) ? d1 : d2;
}

double min(double d1, double d2) {
    return (d1 < d2) ? d1 : d2;
}



namespace rcsc {
Vector2D intersect_ellipse_dir(Vector2D dir, Vector2D center, double a, double b, double e) {
    Vector2D d = dir;
    d.x = d.x * b / a;
    Segment2D s = Segment2D(center + Vector2D(0.0, e), center + dir * 2.0 * b + Vector2D(0.0, e));
    Vector2D sol1, sol2;
    int n = Circle2D(center, b).intersection(s, &sol1, &sol2);
    if (sol1.valid()) {
        sol1.x = (sol1.x - center.x) * a / b + center.x;
    }
    return sol1;
}


bool intersect_ellipse_line(Vector2D point1, Vector2D point2, Vector2D center, double _a, double _b, Vector2D * sol1, Vector2D * sol2) {
    double h = center.x;
    double k = center.y;
    double a = _a;
    double b = _b;
    double x1 = point1.x;
    double y1 = point1.y;
    double x2 = point2.x;
    double y2 = point2.y;
    double xi1, xi2, yi1, yi2;

    float aa, bb, cc, m;
    //
    if (x1 != x2) {
        m = (y2 - y1) / (x2 - x1);
        float c = y1 - m * x1;
        //
        aa = b * b + a * a * m * m;
        bb = 2 * a * a * c * m - 2 * a * a * k * m - 2 * h * b * b;
        cc = b * b * h * h + a * a * c * c - 2 * a * a * k * c + a * a * k * k - a * a * b * b;
    } else {
        //
        // vertical line case
        //
        aa = a * a;
        bb = -2.0 * k * a * a;
        cc = -a * a * b * b + b * b * (x1 - h) * (x1 - h);
    }

    float d = bb * bb - 4 * aa * cc;
    //
    // intersection points : (xi1,yi1) and (xi2,yi2)
    //
    if (d > 0.0) {
        if (x1 != x2) {
            xi1 = (-bb + sqrt(d)) / (2 * aa);
            xi2 = (-bb - sqrt(d)) / (2 * aa);
            yi1 = y1 + m * (xi1 - x1);
            yi2 = y1 + m * (xi2 - x1);
        } else {
            yi1 = (-bb + sqrt(d)) / (2 * aa);
            yi2 = (-bb - sqrt(d)) / (2 * aa);
            xi1 = x1;
            xi2 = x1;
        }
    } else {
        return false; // no intersections
    }
    sol1->x = xi1;
    sol1->y = yi1;
    sol2->x = xi2;
    sol2->y = yi2;
    return true;
}

}
