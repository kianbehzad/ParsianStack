// -*-c++-*-

/*!
  \file rect_2d.h
  \brief 2D rectangle region Header File.
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

#ifndef RCSC_GEOM_RECT2D_H
#define RCSC_GEOM_RECT2D_H

#include <parsian_util/geom/size_2d.h>
#include <parsian_util/geom/line_2d.h>
#include <parsian_util/geom/vector_2d.h>
#include <parsian_util/geom/circle_2d.h>

namespace rcsc {

class Ray2D;
class Segment2D;

/*!
  \class Rect2D
  \brief 2D rectangle regin class.

  The model and naming rules are depend on soccer simulator environment
          -34.0
            |
            |
-52.5 ------+------- 52.5
            |
            |
          34.0
*/
class Rect2D {
private:
    //! top left point
    Vector2D M_top_left;

    //! XY range
    Size2D M_size;

public:
    /*!
      \brief default constructor creates a zero area rectanble at (0,0)
     */
    Rect2D()
        : M_top_left(0.0, 0.0)
        , M_size(0.0, 0.0) {
    }

    /*!
      \brief constructor
      \param left_x left x
      \param top_y top y
      \param length length (x-range)
      \param width width (y-range)
     */
    Rect2D(const double & left_x,
           const double & top_y,
           const double & length,
           const double & width)
        : M_top_left(left_x, top_y)
        , M_size(length, width) {
    }

    /*!
      \brief constructor with variables
      \param top_left top left point
      \param length X range
      \param width Y range
     */
    Rect2D(const Vector2D & top_left,
           const double & length,
           const double & width)
        : M_top_left(top_left)
        , M_size(length, width) {
    }

    /*!
      \brief constructor with variables
      \param top_left top left point
      \param size XY range
     */
    Rect2D(const Vector2D & top_left,
           const Size2D & size)
        : M_top_left(top_left)
        , M_size(size) {
    }

    /*!
      \brief constructor with 2 points.
      \param top_left top left vertex
      \param bottom_right bottom right vertex

      Even if argument point has incorrect values,
      the assigned values are normalized automatically.
    */
    Rect2D(const Vector2D & top_left,
           const Vector2D & bottom_right)
        : M_top_left(top_left)
        , M_size(bottom_right.x - top_left.x,
                 top_left.y - bottom_right.y) {
        if (bottom_right.x - top_left.x < 0.0) {
            M_top_left.x = bottom_right.x;
        }
        if (bottom_right.y - top_left.y > 0.0) {
            M_top_left.y = bottom_right.y;
        }
    }

    /*!
      \brief create rectangle with center point and size.
      \param center center point of new rectangle.
      \param length length(x-range) of new rectangle.
      \param width width(y-range) of new rectangle.
     */
    static
    Rect2D from_center(const Vector2D & center,
                       const double & length,
                       const double & width) {
        return Rect2D(center.x - length * 0.5,
                      center.y + width * 0.5,
                      length,
                      width);
    }

    /*!
      \brief create rectangle with center point and size.
      \param center_x x value of center point of new rectangle.
      \param center_y y value of center point of new rectangle.
      \param length length(x-range) of new rectangle.
      \param width width(y-range) of new rectangle.
     */
    static
    Rect2D from_center(const double & center_x,
                       const double & center_y,
                       const double & length,
                       const double & width) {
        return Rect2D(center_x - length * 0.5,
                      center_y + width * 0.5,
                      length,
                      width);
    }

    /*!
      \brief create rectangle with 2 corner points. just call one of constructor.
      \param top_left top left vertex
      \param bottom_right bottom right vertex
    */
    static
    Rect2D from_corners(const Vector2D & top_left,
                        const Vector2D & bottom_right) {
        return Rect2D(top_left, bottom_right);
    }

    /*!
      \brief assign new values
      \param left_x left x
      \param top_y top y
      \param length X range
      \param width Y range
     */
    const
    Rect2D & assign(const double & left_x,
                    const double & top_y,
                    const double & length,
                    const double & width) {
        M_top_left.assign(left_x, top_y);
        M_size.assign(length, width);
        return *this;
    }

    /*!
      \brief assign new values
      \param top_left top left point
      \param length X range
      \param width Y range
      \return const referenct to itself
     */
    const
    Rect2D & assign(const Vector2D & top_left,
                    const double & length,
                    const double & width) {
        M_top_left = top_left;
        M_size.assign(length, width);
        return *this;
    }

    /*!
      \brief assign new values
      \param top_left top left
      \param size XY range
      \return const referenct to itself
     */
    const
    Rect2D & assign(const Vector2D & top_left,
                    const Size2D & size) {
        M_top_left = top_left;
        M_size = size;
        return *this;
    }

    /*!
      \brief set a new top left corner point
      \param x new x coordinate
      \param y new y coordinate
      \return const referenct to itself
     */
    const
    Rect2D & setTopLeft(const double & x,
                        const double & y) {
        M_top_left.assign(x, y);
        return *this;
    }

    /*!
      \brief set a new top left corner point
      \param point new coordinate
      \return const referenct to itself
     */
    const
    Rect2D & setTopLeft(const Vector2D & point) {
        M_top_left = point;
        return *this;
    }

    /*!
      \brief set a new center point. only top left corner is moved.
      size is not changed.
      \param point new center coordinate
      \return const referenct to itself
     */
    const
    Rect2D & setCenter(const Vector2D & point) {
        M_top_left.assign(point.x - M_size.length() * 0.5,
                          point.y - M_size.width() * 0.5);
        return *this;
    }

    /*!
      \brief set a new x-range
      \param length new range
      \return const referenct to itself
     */
    const
    Rect2D & setLength(const double & length) {
        M_size.setLength(length);
        return *this;
    }

    /*!
      \brief set a new y-range
      \param width new range
      \return const referenct to itself
     */
    const
    Rect2D & setWidth(const double & width) {
        M_size.setWidth(width);
        return *this;
    }

    /*!
      \brief set a new size
      \param length new range
      \param width new range
      \return const referenct to itself
     */
    const
    Rect2D & setSize(const double & length,
                     const double & width) {
        M_size.assign(length, width);
        return *this;
    }

    /*!
      \brief set a new size
      \param size new range
      \return const referenct to itself
     */
    const
    Rect2D & setSize(const Size2D & size) {
        M_size = size;
        return *this;
    }

    /*!
      \brief check if point is within this region.
      \param point considered point
      \return true or false
     */
    bool contains(const Vector2D & point) const {
        return (left() <= point.x
                && point.x <= right()
                && top() >= point.y
                && point.y >= bottom());
    }

    /*!
      \brief get the left x coordinate of this rectangle.
      \return x coordinate value
    */
    const
    double & left() const {
        return M_top_left.x;
    }

    /*!
      \brief get the right x coordinate of this rectangle.
      \return x coordinate value
    */
    double right() const {
        return left() + size().length();
    }

    /*!
      \brief get the top y coordinate of this rectangle.
      \return y coordinate value
    */
    const
    double & top() const {
        return M_top_left.y;
    }

    /*!
      \brief get the bottom y coordinate of this rectangle.
      \return y coordinate value
    */
    double bottom() const {
        return top() - size().width();
    }

    /*!
      \brief get minimum value of x coordinate of this rectangle
      \return x coordinate value (equivalent to left())
    */
    double minX() const {
        return left();
    }

    /*!
      \brief get maximum value of x coordinate of this rectangle
      \return x coordinate value (equivalent to right())
    */
    double maxX() const {
        return right();
    }

    /*!
      \brief get minimum value of y coordinate of this rectangle
      \return y coordinate value (equivalent to top())
    */
    double minY() const {
        return bottom();
    }

    /*!
      \brief get maximum value of y coordinate of this rectangle
      \return y coordinate value (equivalent to bottom())
    */
    double maxY() const {
        return top();
    }

    /*!
      \brief get the XY range of this rectangle
      \return size object
    */
    const
    Size2D & size() const {
        return M_size;
    }

    /*!
      \brief get center point
      \return coordinate value by vector object
     */
    Vector2D center() const {
        return Vector2D((left() + right()) * 0.5,
                        (top() + bottom()) * 0.5);
    }

    /*!
      \brief get the top-left corner point
      \return coordiante value by vector object
    */
    const
    Vector2D & topLeft() const {
        return M_top_left;
    }

    /*!
      \brief get the top-right corner point
      \return coordiante value by vector object
    */
    Vector2D topRight() const {
        return Vector2D(right(), top());
    }

    /*!
      \brief get the bottom-left corner point
      \return coordiante value by vector object
    */
    Vector2D bottomLeft() const {
        return Vector2D(left(), bottom());
    }

    /*!
      \brief get the bottom-right corner point
      \return coordiante value by vector object
    */
    Vector2D bottomRight() const {
        return Vector2D(right(), bottom());
    }

    /*!
      \brief get the left edge line
      \return line object
    */
    Line2D leftEdge() const {
        return Line2D(topLeft(), bottomLeft());
    }

    /*!
      \brief get the right edge line
      \return line object
    */
    Line2D rightEdge() const {
        return Line2D(topRight(), bottomRight());
    }

    /*!
      \brief get the top edge line
      \return line object
    */
    Line2D topEdge() const {
        return Line2D(topLeft(), topRight());
    }

    /*!
      \brief get the bottom edge line
      \return line object
    */
    Line2D bottomEdge() const {
        return Line2D(bottomLeft(), bottomRight());
    }

    /*!
      \brief calculate intersection point with line.
      \param line considerd line.
      \param sol1 pointer to the 1st solution variable
      \param sol2 pointer to the 2nd solution variable
      \return number of intersection
    */
    int intersection(const Line2D & line,
                     Vector2D * sol1,
                     Vector2D * sol2) const;

    /*!
      \brief calculate intersection point with ray.
      \param ray considerd ray line.
      \param sol1 pointer to the 1st solution variable
      \param sol2 pointer to the 2nd solution variable
      \return number of intersection
    */
    int intersection(const Ray2D & ray,
                     Vector2D * sol1,
                     Vector2D * sol2) const;

    /*!
      \brief calculate intersection point with line segment.
      \param segment considerd line segment.
      \param sol1 pointer to the 1st solution variable
      \param sol2 pointer to the 2nd solution variable
      \return number of intersection
    */
    int intersection(const Segment2D & segment,
                     Vector2D * sol1,
                     Vector2D * sol2) const;

    int intersection(const Circle2D & circle,
                     Vector2D * sol1,
                     Vector2D * sol2,
                     Vector2D * sol3,
                     Vector2D * sol4) const;

    int rotateAndintersect(const Segment2D & segment, Vector2D center, float angle ,
                           Vector2D * sol1,
                           Vector2D * sol2) const;
    int rotateAndintersect(const Circle2D & circle, Vector2D center, float angle ,
                           Vector2D * sol1,
                           Vector2D * sol2,
                           Vector2D * sol3,
                           Vector2D * sol4) const;
};

}

#endif
