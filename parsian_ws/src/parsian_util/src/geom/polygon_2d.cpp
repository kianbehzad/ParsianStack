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
    : M_vertex()
{
}

/*-------------------------------------------------------------------*/
/*!

*/
Polygon2D::Polygon2D( const std::vector< Vector2D > & v )
    : M_vertex( v )
{
}

/*-------------------------------------------------------------------*/
/*!

*/
const Polygon2D &
Polygon2D::assign()
{
    M_vertex.clear();
    return *this;
}

/*-------------------------------------------------------------------*/
/*!

*/
const Polygon2D &
Polygon2D::assign( const std::vector< Vector2D > & v )
{
    M_vertex = v;
    return *this;
}

/*-------------------------------------------------------------------*/
/*!

*/
void
Polygon2D::addVertex( const Vector2D & p )
{
    M_vertex.push_back( p );
}

/*-------------------------------------------------------------------*/
/*!

*/
const std::vector< Vector2D > &
Polygon2D::vertex() const
{
    return M_vertex;
}

/*-------------------------------------------------------------------*/
/*!

*/
std::vector< Vector2D > &
Polygon2D::vertex()
{
    return M_vertex;
}

/*-------------------------------------------------------------------*/
/*!

*/
Rect2D
Polygon2D::getBoundingBox() const
{
    if ( M_vertex.empty() )
    {
        return Rect2D();
    }

    double x_min = +DBL_MAX;
    double x_max = -DBL_MAX;
    double y_min = +DBL_MAX;
    double y_max = -DBL_MAX;

    for ( size_t i = 0; i < M_vertex.size(); ++i )
    {
        const Vector2D & p = M_vertex[i];

        if ( p.x > x_max )
        {
            x_max = p.x;
        }
        if ( p.x < x_min )
        {
            x_min = p.x;
        }
        if ( p.y > y_max )
        {
            y_max = p.y;
        }
        if ( p.y < y_min )
        {
            y_min = p.y;
        }
    }

    return( Rect2D( x_min, y_max, (x_max - x_min), (y_max - y_min) ) );
}

/*-------------------------------------------------------------------*/
/*!

*/
Vector2D
Polygon2D::xyCenter() const
{
    return this -> getBoundingBox().center();
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
Polygon2D::contains( const Vector2D & p,
                     bool allow_on_segment ) const
{
    if ( M_vertex.empty() )
    {
        return false;
    }
    else if ( M_vertex.size() == 1 )
    {
        return allow_on_segment && (M_vertex[0] == p);
    }


    Rect2D r = this -> getBoundingBox();

    // if ( ! r.contains( p ) )
    // {
    //     return false;
    // }


    //
    // make virtual half line
    //
    Segment2D line( p, Vector2D( p.x + ((r.maxX() - r.minX()
                                         + r.maxY() - r.minY())
                                        + (M_vertex[0] - p).r()) * 3.0,
                                 p.y ) );

    //
    // check intersection with all segments
    //
    bool inside = false;
    double min_line_x = r.maxX() + 1.0;

    for ( size_t i = 0; i < M_vertex.size(); ++i )
    {
        size_t p1_index = i + 1;

        if ( p1_index >= M_vertex.size() )
        {
            p1_index = 0;
        }

        const Vector2D p0 = M_vertex[i];
        const Vector2D p1 = M_vertex[p1_index];

        if ( ! allow_on_segment )
        {
            if ( Segment2D( p0, p1 ).onSegment( p ) )
            {
                return false;
            }
        }

        if ( allow_on_segment && p == p0 )
        {
            return true;
        }

        if ( line.existIntersection( Segment2D( p0, p1 ) ) )
        {
            if ( p0.y == p.y
              || p1.y == p.y )
            {
                if ( p0.y == p.y )
                {
                    if ( p0.x < min_line_x )
                    {
                        min_line_x = p0.x;
                    }
                }

                if ( p1.y == p.y )
                {
                    if ( p1.x < min_line_x )
                    {
                        min_line_x = p1.x;
                    }
                }


                if ( p0.y == p1.y )
                {
                    continue;
                }
                else if ( p0.y < p.y
                       || p1.y < p.y )
                {
                    continue;
                }
                else // bottom point on the line
                {
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
Polygon2D::dist( const Vector2D & p,
                 bool check_as_plane ) const
{
    if ( this -> vertex().size() == 1 )
    {
        return (this -> vertex()[0] - p).r();
    }

    if ( check_as_plane && this -> contains( p ) )
    {
        return 0.0;
    }

    double min_dist = +DBL_MAX;

    for ( size_t i = 0; i + 1 < this -> vertex().size(); ++i )
    {
        Segment2D seg( this -> vertex()[i],
                       this -> vertex()[i + 1] );

        double d = seg.dist( p );

        if ( d < min_dist )
        {
            min_dist = d;
        }
    }

    if ( this -> vertex().size() >= 3 )
    {
        Segment2D seg( *(this -> vertex().rbegin()),
                       *(this -> vertex().begin()) );

        double d = seg.dist( p );

        if ( d < min_dist )
        {
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
Polygon2D::area() const
{
    return std::fabs( this -> signedArea2() / 2.0 );
}

/*-------------------------------------------------------------------*/
/*!

*/
double
Polygon2D::signedArea2() const
{
    if ( M_vertex.size() < 3 )
    {
        return 0.0;
    }

    double sum = 0.0;
    for ( size_t i = 0; i < M_vertex.size(); ++i )
    {
        size_t n = i + 1;
        if ( n == M_vertex.size() )
        {
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
Polygon2D::isCounterclockwise() const
{
    return this -> signedArea2() > 0.0;
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
Polygon2D::isClockwise() const
{
    return this -> signedArea2() < 0.0;
}


/*-------------------------------------------------------------------*/
/*!

*/
template< class Predicate >
void
scissorWithLine( const Predicate & in_region,
                 const std::vector< Vector2D > & points,
                 std::vector< Vector2D > * new_points,
                 const Line2D & line )
{
    new_points -> clear();

    std::vector< bool > in_rectangle( points.size() );

    for ( size_t i = 0; i < points.size(); ++i )
    {
        in_rectangle[i] = in_region( points[i] );
    }

    for ( size_t i = 0; i < points.size(); ++i )
    {
        size_t index_0 = i;
        size_t index_1 = i + 1;

        if ( index_1 >= points.size() )
        {
            index_1 = 0;
        }

        const Vector2D & p0 = points[index_0];
        const Vector2D & p1 = points[index_1];

        if ( in_rectangle[index_0] )
        {
            if ( in_rectangle[index_1] )
            {
                new_points -> push_back( p1 );
            }
            else
            {
                Vector2D c = line.intersection( Line2D( p0, p1 ) );

                if ( ! c.valid() )
                {
                    std::cerr << "internal error:"
                              << " in rcsc::Polygon2D::scissorWithLine()"
                              << std::endl;

                    std::abort();
                }

                new_points -> push_back( c );
            }
        }
        else
        {
            if ( in_rectangle[index_1] )
            {
                Vector2D c = line.intersection( Line2D( p0, p1 ) );

                if ( ! c.valid() )
                {
                    std::cerr << "internal error:"
                              << " in rcsc::Polygon2D::scissorWithLine()"
                              << std::endl;

                    std::abort();
                }

                new_points -> push_back( c );
                new_points -> push_back( p1 );
            }
            else
            {
                // noting to do
            }
        }
    }
}

class XLessEqual
{
private:
    double threshold;

public:
    XLessEqual( double threshold )
        : threshold( threshold )
      {
      }

    bool operator()( const Vector2D & p ) const
      {
          return p.x <= threshold;
      }
};

class XMoreEqual
{
private:
    double threshold;

public:
    XMoreEqual( double threshold )
        : threshold( threshold )
      {
      }

    bool operator()( const Vector2D & p ) const
      {
          return p.x >= threshold;
      }
};

class YLessEqual
{
private:
    double threshold;

public:
    YLessEqual( double threshold )
        : threshold( threshold )
      {
      }

    bool operator()( const Vector2D & p ) const
      {
          return p.y <= threshold;
      }
};

class YMoreEqual
{
private:
    double threshold;

public:
    YMoreEqual( double threshold )
        : threshold( threshold )
      {
      }

    bool operator()( const Vector2D & p ) const
      {
          return p.y >= threshold;
      }
};


/*-------------------------------------------------------------------*/
/*!

*/
Polygon2D
Polygon2D::getScissoredConnectedPolygon( const Rect2D & r ) const
{
    if ( M_vertex.empty() )
    {
        return Polygon2D();
    }

    std::vector< Vector2D > p = M_vertex;
    std::vector< Vector2D > clipped_p_1;
    std::vector< Vector2D > clipped_p_2;
    std::vector< Vector2D > clipped_p_3;
    std::vector< Vector2D > clipped_p_4;

    scissorWithLine< XLessEqual >( XLessEqual( r.maxX() ),
                                   p, &clipped_p_1,
                                   Line2D( Vector2D( r.maxX(), 0.0 ), 90.0 ) );

    scissorWithLine< YLessEqual >( YLessEqual( r.maxY() ),
                                   clipped_p_1, &clipped_p_2,
                                   Line2D( Vector2D( 0.0, r.maxY() ), 0.0 ) );

    scissorWithLine< XMoreEqual >( XMoreEqual( r.minX() ),
                                   clipped_p_2, &clipped_p_3,
                                   Line2D( Vector2D( r.minX(), 0.0 ), 90.0 ) );

    scissorWithLine< YMoreEqual >( YMoreEqual( r.minY() ),
                                   clipped_p_3, &clipped_p_4,
                                   Line2D( Vector2D( 0.0, r.minY() ), 0.0 ) );

    return Polygon2D( clipped_p_4 );
}

}
