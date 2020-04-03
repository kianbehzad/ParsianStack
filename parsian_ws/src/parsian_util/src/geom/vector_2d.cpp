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

