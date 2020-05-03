//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
 \file    field.cpp
 \brief   Field markings management
 \author  Joydeep Biswas, (C) 2013
 */
//========================================================================

#include "parsian_gui/interface/application/widgets/graphical_client/field.h"

FieldLine::FieldLine(QString name_, double p1_x_, double p1_y_, double p2_x_,
                     double p2_y_, double thickness_):
    name(name_), p1_x(p1_x_), p1_y(p1_y_), p2_x(p2_x_), p2_y(p2_y_),
    thickness(thickness_) {
}

FieldLine::FieldLine(const FieldLine& other) :
    name(other.name),
    p1_x(other.p1_x),
    p1_y(other.p1_y),
    p2_x(other.p2_x),
    p2_y(other.p2_y),
    thickness(other.thickness) {}

FieldLine::FieldLine(const QString& marking_name) :
    name(marking_name),
    p1_x(),
    p1_y(),
    p2_x(),
    p2_y(),
    thickness(10) {}


FieldLine::~FieldLine() {

}

FieldCircularArc::FieldCircularArc(
        QString name_,
        double center_x_,
        double center_y_,
        double radius_,
        double a1_,
        double a2_,
        double thickness_):
    name(name_), center_x(center_x_), center_y(center_y_), radius(radius_),
    a1(a1_), a2(a2_), thickness(thickness_) {}

FieldCircularArc::FieldCircularArc(const FieldCircularArc& other) :
    name(other.name),
    center_x(other.center_x),
    center_y(other.center_y),
    radius(other.radius),
    a1(other.a1),
    a2(other.a2),
    thickness(other.thickness) {}

FieldCircularArc::FieldCircularArc(const QString& marking_name) :
    name(marking_name),
    center_x(),
    center_y(),
    radius(),
    a1(),
    a2(),
    thickness(10) {}

FieldCircularArc::~FieldCircularArc() {

}

FieldTriangle::FieldTriangle(QString name_, double p1_x_, double p1_y_, double p2_x_, double p2_y_, double p3_x_, double p3_y_) :
    name(name_),
    p1_x(p1_x_),
    p1_y(p1_y_),
    p2_x(p2_x_),
    p2_y(p2_y_),
    p3_x(p3_x_),
    p3_y(p3_y_) {}

FieldTriangle::FieldTriangle(const FieldTriangle &other) :
    name(other.name),
    p1_x(other.p1_x),
    p1_y(other.p1_y),
    p2_x(other.p2_x),
    p2_y(other.p2_y),
    p3_x(other.p3_x),
    p3_y(other.p3_y) {}

FieldTriangle::FieldTriangle(const QString &marking_name) :
    name(marking_name),
    p1_x(),
    p1_y(),
    p2_x(),
    p2_y(),
    p3_x(),
    p3_y() {}

FieldTriangle::~FieldTriangle() {

}
