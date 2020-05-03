#ifndef FIELD_H
#define FIELD_H

#include <QString>

class FieldLine {
public:
    FieldLine(QString name_,
              double p1_x_,
              double p1_y_,
              double p2_x_,
              double p2_y_,
              double thickness_);
private:
    // Disable assignment operator.
    const FieldLine& operator=(const FieldLine& other);
public:
    QString name;
    double p1_x;
    double p1_y;
    double p2_x;
    double p2_y;
    double thickness;

    FieldLine(const FieldLine& other);
    FieldLine(const QString &marking_name);

    ~FieldLine();

};

class FieldCircularArc {
public:
    FieldCircularArc(QString name_,
                     double center_x_,
                     double center_y_,
                     double radius_,
                     double a1_,
                     double a2_,
                     double thickness_);

private:
    // Disable assignment operator.
    const FieldCircularArc& operator=(const FieldCircularArc& other);

public:
    QString name;
    double center_x;
    double center_y;
    double radius;
    double a1;
    double a2;
    double thickness;

    FieldCircularArc(const FieldCircularArc& other);
    FieldCircularArc(const QString &marking_name);

    ~FieldCircularArc();

};

class FieldTriangle {
public:

    FieldTriangle(QString name_,
                  double p1_x_,
                  double p1_y_,
                  double p2_x_,
                  double p2_y_,
                  double p3_x_,
                  double p3_y_);

private:
    // Disable assignment operator.
    const FieldTriangle& operator=(const FieldCircularArc& other);

public:
    QString name;
    double p1_x;
    double p1_y;
    double p2_x;
    double p2_y;
    double p3_x;
    double p3_y;

    FieldTriangle(const FieldTriangle& other);
    FieldTriangle(const QString &marking_name);

    ~FieldTriangle();

};


#endif // FIELD_H
