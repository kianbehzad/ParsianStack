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
\file    soccerview.cpp
\brief   C++ Implementation: GLSoccerView
\author  Joydeep Biswas (C) 2011
*/
//========================================================================

#include "parsian_gui/interface/application/widgets/graphical_client/soccerview.h"

const double GLSoccerView::minZValue = -10;
const double GLSoccerView::maxZValue = 10;
const double GLSoccerView::FieldZ = 1.0;
const double GLSoccerView::RobotZ = 2.0;
const double GLSoccerView::BallZ = 3.0;
const int GLSoccerView::PreferedWidth = 1024;
const int GLSoccerView::PreferedHeight = 768;
const double GLSoccerView::MinRedrawInterval = 0.016; ///Minimum time between graphics updates (limits the fps)
const int GLSoccerView::unknownRobotID = -1;

GLSoccerView::FieldDimensions::FieldDimensions() :
  field_length(field._FIELD_WIDTH*1000),
  field_width(field._FIELD_HEIGHT*1000),
  boundary_width(field._BOUNDARY_WIDTH*1000) {
}

GLSoccerView::GLSoccerView(QWidget* parent) :
    QGLWidget(QGLFormat(
        QGL::DoubleBuffer | QGL::DepthBuffer | QGL::SampleBuffers),parent) {
    viewScale = (fieldDim.field_length + fieldDim.boundary_width) / sizeHint().width();
    viewScale = max(viewScale, (fieldDim.field_width + fieldDim.boundary_width) / sizeHint().height());

    viewXOffset = viewYOffset = 0.0;
    setAutoFillBackground(false); //Do not let painter auto fill the widget's background: we'll do it manually through openGl
    connect(this, SIGNAL(postRedraw()), this, SLOT(redraw()));
    blueRobotShape = GL_INVALID_VALUE;
    yellowRobotShape = GL_INVALID_VALUE;
    greyRobotShape = GL_INVALID_VALUE;
    blueCircleRobotShape = GL_INVALID_VALUE;
    yellowCircleRobotShape = GL_INVALID_VALUE;
    greyCircleRobotShape = GL_INVALID_VALUE;
    QFont RobotIDFont = this->font();
    RobotIDFont.setWeight(QFont::Bold);
    RobotIDFont.setPointSize(80);
    glText = GLText(RobotIDFont);
    tLastRedraw = 0;

    // worldmodel callback
    worldmodel_subscription = extern_interface_node->create_subscription<parsian_msgs::msg::ParsianWorldModel>("/world_model", 10, std::bind(&GLSoccerView::worldmodel_callback, this, _1));
    // geometry callback
    geometry_subscription = extern_interface_node->create_subscription<parsian_msgs::msg::SSLVisionGeometry>("/vision_geom", 10, std::bind(&GLSoccerView::geometry_callback, this, _1));

}

void GLSoccerView::redraw()
{
  if(GetTimeSec()-tLastRedraw<MinRedrawInterval)
    return;
  graphicsMutex.lock();
  update();
  graphicsMutex.unlock();
  tLastRedraw = GetTimeSec();
}


void GLSoccerView::mousePressEvent(QMouseEvent* event)
{
  leftButton = event->buttons().testFlag(Qt::LeftButton);
  midButton = event->buttons().testFlag(Qt::MidButton);
  rightButton = event->buttons().testFlag(Qt::RightButton);

  if(leftButton)
    setCursor(Qt::ClosedHandCursor);
  if(midButton)
    setCursor(Qt::SizeVerCursor);
  if(leftButton || midButton){
    // Start Pan / Zoom
    mouseStartX = event->x();
    mouseStartY = event->y();
    postRedraw();
  }
}

void GLSoccerView::mouseReleaseEvent(QMouseEvent* event)
{
  setCursor(Qt::ArrowCursor);
}

void GLSoccerView::mouseMoveEvent(QMouseEvent* event)
{
  static const bool debug = false;
  bool leftButton = event->buttons().testFlag(Qt::LeftButton);
  bool midButton = event->buttons().testFlag(Qt::MidButton);
  bool rightButton = event->buttons().testFlag(Qt::RightButton);

  if(debug) printf("MouseMove Event, Left:%d Mid:%d Right:%d\n", leftButton?1:0, midButton?1:0, rightButton?1:0);

  if(leftButton){
    //Pan
    viewXOffset -= viewScale*double(event->x() - mouseStartX);
    viewYOffset += viewScale*double(event->y() - mouseStartY);
    mouseStartX = event->x();
    mouseStartY = event->y();
    recomputeProjection();
    postRedraw();
  }else if(midButton){
    //Zoom
    double zoomRatio = double(event->y() - mouseStartY)/500.0;
    viewScale = viewScale*(1.0+zoomRatio);
    recomputeProjection();
    mouseStartX = event->x();
    mouseStartY = event->y();
    postRedraw();
  }
}

void GLSoccerView::wheelEvent(QWheelEvent* event)
{
  static const bool debug = false;
  double zoomRatio = -double(event->delta())/1000.0;
  viewScale = viewScale*(1.0+zoomRatio);
  recomputeProjection();
  if(debug) printf("Zoom: %5.3f\n",viewScale);
  postRedraw();
}

void GLSoccerView::keyPressEvent(QKeyEvent* event)
{
  static const bool debug = false;
  if(debug) printf("KeyPress: 0x%08X\n",event->key());
  if(event->key() == Qt::Key_Space)
    resetView();
  if(event->key() == Qt::Key_Escape)
    close();
}

void GLSoccerView::resetView()
{
  viewScale =
      (fieldDim.field_length + fieldDim.boundary_width) / width();
  viewScale = max(viewScale,
                  (fieldDim.field_width + fieldDim.boundary_width) / height());

  viewXOffset = viewYOffset = 0.0;
  recomputeProjection();
  postRedraw();
}

void GLSoccerView::resizeEvent(QResizeEvent* event)
{
  QGLWidget::resizeEvent(event);
  redraw();
}

void GLSoccerView::recomputeProjection()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-0.5*viewScale*width()+viewXOffset, 0.5*viewScale*width()+viewXOffset, -0.5*viewScale*height()+viewYOffset, 0.5*viewScale*height()+viewYOffset, minZValue, maxZValue);
  glMatrixMode(GL_MODELVIEW);
}

void GLSoccerView::resizeGL(int width, int height)
{
  glViewport(0, 0, width, height);
  recomputeProjection();
}

void GLSoccerView::initializeGL()
{
  blueRobotShape = glGenLists(1);
  if(blueRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(blueRobotShape, GL_COMPILE);
  drawRobot(teamBlue,true,false);
  glEndList();

  yellowRobotShape = glGenLists(1);
  if(yellowRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(yellowRobotShape, GL_COMPILE);
  drawRobot(teamYellow,true,false);
  glEndList();

  greyRobotShape = glGenLists(1);
  if(greyRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(greyRobotShape, GL_COMPILE);
  drawRobot(teamUnknown,true,false);
  glEndList();

  blueCircleRobotShape = glGenLists(1);
  if(blueRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(blueCircleRobotShape, GL_COMPILE);
  drawRobot(teamBlue,false,false);
  glEndList();

  yellowCircleRobotShape = glGenLists(1);
  if(yellowRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(yellowCircleRobotShape, GL_COMPILE);
  drawRobot(teamYellow,false,false);
  glEndList();

  greyCircleRobotShape = glGenLists(1);
  if(greyRobotShape==GL_INVALID_VALUE){
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(greyCircleRobotShape, GL_COMPILE);
  drawRobot(teamUnknown,false,false);
  glEndList();
}


void GLSoccerView::vectorTextTest()
{
  #define TextTest(loc,angle,size,str,halign,valign) \
  {glText.drawString((loc),angle,size,str,halign,valign); \
  rcsc::Vector2D l1,l2; \
  l1.setDir(M_PI/180.0*angle); \
  l1 = loc+l1*size*(glText.getWidth(str)); \
  l2.setDir(M_PI/180.0*angle); \
  l2 = loc-l2*size*(glText.getWidth(str)); \
  glBegin(GL_LINES); \
  glVertex3d(l1.x,l1.y,9); \
  glVertex3d(l2.x,l2.y,9); \
  glEnd();}

  glColor3d(1,1,1);
  TextTest(rcsc::Vector2D(1,1)*353.6,45,500,"123agdo0",GLText::LeftAligned,GLText::MedianAligned)
  TextTest(rcsc::Vector2D(fieldDim.field_length*0.5,0),0,500,"123agdo0",GLText::RightAligned,GLText::BottomAligned)
  TextTest(rcsc::Vector2D(0,-fieldDim.field_width*0.5),0,500,"123agdo0",GLText::CenterAligned,GLText::TopAligned)
  TextTest(rcsc::Vector2D(-fieldDim.field_length*0.5,0),0,500,"1\ub023agdo0",GLText::CenterAligned,GLText::MiddleAligned)
}

void GLSoccerView::paintEvent(QPaintEvent* event)
{
  graphicsMutex.lock();
  makeCurrent();
  glClearColor(FIELD_COLOR);
  glShadeModel(GL_SMOOTH);
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  drawFieldLines(fieldDim);
  drawRobots();
//  drawBalls();
  //vectorTextTest();
  glPopMatrix();
  swapBuffers();
  graphicsMutex.unlock();
}

void GLSoccerView::drawQuad(rcsc::Vector2D loc1, rcsc::Vector2D loc2, double z)
{
  glBegin(GL_QUADS);
  glVertex3d(loc1.x,loc1.y,z);
  glVertex3d(loc2.x,loc1.y,z);
  glVertex3d(loc2.x,loc2.y,z);
  glVertex3d(loc1.x,loc2.y,z);
  glEnd();
}

void GLSoccerView::drawArc(rcsc::Vector2D loc, double r1, double r2, double theta1, double theta2, double z, double dTheta)
{
  static const double tesselation = 1.0;
  if(dTheta<0){
    dTheta = tesselation/r2;
  }
  glBegin(GL_QUAD_STRIP);
  for(double theta=theta1; theta<theta2; theta+=dTheta){
    double c1 = cos(theta), s1 = sin(theta);
    glVertex3d(r2*c1+loc.x,r2*s1+loc.y,z);
    glVertex3d(r1*c1+loc.x,r1*s1+loc.y,z);
  }
  double c1 = cos(theta2), s1 = sin(theta2);
  glVertex3d(r2*c1+loc.x,r2*s1+loc.y,z);
  glVertex3d(r1*c1+loc.x,r1*s1+loc.y,z);
  glEnd();
}

void GLSoccerView::drawRobot(int team, bool hasAngle, bool useDisplayLists)
{
  if(useDisplayLists){
    switch ( team ){
      case teamBlue:{
        if(hasAngle)
          glCallList(blueRobotShape);
        else
          glCallList(blueCircleRobotShape);
        break;
      }
      case teamYellow:{
        if(hasAngle)
          glCallList(yellowRobotShape);
        else
          glCallList(yellowCircleRobotShape);
        break;
      }
      default:{
        if(hasAngle)
          glCallList(greyRobotShape);
        else
          glCallList(greyCircleRobotShape);
        break;
      }
    }
    return;
  }

  switch ( team ){
    case teamBlue:{
      glColor3d(0.2549, 0.4941, 1.0);
      break;
    }
    case teamYellow:{
      glColor3d(1.0, 0.9529, 0.2431);
      break;
    }
    default:{
      glColor3d(0.5882,0.5882,0.5882);
      break;
    }
  }
  double theta1 = hasAngle?40*knowledge::PI/180.0:0.0;
  double theta2 = 2.0*M_PI - theta1;
  drawArc(0,0,0,90,theta1, theta2, RobotZ);
  if(hasAngle){
    glBegin(GL_TRIANGLES);
    glVertex3d(0,0,RobotZ);
    glVertex3d(90.0*cos(theta1),90.0*sin(theta1),RobotZ);
    glVertex3d(90.0*cos(theta2),90.0*sin(theta2),RobotZ);
    glEnd();
  }

  switch ( team ){
    case teamBlue:{
      glColor3d(0.0706, 0.2314, 0.6275);
      break;
    }
    case teamYellow:{
      glColor3d(0.8, 0.6157, 0.0);
      break;
    }
    default:{
      glColor3d(0.2745,0.2745,0.2745);
      break;
    }
  }
  drawArc(0,0,80,90,theta1, theta2, RobotZ+0.01);
  if(hasAngle)
    drawQuad(90.0*cos(theta1)-10,90.0*sin(theta1), 90.0*cos(theta2),90.0*sin(theta2),RobotZ+0.01);
}

void GLSoccerView::drawRobot(rcsc::Vector2D loc, double theta, double conf, int robotID, int team, bool hasAngle)
{
  glPushMatrix();
  glLoadIdentity();
  glTranslated(loc.x,loc.y,0);
  switch ( team ){
    case teamBlue:{
      glColor3d(0.2549, 0.4941, 1.0);
      break;
    }
    case teamYellow:{
      glColor3d(1.0, 0.9529, 0.2431);
      break;
    }
    default:{
      glColor3d(0.5882,0.5882,0.5882);
      break;
    }
  }
  drawQuad(-90,130,-90.0+180.0*conf,160,RobotZ);
  glColor3d(0.0,0.0,0.0);
  char buf[1024];
  if(robotID!=unknownRobotID)
    snprintf(buf,1023,"%X",robotID);
  else
    snprintf(buf,1023,"?");
  glText.drawString(loc,0,100,buf,GLText::CenterAligned,GLText::MiddleAligned);
  switch ( team ){
    case teamBlue:{
      glColor3d(0.0706, 0.2314, 0.6275);
      break;
    }
    case teamYellow:{
      glColor3d(0.8, 0.6157, 0.0);
      break;
    }
    default:{
      glColor3d(0.2745,0.2745,0.2745);
      break;
    }
  }
  drawQuad(-96,124,96.0,130,RobotZ+0.01);
  drawQuad(-96,124,-90.0,166,RobotZ+0.01);
  drawQuad(-96,160,96.0,166,RobotZ+0.01);
  drawQuad(90,124,96.0,166,RobotZ+0.01);

  glRotated(theta,0,0,1.0);
  drawRobot(team, hasAngle, true);
  glPopMatrix();
}

void GLSoccerView::drawFieldLines(FieldDimensions& dimensions)
{
  glColor4f(FIELD_LINES_COLOR);
  for (size_t i = 0; i < fieldDim.lines.size(); ++i) {
    const FieldLine& line = *fieldDim.lines[i];
    const double half_thickness = 0.5 * line.thickness;
    const rcsc::Vector2D p1(line.p1_x, line.p1_y);
    const rcsc::Vector2D p2(line.p2_x, line.p2_y);
    const rcsc::Vector2D perp = (p2 - p1).norm().rotate(90);
    const rcsc::Vector2D corner1 = p1 - half_thickness * perp;
    const rcsc::Vector2D corner2 = p2 + half_thickness * perp;
    drawQuad(corner1, corner2, FieldZ);
//      qDebug() << "(" << corner1.x << ", " << corner1.y << ")    (" << corner2.x << ", " << corner2.y << ")";
  }
//    qDebug() << "**********************";

  for (size_t i = 0; i < fieldDim.arcs.size(); ++i) {
    const FieldCircularArc& arc = *fieldDim.arcs[i];
    const double half_thickness = 0.5 * arc.thickness;
    const double radius = arc.radius;
    const rcsc::Vector2D center(arc.center_x, arc.center_y);
    const double a1 = arc.a1;
    const double a2 = arc.a2;
    drawArc(center, radius - half_thickness, radius + half_thickness, a1, a2,
            FieldZ);
  }
}

void GLSoccerView::drawBall(rcsc::Vector2D loc)
{
  glColor3d(1.0,0.5059,0.0);
  drawArc(loc,0,16,-M_PI,M_PI,BallZ);
  glColor3d(0.8706,0.3490,0.0);
  drawArc(loc,15,21,-M_PI,M_PI,BallZ);

}

void GLSoccerView::drawBalls()
{
  for(int i=0; i<balls.size(); i++){
    for(int j=0; j<balls[i].size(); j++){
      drawBall(balls[i][j]);
    }
  }
}

void GLSoccerView::drawRobots()
{
  for(int i=0; i<robots.size(); i++){
    for(int j=0; j<robots[i].size(); j++){
      Robot r = robots[i][j];
      drawRobot(r.loc,r.angle,r.conf,r.id,r.team,r.hasAngle);
    }
  }
}

void GLSoccerView::worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg)
{
    std::vector<parsian_msgs::msg::ParsianRobot>& yellow_robots = extern_is_our_color_yellow ? msg->our : msg->opp;
    std::vector<parsian_msgs::msg::ParsianRobot>& blue_robots = extern_is_our_color_yellow ? msg->opp : msg->our;
    int numBlueRobots =  blue_robots.size();
    int numYellowRobots =  yellow_robots.size();
    int numBalls = 1;
    parsian_msgs::msg::ParsianRobot sslRobot;
    Robot robot;
    parsian_msgs::msg::ParsianRobot sslBall;
    rcsc::Vector2D ball;
    int cam = 0;

    graphicsMutex.lock();
    if(cam+1>robots.size()){
    robots.resize(cam+1);
    }
    if(cam+1>balls.size()){
    balls.resize(cam+1);
    }

    robots[cam].clear();
    balls[cam].clear();
    for(int i=0; i<numBlueRobots; i++){
        sslRobot = blue_robots[i];
        robot.loc.assign(sslRobot.pos.x*1000, sslRobot.pos.y*1000);
        robot.id = sslRobot.id;

        robot.hasAngle = true;
        robot.angle = Vector2D(sslRobot.dir.x, sslRobot.dir.y).dir().degree();
        robot.team = teamBlue;
        robot.cameraID = 0;
        robot.conf = 1;
        robots[cam].append(robot);
    }

    for(int i=0; i<numYellowRobots; i++){
        sslRobot = yellow_robots[i];
        robot.loc.assign(sslRobot.pos.x*1000, sslRobot.pos.y*1000);
        robot.id = sslRobot.id;
        robot.hasAngle = true;
        robot.angle = Vector2D(sslRobot.dir.x, sslRobot.dir.y).dir().degree();
        robot.team = teamYellow;
        robot.cameraID = 0;
        robot.conf = 1;
        robots[cam].append(robot);
    }

    for(int i=0; i<numBalls; i++){
    sslBall = msg->ball;
    ball.assign(sslBall.pos.x*1000, sslBall.pos.y*1000);
    balls[cam].append(ball);

    }
    graphicsMutex.unlock();
    postRedraw();
}
//
void GLSoccerView::geometry_callback(const parsian_msgs::msg::SSLVisionGeometry::SharedPtr msg)
{
  graphicsMutex.lock();
  for (size_t i = 0; i < fieldDim.lines.size(); ++i) {
    delete fieldDim.lines[i];
  }
  fieldDim.lines.clear();
//  for (int i = 0; i < fieldSize.field_lines_size(); ++i) {
//    const SSL_FieldLineSegment& line = fieldSize.field_lines(i);
//    fieldDim.lines.push_back(new FieldLine(
//        line.name(), line.p1().x(), line.p1().y(),
//        line.p2().x(), line.p2().y(), line.thickness()));
//  }
  fieldDim.lines.push_back(new FieldLine(
          QString::fromStdString(msg->field.top_line.name), msg->field.top_line.begin.x*1000, msg->field.top_line.begin.y*1000,
          msg->field.top_line.end.x*1000, msg->field.top_line.end.y*1000, msg->field.top_line.thickness*1000));
  fieldDim.lines.push_back(new FieldLine(
          QString::fromStdString(msg->field.bottom_line.name), msg->field.bottom_line.begin.x*1000, msg->field.bottom_line.begin.y*1000,
          msg->field.bottom_line.end.x*1000, msg->field.bottom_line.end.y*1000, msg->field.bottom_line.thickness*1000));
  fieldDim.lines.push_back(new FieldLine(
          QString::fromStdString(msg->field.left_line.name), msg->field.left_line.begin.x*1000, msg->field.left_line.begin.y*1000,
          msg->field.left_line.end.x*1000, msg->field.left_line.end.y*1000, msg->field.left_line.thickness*1000));
  fieldDim.lines.push_back(new FieldLine(
          QString::fromStdString(msg->field.right_line.name), msg->field.right_line.begin.x*1000, msg->field.right_line.begin.y*1000,
          msg->field.right_line.end.x*1000, msg->field.right_line.end.y*1000, msg->field.right_line.thickness*1000));
  fieldDim.lines.push_back(new FieldLine(
          QString::fromStdString(msg->field.half_line.name), msg->field.half_line.begin.x*1000, msg->field.half_line.begin.y*1000,
          msg->field.half_line.end.x*1000, msg->field.half_line.end.y*1000, msg->field.half_line.thickness*1000));
  fieldDim.lines.push_back(new FieldLine(
          QString::fromStdString(msg->field.center_line.name), msg->field.center_line.begin.x*1000, msg->field.center_line.begin.y*1000,
          msg->field.center_line.end.x*1000, msg->field.center_line.end.y*1000, msg->field.center_line.thickness*1000));
  fieldDim.lines.push_back(new FieldLine(
          QString::fromStdString(msg->field.left_penalty_line.name), msg->field.left_penalty_line.begin.x*1000, msg->field.left_penalty_line.begin.y*1000,
          msg->field.left_penalty_line.end.x*1000, msg->field.left_penalty_line.end.y*1000, msg->field.left_penalty_line.thickness*1000));
  fieldDim.lines.push_back(new FieldLine(
          QString::fromStdString(msg->field.right_penalty_line.name), msg->field.right_penalty_line.begin.x*1000, msg->field.right_penalty_line.begin.y*1000,
          msg->field.right_penalty_line.end.x*1000, msg->field.right_penalty_line.end.y*1000, msg->field.right_penalty_line.thickness*1000));



  for (size_t i = 0; i < fieldDim.arcs.size(); ++i) {
    delete fieldDim.arcs[i];
  }
  fieldDim.arcs.clear();
//  for (int i = 0; i < fieldSize.field_arcs_size(); ++i) {
//    const SSL_FieldCicularArc& arc = fieldSize.field_arcs(i);
//    fieldDim.arcs.push_back(new FieldCircularArc(
//        arc.name(), arc.center().x(), arc.center().y(),  arc.radius(),
//        arc.a1(), arc.a2(), arc.thickness()));
//  }
    fieldDim.arcs.push_back(new FieldCircularArc(
            QString::fromStdString(msg->field.top_left_penalty_arc.name), msg->field.top_left_penalty_arc.center.x*1000, msg->field.top_left_penalty_arc.center.y,
            msg->field.top_left_penalty_arc.radius*1000, msg->field.top_left_penalty_arc.a1*1000, msg->field.top_left_penalty_arc.a2*1000, msg->field.top_left_penalty_arc.thickness*1000));
    fieldDim.arcs.push_back(new FieldCircularArc(
            QString::fromStdString(msg->field.bottom_left_penalty_arc.name), msg->field.bottom_left_penalty_arc.center.x*1000, msg->field.bottom_left_penalty_arc.center.y*1000,
            msg->field.bottom_left_penalty_arc.radius*1000, msg->field.bottom_left_penalty_arc.a1*1000, msg->field.bottom_left_penalty_arc.a2*1000, msg->field.bottom_left_penalty_arc.thickness*1000));
    fieldDim.arcs.push_back(new FieldCircularArc(
            QString::fromStdString(msg->field.top_right_penalty_arc.name), msg->field.top_right_penalty_arc.center.x*1000, msg->field.top_right_penalty_arc.center.y*1000,
            msg->field.top_right_penalty_arc.radius*1000, msg->field.top_right_penalty_arc.a1*1000, msg->field.top_right_penalty_arc.a2*1000, msg->field.top_right_penalty_arc.thickness*1000));
    fieldDim.arcs.push_back(new FieldCircularArc(
            QString::fromStdString(msg->field.bottom_right_penalty_arc.name), msg->field.bottom_right_penalty_arc.center.x*1000, msg->field.bottom_right_penalty_arc.center.y*1000,
            msg->field.bottom_right_penalty_arc.radius*1000, msg->field.bottom_right_penalty_arc.a1*1000, msg->field.bottom_right_penalty_arc.a2*1000, msg->field.bottom_right_penalty_arc.thickness*1000));
    fieldDim.arcs.push_back(new FieldCircularArc(
            QString::fromStdString(msg->field.center_circle.name), msg->field.center_circle.center.x*1000, msg->field.center_circle.center.y*1000,
            msg->field.center_circle.radius*1000, msg->field.center_circle.a1*1000, msg->field.center_circle.a2*1000, msg->field.center_circle.thickness*1000));

  graphicsMutex.unlock();
}
