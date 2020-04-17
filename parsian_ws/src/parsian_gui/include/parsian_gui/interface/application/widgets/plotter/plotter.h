/***************************************************************************
 *   Copyright 2015 Michael Eischer, Philipp Nordhus                       *
 *   Robotics Erlangen e.V.                                                *
 *   http://www.robotics-erlangen.de/                                      *
 *   info@robotics-erlangen.de                                             *
 *                                                                         *
 *   This program is free software: you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation, either version 3 of the License, or     *
 *   any later version.                                                    *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef PLOTTER_H
#define PLOTTER_H

#include <QDialog>
#include <QSet>
#include <QStandardItemModel>

#include "parsian_gui/interface/application/extern_variables.h"
#include "parsian_msgs/msg/parsian_world_model.hpp"

#include "rclcpp/rclcpp.hpp"
#include "google/protobuf/descriptor.h"

#define MSG WorldModel

class LeafFilterProxyModel;
class Plot;
class GuiTimer;
class QMenu;
namespace Ui {
    class Plotter;
}

class Plotter : public QWidget
{
    Q_OBJECT

public:
    explicit Plotter();
    ~Plotter() override;

public slots:
    void setScaling(float min, float max, float timespan);
//    void handleStatus(WorldModel *status);
    void clearData();

signals:
    void addPlot(const Plot *plot);
    void removePlot(const Plot *plot);

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void updateScrollBar();
    void updateOffset(int pos);

    void setFreeze(bool freeze);
    void plotContextMenu(const QPoint &pos);
    void clearSelection();
    void itemChanged(QStandardItem *item);
    void invalidatePlots();

private:
    bool eventFilter(QObject *obj, QEvent *event) override;

    void loadSelection();
    QStandardItem* getItem(const QString &name);
    void addRootItem(const QString &name, const QString &displayName);
//    void parseMessage(const google::protobuf::Message &message, const QString &parent, float time);
    void parseVector2D(const parsian_msgs::msg::Vector2D& vec, const QString& parent, float time);
    void parseParsianRobot(const parsian_msgs::msg::ParsianRobot& rob, const QString& parent, float time);
    void addPoint(const std::string &name, const QString &parent, float time, float value, QVector<QStandardItem *> &childLookup, int descriptorIndex);
    void tryAddLength(const std::string &name, const QString &parent, float time, float value1, float value2, QVector<QStandardItem *> &childLookup, int descriptorIndex);

private:
    void worldmodel_callback(const parsian_msgs::msg::ParsianWorldModel::SharedPtr msg);
    rclcpp::Subscription<parsian_msgs::msg::ParsianWorldModel>::SharedPtr worldmodel_subscription;


    enum ItemRole {
        FullNameRole = Qt::UserRole + 2
    };

    Ui::Plotter *ui;
    qint64 m_startTime;
    qint64 m_time;
    double m_timeLimit;
    bool m_freeze;
    GuiTimer *m_guiTimer;
    QHash<QString, QStandardItem*> m_items;
    QHash<QString, QVector<QStandardItem *>> m_itemLookup;
    QHash<QStandardItem*, Plot*> m_plots;
    QHash<QStandardItem*, Plot*> m_frozenPlots;
    QSet<QString> m_selection;
    QStandardItemModel m_model;
    LeafFilterProxyModel *m_proxy;
    QMenu *m_plotMenu;
    int cnt;
};

#endif // PLOTTER_H
