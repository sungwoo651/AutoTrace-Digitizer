/******************************************************************************************************
 * (C) 2026. This file is part of Engauge Digitizer, which is released under GNU General Public       *
 * License version 2 (GPLv2) or (at your option) any later version. See file LICENSE for details.     *
 ******************************************************************************************************/

#ifndef AUTO_DIGITIZE_H
#define AUTO_DIGITIZE_H

#include <QImage>
#include <QList>
#include <QPoint>
#include <QPointF>
#include <QRect>
#include <QString>

class Transformation;

struct AutoCurveGroup
{
  AutoCurveGroup();

  QString name;
  QList<QPoint> points;
  double confidence;
};

struct AutoCurveResult
{
  AutoCurveResult();

  QList<AutoCurveGroup> groups;
  QString message;
  int rawCandidateCount;
  int rejectedLineLikeCount;
  int rejectedTextLikeCount;
  int finalCandidateCount;
};

struct AutoAxisResult
{
  AutoAxisResult();

  bool success;
  bool rotated;
  double rotationDegrees;
  QImage image;
  QList<QPointF> pointsScreen;
  QList<QPointF> pointsGraph;
  QString message;
};

class AutoDigitize
{
public:
  static AutoAxisResult detectAxes(const QImage &image,
                                   double yMaximum);

  static QList<QPoint> detectCurvePoints(const QImage &image,
                                         const QRect &plotRect,
                                         const Transformation &transformation,
                                         double yMinimum,
                                         double yMaximum);

  static AutoCurveResult detectCurvePointGroups(const QImage &image,
                                                const QRect &plotRect,
                                                const Transformation &transformation,
                                                double yMinimum,
                                                double yMaximum);

  static AutoCurveResult detectCurvePointGroupFromExample(const QImage &image,
                                                          const QRect &plotRect,
                                                          const Transformation &transformation,
                                                          double yMinimum,
                                                          double yMaximum,
                                                          const QPoint &examplePoint);

private:
  AutoDigitize();
};

#endif // AUTO_DIGITIZE_H
