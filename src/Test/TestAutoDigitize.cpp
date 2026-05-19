#include "AutoDigitize.h"
#include "DocumentModelCoords.h"
#include "DocumentModelGeneral.h"
#include "Logger.h"
#include "MainWindowModel.h"
#include "Test/TestAutoDigitize.h"
#include "Transformation.h"
#include <QImage>
#include <QPainter>
#include <QPolygon>
#include <QStringList>
#include <QtTest/QtTest>

QTEST_MAIN (TestAutoDigitize)

namespace {

QList<QPoint> allPointsFromGroups(const QList<AutoCurveGroup> &groups)
{
  QList<QPoint> points;
  for (int groupIndex = 0; groupIndex < groups.count(); ++groupIndex) {
    for (int pointIndex = 0; pointIndex < groups.at(groupIndex).points.count(); ++pointIndex) {
      points << groups.at(groupIndex).points.at(pointIndex);
    }
  }

  return points;
}

QString pointText(const QPoint &point)
{
  return QString ("(%1,%2)").arg(point.x()).arg(point.y());
}

QString groupsText(const QList<AutoCurveGroup> &groups)
{
  QStringList lines;
  for (int groupIndex = 0; groupIndex < groups.count(); ++groupIndex) {
    QStringList points;
    for (int pointIndex = 0; pointIndex < groups.at(groupIndex).points.count(); ++pointIndex) {
      points << pointText(groups.at(groupIndex).points.at(pointIndex));
    }
    lines << QString("%1:%2:%3")
             .arg(groupIndex)
             .arg(groups.at(groupIndex).name)
             .arg(points.join(","));
  }

  return lines.join(" | ");
}

int matchCount(const QList<QPoint> &points,
               const QList<QPoint> &expected,
               int tolerance)
{
  int count = 0;
  const int toleranceSquared = tolerance * tolerance;
  for (int expectedIndex = 0; expectedIndex < expected.count(); ++expectedIndex) {
    for (int pointIndex = 0; pointIndex < points.count(); ++pointIndex) {
      const QPoint delta = points.at(pointIndex) - expected.at(expectedIndex);
      if (delta.x() * delta.x() + delta.y() * delta.y() <= toleranceSquared) {
        ++count;
        break;
      }
    }
  }

  return count;
}

QList<QPoint> bestMatchingGroupPoints(const QList<AutoCurveGroup> &groups,
                                      const QList<QPoint> &expected,
                                      int tolerance)
{
  QList<QPoint> best;
  int bestCount = -1;
  for (int index = 0; index < groups.count(); ++index) {
    const int count = matchCount(groups.at(index).points,
                                 expected,
                                 tolerance);
    if (count > bestCount) {
      bestCount = count;
      best = groups.at(index).points;
    }
  }

  return best;
}

} // namespace

TestAutoDigitize::TestAutoDigitize(QObject *parent) :
  QObject(parent)
{
}

void TestAutoDigitize::cleanupTestCase ()
{
}

void TestAutoDigitize::initTestCase ()
{
  initializeLogging ("engauge_test",
                     "engauge_test.log",
                     false);
}

bool TestAutoDigitize::containsPointNear(const QList<QPoint> &points,
                                         const QPoint &expected,
                                         int tolerance) const
{
  const int toleranceSquared = tolerance * tolerance;
  for (int index = 0; index < points.count(); ++index) {
    const QPoint delta = points.at(index) - expected;
    if (delta.x() * delta.x() + delta.y() * delta.y() <= toleranceSquared) {
      return true;
    }
  }

  return false;
}

QImage TestAutoDigitize::regressionImage() const
{
  QImage image(850, 270, QImage::Format_ARGB32);
  image.fill(Qt::white);

  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, false);

  painter.setPen(QPen(Qt::black, 1));
  painter.drawLine(55, 239, 789, 239);
  painter.drawLine(55, 31, 55, 239);

  QPen gridPen(QColor(190, 190, 190), 1, Qt::DashLine);
  painter.setPen(gridPen);
  painter.drawLine(55, 135, 789, 135);
  painter.drawLine(420, 31, 420, 239);

  painter.setPen(QPen(Qt::black, 1, Qt::DotLine));
  painter.drawLine(214, 31, 214, 239);
  painter.setPen(QPen(Qt::black, 2));
  painter.drawLine(779, 31, 779, 239);

  painter.setPen(QPen(Qt::black, 1));
  painter.setBrush(Qt::NoBrush);
  painter.drawRect(702, 204, 70, 24);
  painter.drawText(QRect(704, 204, 66, 24),
                   Qt::AlignCenter,
                   "P1 Box");

  painter.setPen(QPen(Qt::black, 1));
  painter.setBrush(Qt::black);
  const QList<QPoint> filledCircles = {
    QPoint(96, 220),
    QPoint(150, 207),
    QPoint(318, 170),
    QPoint(475, 142)
  };
  for (int index = 0; index < filledCircles.count(); ++index) {
    painter.drawEllipse(filledCircles.at(index), 5, 5);
  }

  painter.setBrush(Qt::NoBrush);
  const QList<QPoint> openTriangles = {
    QPoint(116, 194),
    QPoint(250, 181),
    QPoint(370, 155),
    QPoint(560, 132)
  };
  for (int index = 0; index < openTriangles.count(); ++index) {
    const QPoint center = openTriangles.at(index);
    QPolygon triangle;
    triangle << QPoint(center.x(), center.y() - 6)
             << QPoint(center.x() - 6, center.y() + 6)
             << QPoint(center.x() + 6, center.y() + 6);
    painter.drawPolygon(triangle);
  }

  painter.end();
  return image;
}

Transformation TestAutoDigitize::regressionTransformation() const
{
  DocumentModelCoords modelCoords;
  modelCoords.setCoordScaleXTheta(COORD_SCALE_LINEAR);
  modelCoords.setCoordScaleYRadius(COORD_SCALE_LINEAR);
  modelCoords.setCoordsType(COORDS_TYPE_CARTESIAN);
  modelCoords.setCoordUnitsDate(COORD_UNITS_DATE_YEAR_MONTH_DAY);
  modelCoords.setCoordUnitsRadius(COORD_UNITS_NON_POLAR_THETA_NUMBER);
  modelCoords.setCoordUnitsTheta(COORD_UNITS_POLAR_THETA_DEGREES);
  modelCoords.setCoordUnitsTime(COORD_UNITS_TIME_HOUR_MINUTE_SECOND);
  modelCoords.setCoordUnitsX(COORD_UNITS_NON_POLAR_THETA_NUMBER);
  modelCoords.setCoordUnitsY(COORD_UNITS_NON_POLAR_THETA_NUMBER);
  modelCoords.setOriginRadius(0.0);

  DocumentModelGeneral modelGeneral;
  modelGeneral.setCursorSize(5);
  modelGeneral.setExtraPrecision(1);

  MainWindowModel mainWindowModel;
  QTransform matrixScreen(55, 789, 55,
                          239, 239, 31,
                          1, 1, 1);
  QTransform matrixGraph(1, 735, 1,
                         0, 0, 100,
                         1, 1, 1);

  Transformation transformation;
  transformation.setModelCoords(modelCoords,
                                modelGeneral,
                                mainWindowModel);
  transformation.updateTransformFromMatrices(matrixScreen,
                                             matrixGraph);
  return transformation;
}

QImage TestAutoDigitize::singleCaseCircleImage() const
{
  QImage image(850, 259, QImage::Format_ARGB32);
  image.fill(Qt::white);

  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, false);

  painter.setPen(QPen(Qt::black, 1));
  painter.drawLine(55, 239, 789, 239);
  painter.drawLine(55, 31, 55, 239);

  for (int y = 31; y <= 239; y += 52) {
    painter.drawLine(51, y, 55, y);
  }
  for (int x = 84; x <= 789; x += 28) {
    painter.drawLine(x, 239, x, 244);
  }

  painter.drawText(12, 36, "100");
  painter.drawText(21, 88, "75");
  painter.drawText(21, 140, "50");
  painter.drawText(21, 192, "25");
  painter.drawText(31, 202, "0");

  painter.setPen(QPen(Qt::black, 1, Qt::DashLine));
  painter.drawLine(214, 24, 214, 239);
  painter.drawLine(782, 24, 782, 239);

  painter.setPen(QPen(Qt::black, 1));
  painter.setBrush(Qt::NoBrush);
  painter.drawRect(706, 207, 70, 23);
  painter.drawText(QRect(706, 207, 70, 23),
                   Qt::AlignCenter,
                   "Ann");

  const QList<QPoint> baselineOpenCircles = {
    QPoint(84, 197),
    QPoint(112, 197),
    QPoint(140, 197),
    QPoint(168, 197),
    QPoint(196, 197)
  };
  const QList<QPoint> interventionOpenCircles = {
    QPoint(251, 86),
    QPoint(307, 44),
    QPoint(363, 44),
    QPoint(419, 31),
    QPoint(447, 31)
  };
  const QList<QPoint> filledCircles = {
    QPoint(223, 59),
    QPoint(279, 44),
    QPoint(335, 31),
    QPoint(391, 31)
  };

  painter.setPen(QPen(Qt::black, 1));
  for (int index = 1; index < baselineOpenCircles.count(); ++index) {
    painter.drawLine(baselineOpenCircles.at(index - 1),
                     baselineOpenCircles.at(index));
  }

  QList<QPoint> interventionSeries;
  interventionSeries << filledCircles.at(0)
                     << interventionOpenCircles.at(0)
                     << filledCircles.at(1)
                     << interventionOpenCircles.at(1)
                     << filledCircles.at(2)
                     << interventionOpenCircles.at(2)
                     << filledCircles.at(3)
                     << interventionOpenCircles.at(3)
                     << interventionOpenCircles.at(4);
  for (int index = 1; index < interventionSeries.count(); ++index) {
    painter.drawLine(interventionSeries.at(index - 1),
                     interventionSeries.at(index));
  }

  painter.setBrush(Qt::NoBrush);
  for (int index = 0; index < baselineOpenCircles.count(); ++index) {
    painter.drawEllipse(baselineOpenCircles.at(index), 7, 7);
  }
  for (int index = 0; index < interventionOpenCircles.count(); ++index) {
    painter.drawEllipse(interventionOpenCircles.at(index), 7, 7);
  }

  painter.setBrush(Qt::black);
  for (int index = 0; index < filledCircles.count(); ++index) {
    painter.drawEllipse(filledCircles.at(index), 7, 7);
  }

  QPolygon triangle;
  triangle << QPoint(808, 23)
           << QPoint(799, 40)
           << QPoint(817, 40);
  painter.drawPolygon(triangle);

  painter.end();
  return image;
}

Transformation TestAutoDigitize::singleCaseCircleTransformation() const
{
  DocumentModelCoords modelCoords;
  modelCoords.setCoordScaleXTheta(COORD_SCALE_LINEAR);
  modelCoords.setCoordScaleYRadius(COORD_SCALE_LINEAR);
  modelCoords.setCoordsType(COORDS_TYPE_CARTESIAN);
  modelCoords.setCoordUnitsDate(COORD_UNITS_DATE_YEAR_MONTH_DAY);
  modelCoords.setCoordUnitsRadius(COORD_UNITS_NON_POLAR_THETA_NUMBER);
  modelCoords.setCoordUnitsTheta(COORD_UNITS_POLAR_THETA_DEGREES);
  modelCoords.setCoordUnitsTime(COORD_UNITS_TIME_HOUR_MINUTE_SECOND);
  modelCoords.setCoordUnitsX(COORD_UNITS_NON_POLAR_THETA_NUMBER);
  modelCoords.setCoordUnitsY(COORD_UNITS_NON_POLAR_THETA_NUMBER);
  modelCoords.setOriginRadius(0.0);

  DocumentModelGeneral modelGeneral;
  modelGeneral.setCursorSize(5);
  modelGeneral.setExtraPrecision(1);

  MainWindowModel mainWindowModel;
  QTransform matrixScreen(55, 789, 55,
                          239, 239, 31,
                          1, 1, 1);
  QTransform matrixGraph(0, 27, 0,
                         0, 0, 100,
                         1, 1, 1);

  Transformation transformation;
  transformation.setModelCoords(modelCoords,
                                modelGeneral,
                                mainWindowModel);
  transformation.updateTransformFromMatrices(matrixScreen,
                                             matrixGraph);
  return transformation;
}

QImage TestAutoDigitize::mixedShapeSCDImage() const
{
  QImage image(850, 259, QImage::Format_ARGB32);
  image.fill(Qt::white);

  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, false);

  painter.setPen(QPen(Qt::black, 1));
  painter.drawLine(55, 239, 789, 239);
  painter.drawLine(55, 31, 55, 239);
  for (int y = 31; y <= 239; y += 52) {
    painter.drawLine(51, y, 55, y);
  }

  painter.setPen(QPen(QColor(180, 180, 180), 1, Qt::DashLine));
  painter.drawLine(55, 187, 789, 187);
  painter.drawLine(55, 83, 789, 83);

  painter.setPen(QPen(Qt::black, 1, Qt::DashLine));
  painter.drawLine(214, 31, 214, 239);
  painter.drawLine(690, 31, 690, 239);

  painter.setPen(QPen(Qt::black, 1));
  painter.setBrush(Qt::NoBrush);
  painter.drawRect(700, 205, 72, 24);
  painter.drawText(QRect(700, 205, 72, 24),
                   Qt::AlignCenter,
                   "Ben");
  painter.drawText(510, 60, "probe");
  painter.drawLine(552, 62, 582, 92);
  painter.drawLine(582, 92, 575, 89);
  painter.drawLine(582, 92, 579, 84);
  painter.drawLine(795, 231, 802, 247);
  painter.drawLine(807, 231, 814, 247);

  const QList<QPoint> filledCircles = {
    QPoint(88, 205),
    QPoint(144, 182),
    QPoint(256, 160),
    QPoint(424, 137),
    QPoint(584, 110)
  };
  const QList<QPoint> filledSquares = {
    QPoint(116, 155),
    QPoint(228, 130),
    QPoint(340, 170),
    QPoint(480, 100),
    QPoint(620, 125)
  };

  painter.setPen(QPen(Qt::black, 1));
  for (int index = 1; index < filledCircles.count(); ++index) {
    painter.drawLine(filledCircles.at(index - 1),
                     filledCircles.at(index));
  }
  for (int index = 1; index < filledSquares.count(); ++index) {
    painter.drawLine(filledSquares.at(index - 1),
                     filledSquares.at(index));
  }

  painter.setBrush(Qt::black);
  for (int index = 0; index < filledCircles.count(); ++index) {
    painter.drawEllipse(filledCircles.at(index), 8, 8);
  }
  for (int index = 0; index < filledSquares.count(); ++index) {
    const QPoint center = filledSquares.at(index);
    painter.drawRect(center.x() - 6,
                     center.y() - 6,
                     12,
                     12);
  }

  painter.end();
  return image;
}

QImage TestAutoDigitize::triangleSCDImage() const
{
  QImage image(850, 259, QImage::Format_ARGB32);
  image.fill(Qt::white);

  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, false);

  painter.setPen(QPen(Qt::black, 1));
  painter.drawLine(55, 239, 789, 239);
  painter.drawLine(55, 31, 55, 239);
  painter.setPen(QPen(Qt::black, 1, Qt::DashLine));
  painter.drawLine(300, 31, 300, 239);

  const QList<QPoint> openTriangles = {
    QPoint(90, 205),
    QPoint(150, 180),
    QPoint(240, 150),
    QPoint(360, 128),
    QPoint(520, 100)
  };
  const QList<QPoint> filledTriangles = {
    QPoint(120, 120),
    QPoint(260, 90),
    QPoint(420, 150),
    QPoint(610, 76)
  };

  painter.setPen(QPen(Qt::black, 1));
  for (int index = 1; index < openTriangles.count(); ++index) {
    painter.drawLine(openTriangles.at(index - 1),
                     openTriangles.at(index));
  }
  for (int index = 1; index < filledTriangles.count(); ++index) {
    painter.drawLine(filledTriangles.at(index - 1),
                     filledTriangles.at(index));
  }

  painter.setBrush(Qt::NoBrush);
  for (int index = 0; index < openTriangles.count(); ++index) {
    const QPoint center = openTriangles.at(index);
    QPolygon triangle;
    triangle << QPoint(center.x(), center.y() - 7)
             << QPoint(center.x() - 7, center.y() + 7)
             << QPoint(center.x() + 7, center.y() + 7);
    painter.drawPolygon(triangle);
  }

  painter.setBrush(Qt::black);
  for (int index = 0; index < filledTriangles.count(); ++index) {
    const QPoint center = filledTriangles.at(index);
    QPolygon triangle;
    triangle << QPoint(center.x(), center.y() - 7)
             << QPoint(center.x() - 7, center.y() + 7)
             << QPoint(center.x() + 7, center.y() + 7);
    painter.drawPolygon(triangle);
  }

  painter.end();
  return image;
}

void TestAutoDigitize::testSingleCaseArtifactsRejected()
{
  const AutoCurveResult result = AutoDigitize::detectCurvePointGroups(regressionImage(),
                                                                      QRect(QPoint(55, 31), QPoint(789, 239)),
                                                                      regressionTransformation(),
                                                                      0.0,
                                                                      100.0);
  QVERIFY2(result.rawCandidateCount <= 250,
           qPrintable(QString ("raw candidates=%1 message=%2").arg(result.rawCandidateCount).arg(result.message)));
  QVERIFY2(!result.groups.isEmpty(), qPrintable(result.message));

  const QList<QPoint> points = allPointsFromGroups(result.groups);
  QVERIFY2(points.count() <= 20,
           qPrintable(QString ("unexpected point count=%1").arg(points.count())));

  const QRect participantLabelRegion(702, 204, 70, 24);
  for (int index = 0; index < points.count(); ++index) {
    const QPoint point = points.at(index);
    QVERIFY2(!(point.x() >= 210 && point.x() <= 218),
             qPrintable(QString ("point on dotted phase divider %1").arg(pointText(point))));
    QVERIFY2(!(point.x() >= 778 && point.x() <= 780),
             qPrintable(QString ("point on solid phase divider %1").arg(pointText(point))));
    QVERIFY2(!participantLabelRegion.adjusted(-4, -4, 4, 4).contains(point),
             qPrintable(QString ("point in participant label/text box region %1").arg(pointText(point))));
  }
}

void TestAutoDigitize::testSingleCaseOpenAndFilledCirclesDetected()
{
  const AutoCurveResult result = AutoDigitize::detectCurvePointGroups(singleCaseCircleImage(),
                                                                      QRect(QPoint(55, 31), QPoint(789, 239)),
                                                                      singleCaseCircleTransformation(),
                                                                      0.0,
                                                                      100.0);
  QVERIFY2(result.groups.count() >= 2,
           qPrintable(QString ("groups=%1 candidates=%2 message=%3")
                      .arg(result.groups.count())
                      .arg(result.finalCandidateCount)
                      .arg(result.message + " " + groupsText(result.groups))));

  QVERIFY2(result.groups.at(0).points.count() == 10,
           qPrintable(groupsText(result.groups)));
  QVERIFY2(result.groups.at(1).points.count() == 4,
           qPrintable(groupsText(result.groups)));
  QVERIFY2(result.groups.at(0).name.contains("open", Qt::CaseInsensitive),
           qPrintable(QString ("first group name=%1").arg(result.groups.at(0).name)));
  QVERIFY2(result.groups.at(1).name.contains("filled", Qt::CaseInsensitive),
           qPrintable(QString ("second group name=%1").arg(result.groups.at(1).name)));

  const QList<QPoint> openPoints = result.groups.at(0).points;
  const QList<QPoint> filledPoints = result.groups.at(1).points;
  const QList<QPoint> expectedOpen = {
    QPoint(84, 197),
    QPoint(112, 197),
    QPoint(140, 197),
    QPoint(168, 197),
    QPoint(196, 197),
    QPoint(251, 86),
    QPoint(307, 44),
    QPoint(363, 44),
    QPoint(419, 31),
    QPoint(447, 31)
  };
  const QList<QPoint> expectedFilled = {
    QPoint(223, 59),
    QPoint(279, 44),
    QPoint(335, 31),
    QPoint(391, 31)
  };

  for (int index = 0; index < expectedOpen.count(); ++index) {
    QVERIFY2(containsPointNear(openPoints, expectedOpen.at(index), 4),
             qPrintable(QString ("open marker center was not detected near %1")
                        .arg(pointText(expectedOpen.at(index)))));
  }
  for (int index = 0; index < expectedFilled.count(); ++index) {
    QVERIFY2(containsPointNear(filledPoints, expectedFilled.at(index), 4),
             qPrintable(QString ("filled marker center was not detected near %1")
                        .arg(pointText(expectedFilled.at(index)))));
  }

  const QList<QPoint> points = allPointsFromGroups(result.groups);
  const QRect annRegion(706, 207, 70, 23);
  for (int index = 0; index < points.count(); ++index) {
    const QPoint point = points.at(index);
    QVERIFY2(!(point.x() >= 210 && point.x() <= 218),
             qPrintable(QString ("point on baseline/intervention phase divider %1").arg(pointText(point))));
    QVERIFY2(!(point.x() >= 778 && point.x() <= 786),
             qPrintable(QString ("point on right phase divider %1").arg(pointText(point))));
    QVERIFY2(!annRegion.adjusted(-4, -4, 4, 4).contains(point),
             qPrintable(QString ("point in Ann label/text box region %1").arg(pointText(point))));
    const QPoint triangleDelta = point - QPoint(808, 31);
    QVERIFY2(triangleDelta.x() * triangleDelta.x() + triangleDelta.y() * triangleDelta.y() > 12 * 12,
             qPrintable(QString ("singleton triangle was detected as data %1").arg(pointText(point))));
  }

  for (int left = 0; left < points.count(); ++left) {
    for (int right = left + 1; right < points.count(); ++right) {
      const QPoint delta = points.at(left) - points.at(right);
      QVERIFY2(delta.x() * delta.x() + delta.y() * delta.y() >= 36,
               qPrintable(QString ("duplicate marker-sized neighborhood %1 and %2")
                          .arg(pointText(points.at(left)))
                          .arg(pointText(points.at(right)))));
    }
  }
}

void TestAutoDigitize::testSCDSquaresAndFilledCirclesDetected()
{
  const AutoCurveResult result = AutoDigitize::detectCurvePointGroups(mixedShapeSCDImage(),
                                                                      QRect(QPoint(55, 31), QPoint(789, 239)),
                                                                      singleCaseCircleTransformation(),
                                                                      0.0,
                                                                      100.0);
  QVERIFY2(result.groups.count() >= 2,
           qPrintable(QString ("groups=%1 candidates=%2 message=%3")
                      .arg(result.groups.count())
                      .arg(result.finalCandidateCount)
                      .arg(result.message + " " + groupsText(result.groups))));

  const QList<QPoint> expectedSquares = {
    QPoint(116, 155),
    QPoint(228, 130),
    QPoint(340, 170),
    QPoint(480, 100),
    QPoint(620, 125)
  };
  const QList<QPoint> squareGroup = bestMatchingGroupPoints(result.groups,
                                                            expectedSquares,
                                                            5);
  QVERIFY2(matchCount(squareGroup, expectedSquares, 5) == expectedSquares.count(),
           qPrintable(QString ("filled-square group mismatch: %1")
                      .arg(groupsText(result.groups))));
  bool foundSecondDataLikeGroup = false;
  for (int index = 0; index < result.groups.count(); ++index) {
    if (result.groups.at(index).points != squareGroup &&
        result.groups.at(index).points.count() >= 4) {
      foundSecondDataLikeGroup = true;
      break;
    }
  }
  QVERIFY2(foundSecondDataLikeGroup,
           qPrintable(QString ("no second data-like marker group was found: %1")
                      .arg(groupsText(result.groups))));

  const QList<QPoint> points = allPointsFromGroups(result.groups);
  const QRect labelRegion(700, 205, 72, 24);
  for (int index = 0; index < points.count(); ++index) {
    const QPoint point = points.at(index);
    QVERIFY2(!labelRegion.adjusted(-4, -4, 4, 4).contains(point),
             qPrintable(QString ("point in participant label region %1").arg(pointText(point))));
    QVERIFY2(!(point.x() >= 210 && point.x() <= 218),
             qPrintable(QString ("point on phase divider %1").arg(pointText(point))));
    QVERIFY2(!(point.x() >= 686 && point.x() <= 694),
             qPrintable(QString ("point on right phase divider %1").arg(pointText(point))));
    QVERIFY2(!(point.x() >= 548 && point.x() <= 586 && point.y() >= 55 && point.y() <= 96),
             qPrintable(QString ("point on probe arrow/label %1").arg(pointText(point))));
  }
}

void TestAutoDigitize::testSCDOpenAndFilledTrianglesDetected()
{
  const AutoCurveResult result = AutoDigitize::detectCurvePointGroups(triangleSCDImage(),
                                                                      QRect(QPoint(55, 31), QPoint(789, 239)),
                                                                      singleCaseCircleTransformation(),
                                                                      0.0,
                                                                      100.0);
  QVERIFY2(result.groups.count() >= 2,
           qPrintable(QString ("groups=%1 candidates=%2 message=%3")
                      .arg(result.groups.count())
                      .arg(result.finalCandidateCount)
                      .arg(result.message + " " + groupsText(result.groups))));

  const QList<QPoint> expectedOpen = {
    QPoint(90, 205),
    QPoint(150, 180),
    QPoint(240, 150),
    QPoint(360, 128),
    QPoint(520, 100)
  };
  const QList<QPoint> expectedFilled = {
    QPoint(120, 120),
    QPoint(260, 90),
    QPoint(420, 150),
    QPoint(610, 76)
  };
  const QList<QPoint> openGroup = bestMatchingGroupPoints(result.groups,
                                                          expectedOpen,
                                                          6);
  const QList<QPoint> filledGroup = bestMatchingGroupPoints(result.groups,
                                                            expectedFilled,
                                                            6);
  QVERIFY2(matchCount(openGroup, expectedOpen, 6) == expectedOpen.count(),
           qPrintable(QString ("open-triangle group mismatch: %1")
                      .arg(groupsText(result.groups))));
  QVERIFY2(matchCount(filledGroup, expectedFilled, 6) == expectedFilled.count(),
           qPrintable(QString ("filled-triangle group mismatch: %1")
                      .arg(groupsText(result.groups))));
  QVERIFY2(openGroup != filledGroup,
           qPrintable(QString ("open and filled triangles were not separated into cycle groups: %1")
                      .arg(groupsText(result.groups))));

  const QList<QPoint> points = allPointsFromGroups(result.groups);
  for (int index = 0; index < points.count(); ++index) {
    const QPoint point = points.at(index);
    QVERIFY2(!(point.x() >= 296 && point.x() <= 304),
             qPrintable(QString ("point on phase divider %1").arg(pointText(point))));
  }
}

void TestAutoDigitize::testTeachMarkerFindsSimilarMarkers()
{
  const AutoCurveResult openResult = AutoDigitize::detectCurvePointGroupFromExample(singleCaseCircleImage(),
                                                                                    QRect(QPoint(55, 31), QPoint(789, 239)),
                                                                                    singleCaseCircleTransformation(),
                                                                                    0.0,
                                                                                    100.0,
                                                                                    QPoint(112, 197));
  QVERIFY2(openResult.groups.count() == 1,
           qPrintable(openResult.message));
  QVERIFY2(openResult.groups.first().points.count() == 10,
           qPrintable(groupsText(openResult.groups)));
  QVERIFY2(openResult.groups.first().name.contains("open", Qt::CaseInsensitive),
           qPrintable(openResult.groups.first().name));

  const AutoCurveResult filledResult = AutoDigitize::detectCurvePointGroupFromExample(singleCaseCircleImage(),
                                                                                      QRect(QPoint(55, 31), QPoint(789, 239)),
                                                                                      singleCaseCircleTransformation(),
                                                                                      0.0,
                                                                                      100.0,
                                                                                      QPoint(335, 31));
  QVERIFY2(filledResult.groups.count() == 1,
           qPrintable(filledResult.message));
  QVERIFY2(filledResult.groups.first().points.count() == 4,
           qPrintable(groupsText(filledResult.groups)));
  QVERIFY2(filledResult.groups.first().name.contains("filled", Qt::CaseInsensitive),
           qPrintable(filledResult.groups.first().name));
}

void TestAutoDigitize::testAutoAxisStartsAtZero()
{
  const AutoAxisResult result = AutoDigitize::detectAxes(singleCaseCircleImage(),
                                                         100.0);
  QVERIFY2(result.success, qPrintable(result.message));
  QCOMPARE(result.pointsGraph.count(), 3);
  QCOMPARE(result.pointsGraph.at(0), QPointF(0.0, 0.0));
  QCOMPARE(result.pointsGraph.at(2), QPointF(0.0, 100.0));
}

void TestAutoDigitize::testMarkerGroupsAreDistinctAndCentered()
{
  const AutoCurveResult result = AutoDigitize::detectCurvePointGroups(regressionImage(),
                                                                      QRect(QPoint(55, 31), QPoint(789, 239)),
                                                                      regressionTransformation(),
                                                                      0.0,
                                                                      100.0);
  QVERIFY2(result.groups.count() >= 2,
           qPrintable(QString ("groups=%1 message=%2 %3")
                      .arg(result.groups.count())
                      .arg(result.message)
                      .arg(groupsText(result.groups))));

  const QList<QPoint> points = allPointsFromGroups(result.groups);
  QVERIFY2(containsPointNear(points, QPoint(96, 220), 3), "filled marker center was not detected");
  QVERIFY2(containsPointNear(points, QPoint(318, 170), 3), "filled marker center was not detected");
  QVERIFY2(containsPointNear(points, QPoint(250, 181), 4), "open triangle marker center was not detected");
  QVERIFY2(containsPointNear(points, QPoint(560, 132), 4), "open triangle marker center was not detected");

  for (int left = 0; left < points.count(); ++left) {
    for (int right = left + 1; right < points.count(); ++right) {
      const QPoint delta = points.at(left) - points.at(right);
      QVERIFY2(delta.x() * delta.x() + delta.y() * delta.y() >= 36,
               qPrintable(QString ("duplicate marker-sized neighborhood %1 and %2")
                          .arg(pointText(points.at(left)))
                          .arg(pointText(points.at(right)))));
    }
  }
}
