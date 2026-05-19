#ifndef TEST_AUTO_DIGITIZE_H
#define TEST_AUTO_DIGITIZE_H

#include <QObject>
#include <QPoint>
#include <QList>

class QImage;
class Transformation;

/// Regression tests for automatic axis and curve digitizing helpers.
class TestAutoDigitize : public QObject
{
  Q_OBJECT
public:
  explicit TestAutoDigitize(QObject *parent = 0);

private slots:
  void cleanupTestCase ();
  void initTestCase ();

  void testSingleCaseArtifactsRejected ();
  void testMarkerGroupsAreDistinctAndCentered ();
  void testSingleCaseOpenAndFilledCirclesDetected ();
  void testSCDSquaresAndFilledCirclesDetected ();
  void testSCDOpenAndFilledTrianglesDetected ();
  void testTeachMarkerFindsSimilarMarkers ();
  void testAutoAxisStartsAtZero ();

private:
  bool containsPointNear (const QList<QPoint> &points,
                          const QPoint &expected,
                          int tolerance) const;
  QImage regressionImage () const;
  Transformation regressionTransformation () const;
  QImage singleCaseCircleImage () const;
  Transformation singleCaseCircleTransformation () const;
  QImage mixedShapeSCDImage () const;
  QImage triangleSCDImage () const;
};

#endif // TEST_AUTO_DIGITIZE_H
