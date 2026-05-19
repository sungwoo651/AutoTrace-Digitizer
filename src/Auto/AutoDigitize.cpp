/******************************************************************************************************
 * (C) 2026. This file is part of Engauge Digitizer, which is released under GNU General Public       *
 * License version 2 (GPLv2) or (at your option) any later version. See file LICENSE for details.     *
 ******************************************************************************************************/

#include "AutoDigitize.h"
#include "Transformation.h"
#include <QColor>
#include <QImage>
#include <QObject>
#include <QPainter>
#include <QQueue>
#include <QSet>
#include <QTransform>
#include <QVector>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {

const double AXIS_ROTATION_THRESHOLD_DEGREES = 0.7;
const double MAX_AXIS_ROTATION_DEGREES = 8.0;
const double PI = 3.14159265358979323846;
const int CURVE_POINT_MIN_SPACING = 16;
const int AUTO_CURVE_MAX_RAW_CANDIDATES = 250;
const int MARKER_PATCH_SIZE = 32;
const int MARKER_PATCH_FEATURE_SIZE = 16;
const double MARKER_CLUSTER_DISTANCE_THRESHOLD = 0.10;

struct Run
{
  Run() :
    start (0),
    end (0),
    rowOrColumn (0),
    length (0)
  {
  }

  int start;
  int end;
  int rowOrColumn;
  int length;
};

struct Component
{
  Component() :
    area (0),
    sumX (0),
    sumY (0),
    minX (std::numeric_limits<int>::max()),
    maxX (std::numeric_limits<int>::min()),
    minY (std::numeric_limits<int>::max()),
    maxY (std::numeric_limits<int>::min())
  {
  }

  int area;
  long long sumX;
  long long sumY;
  int minX;
  int maxX;
  int minY;
  int maxY;
};

struct MarkerCandidate
{
  MarkerCandidate() :
    area (0),
    size (0.0),
    score (0.0),
    coreDensity (0.0),
    shapeKind (0)
  {
  }

  QPoint center;
  QRect bounds;
  int area;
  double size;
  double score;
  double coreDensity;
  int shapeKind;
  QVector<unsigned char> patch;
  QVector<double> features;
};

struct MarkerCluster
{
  QList<int> candidateIndexes;
  QVector<double> centroid;
  double meanDistance;
  double sizeVariation;
};

QPoint clampPointToYRange(const QPointF &pointScreen,
                          const Transformation &transformation,
                          double yMinimum,
                          double yMaximum);

void addCurvePointIfSeparated(QList<QPoint> &points,
                              const QPoint &point);

inline int indexFor(int x,
                    int y,
                    int width)
{
  return y * width + x;
}

bool pixelIsForeground(QRgb rgb)
{
  if (qAlpha(rgb) < 20) {
    return false;
  }

  const int red = qRed(rgb);
  const int green = qGreen(rgb);
  const int blue = qBlue(rgb);
  const int maximum = qMax(red, qMax(green, blue));
  const int minimum = qMin(red, qMin(green, blue));
  const int gray = qGray(rgb);

  return (gray < 190) || ((maximum - minimum) > 45 && maximum < 248);
}

QVector<unsigned char> foregroundMask(const QImage &image)
{
  const QImage converted = image.convertToFormat(QImage::Format_ARGB32);
  const int width = converted.width();
  const int height = converted.height();
  QVector<unsigned char> mask(width * height, 0);

  for (int y = 0; y < height; ++y) {
    const QRgb *line = reinterpret_cast<const QRgb *> (converted.constScanLine(y));
    for (int x = 0; x < width; ++x) {
      if (pixelIsForeground(line [x])) {
        mask [indexFor(x, y, width)] = 1;
      }
    }
  }

  return mask;
}

Run longestHorizontalRun(const QVector<unsigned char> &mask,
                         int width,
                         int row,
                         int gapAllowed)
{
  Run best;
  best.rowOrColumn = row;

  int start = -1;
  int lastForeground = -1;
  int gap = 0;

  for (int x = 0; x < width; ++x) {
    if (mask [indexFor(x, row, width)]) {
      if (start < 0) {
        start = x;
      }
      lastForeground = x;
      gap = 0;
    } else if (start >= 0) {
      ++gap;
      if (gap > gapAllowed) {
        const int length = lastForeground - start + 1;
        if (length > best.length) {
          best.start = start;
          best.end = lastForeground;
          best.length = length;
        }
        start = -1;
        lastForeground = -1;
        gap = 0;
      }
    }
  }

  if (start >= 0) {
    const int length = lastForeground - start + 1;
    if (length > best.length) {
      best.start = start;
      best.end = lastForeground;
      best.length = length;
    }
  }

  return best;
}

Run longestVerticalRun(const QVector<unsigned char> &mask,
                       int width,
                       int height,
                       int column,
                       int yMaximum,
                       int gapAllowed)
{
  Run best;
  best.rowOrColumn = column;

  int start = -1;
  int lastForeground = -1;
  int gap = 0;
  const int yLimit = qBound(0, yMaximum, height - 1);

  for (int y = 0; y <= yLimit; ++y) {
    if (mask [indexFor(column, y, width)]) {
      if (start < 0) {
        start = y;
      }
      lastForeground = y;
      gap = 0;
    } else if (start >= 0) {
      ++gap;
      if (gap > gapAllowed) {
        const int length = lastForeground - start + 1;
        if (length > best.length) {
          best.start = start;
          best.end = lastForeground;
          best.length = length;
        }
        start = -1;
        lastForeground = -1;
        gap = 0;
      }
    }
  }

  if (start >= 0) {
    const int length = lastForeground - start + 1;
    if (length > best.length) {
      best.start = start;
      best.end = lastForeground;
      best.length = length;
    }
  }

  return best;
}

double estimateHorizontalAngleDegrees(const QImage &image)
{
  const QImage converted = image.convertToFormat(QImage::Format_ARGB32);
  const int width = converted.width();
  const int height = converted.height();
  const int step = qMax(1, qMax(width, height) / 800);
  QList<QPoint> points;

  for (int y = 0; y < height; y += step) {
    const QRgb *line = reinterpret_cast<const QRgb *> (converted.constScanLine(y));
    for (int x = 0; x < width; x += step) {
      if (pixelIsForeground(line [x])) {
        points << QPoint(x, y);
      }
    }
  }

  if (points.count() < 50) {
    return 0.0;
  }

  double bestAngle = 0.0;
  double bestScore = -1.0;

  for (int angleHalfDegrees = -16; angleHalfDegrees <= 16; ++angleHalfDegrees) {
    const double angleDegrees = angleHalfDegrees * 0.5;
    const double slope = std::tan(angleDegrees * PI / 180.0);
    const int binMin = static_cast<int> (std::floor(-std::abs(slope) * width)) - 4;
    const int binMax = height + static_cast<int> (std::ceil(std::abs(slope) * width)) + 4;
    const int binCount = binMax - binMin + 1;
    QVector<int> bins(binCount, 0);

    for (int index = 0; index < points.count(); ++index) {
      const QPoint point = points.at(index);
      const int bin = qRound((point.y() - slope * point.x()) / 2.0) - binMin;
      if (bin >= 0 && bin < binCount) {
        ++bins [bin];
      }
    }

    for (int bin = 0; bin < binCount; ++bin) {
      const double yIntercept = (bin + binMin) * 2.0;
      const double yCenter = yIntercept + slope * width / 2.0;
      const double bottomPreference = qBound(0.0, yCenter / qMax(1, height), 1.0);
      const double score = bins [bin] + bottomPreference * 3.0;
      if (score > bestScore) {
        bestScore = score;
        bestAngle = angleDegrees;
      }
    }
  }

  return bestAngle;
}

QImage rotateImage(const QImage &image,
                   double degrees)
{
  QTransform transform;
  transform.rotate(degrees);
  QRect rotatedRect = transform.mapRect(QRect(QPoint(0, 0), image.size()));
  QImage rotated(rotatedRect.size(), QImage::Format_ARGB32);
  rotated.fill(Qt::white);

  QPainter painter(&rotated);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
  painter.translate(-rotatedRect.topLeft());
  painter.setTransform(transform, true);
  painter.drawImage(0, 0, image.convertToFormat(QImage::Format_ARGB32));
  painter.end();

  return rotated;
}

bool detectHorizontalAxis(const QVector<unsigned char> &mask,
                          int width,
                          int height,
                          Run &axisRun)
{
  Run best;
  const int minimumRunLength = qMax(40, width / 5);

  for (int y = 0; y < height; ++y) {
    const Run run = longestHorizontalRun(mask, width, y, 4);
    if (run.length < minimumRunLength) {
      continue;
    }

    if (run.length > best.length ||
        (run.length >= best.length * 8 / 10 && y > best.rowOrColumn)) {
      best = run;
    }
  }

  if (best.length < minimumRunLength) {
    return false;
  }

  axisRun = best;
  return true;
}

bool detectVerticalAxis(const QVector<unsigned char> &mask,
                        int width,
                        int height,
                        const Run &xAxisRun,
                        Run &axisRun)
{
  Run best;
  double bestScore = -1.0;
  const int yMaximum = xAxisRun.rowOrColumn;
  const int minimumRunLength = qMax(30, height / 5);
  const int columnMaximum = qMin(width - 1, qMax(xAxisRun.start + width / 8, width / 2));

  for (int x = 0; x <= columnMaximum; ++x) {
    const Run run = longestVerticalRun(mask, width, height, x, yMaximum, 4);
    if (run.length < minimumRunLength) {
      continue;
    }

    const int bottomDistance = std::abs(run.end - yMaximum);
    const int leftDistance = std::abs(x - xAxisRun.start);
    const double score = run.length - bottomDistance * 0.7 - leftDistance * 0.08;
    if (score > bestScore) {
      best = run;
      bestScore = score;
    }
  }

  if (bestScore < 0.0) {
    return false;
  }

  axisRun = best;
  return true;
}

void addComponentPixel(Component &component,
                       int x,
                       int y)
{
  ++component.area;
  component.sumX += x;
  component.sumY += y;
  component.minX = qMin(component.minX, x);
  component.maxX = qMax(component.maxX, x);
  component.minY = qMin(component.minY, y);
  component.maxY = qMax(component.maxY, y);
}

QVector<unsigned char> denseForegroundMask(const QVector<unsigned char> &mask,
                                           int width,
                                           int height,
                                           const QRect &rect)
{
  QVector<unsigned char> dense(width * height, 0);
  const QRect bounds = rect.intersected(QRect(0, 0, width, height));

  for (int y = bounds.top(); y <= bounds.bottom(); ++y) {
    for (int x = bounds.left(); x <= bounds.right(); ++x) {
      if (!mask [indexFor(x, y, width)]) {
        continue;
      }

      int count = 0;
      for (int yy = qMax(bounds.top(), y - 2); yy <= qMin(bounds.bottom(), y + 2); ++yy) {
        for (int xx = qMax(bounds.left(), x - 2); xx <= qMin(bounds.right(), x + 2); ++xx) {
          count += mask [indexFor(xx, yy, width)];
        }
      }

      if (count >= 3) {
        dense [indexFor(x, y, width)] = 1;
      }
    }
  }

  return dense;
}

QVector<unsigned char> suppressLongRuns(const QVector<unsigned char> &mask,
                                        int width,
                                        int height,
                                        const QRect &rect)
{
  QVector<unsigned char> filtered = mask;
  const QRect bounds = rect.intersected(QRect(0, 0, width, height));
  const int horizontalMinimum = qMax(24, bounds.width() / 5);
  const int verticalMinimum = qMax(24, bounds.height() / 5);

  for (int y = bounds.top(); y <= bounds.bottom(); ++y) {
    int runStart = -1;
    for (int x = bounds.left(); x <= bounds.right() + 1; ++x) {
      const bool foreground = (x <= bounds.right()) && mask [indexFor(x, y, width)];
      if (foreground && runStart < 0) {
        runStart = x;
      } else if ((!foreground || x > bounds.right()) && runStart >= 0) {
        const int runEnd = x - 1;
        if (runEnd - runStart + 1 >= horizontalMinimum) {
          for (int xx = runStart; xx <= runEnd; ++xx) {
            filtered [indexFor(xx, y, width)] = 0;
          }
        }
        runStart = -1;
      }
    }
  }

  for (int x = bounds.left(); x <= bounds.right(); ++x) {
    int runStart = -1;
    for (int y = bounds.top(); y <= bounds.bottom() + 1; ++y) {
      const bool foreground = (y <= bounds.bottom()) && mask [indexFor(x, y, width)];
      if (foreground && runStart < 0) {
        runStart = y;
      } else if ((!foreground || y > bounds.bottom()) && runStart >= 0) {
        const int runEnd = y - 1;
        if (runEnd - runStart + 1 >= verticalMinimum) {
          for (int yy = runStart; yy <= runEnd; ++yy) {
            filtered [indexFor(x, yy, width)] = 0;
          }
        }
        runStart = -1;
      }
    }
  }

  return filtered;
}

QVector<unsigned char> suppressColumnAndRowArtifacts(const QVector<unsigned char> &mask,
                                                     int width,
                                                     int height,
                                                     const QRect &rect,
                                                     int *rejectedLineLikeCount)
{
  QVector<unsigned char> filtered = mask;
  const QRect bounds = rect.intersected(QRect(0, 0, width, height));
  const int verticalCountMinimum = qMax(10, bounds.height() / 12);
  const int horizontalCountMinimum = qMax(10, bounds.width() / 12);
  const int verticalSpanMinimum = qMax(24, bounds.height() / 3);
  const int horizontalSpanMinimum = qMax(24, bounds.width() / 3);

  for (int x = bounds.left(); x <= bounds.right(); ++x) {
    int count = 0;
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::min();

    for (int y = bounds.top(); y <= bounds.bottom(); ++y) {
      if (mask [indexFor(x, y, width)]) {
        ++count;
        minY = qMin(minY, y);
        maxY = qMax(maxY, y);
      }
    }

    if (count >= verticalCountMinimum &&
        maxY - minY + 1 >= verticalSpanMinimum) {
      for (int xx = qMax(bounds.left(), x - 3); xx <= qMin(bounds.right(), x + 3); ++xx) {
        for (int y = bounds.top(); y <= bounds.bottom(); ++y) {
          filtered [indexFor(xx, y, width)] = 0;
        }
      }
      if (rejectedLineLikeCount != nullptr) {
        ++(*rejectedLineLikeCount);
      }
    }
  }

  for (int y = bounds.top(); y <= bounds.bottom(); ++y) {
    int count = 0;
    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();

    for (int x = bounds.left(); x <= bounds.right(); ++x) {
      if (mask [indexFor(x, y, width)]) {
        ++count;
        minX = qMin(minX, x);
        maxX = qMax(maxX, x);
      }
    }

    if (count >= horizontalCountMinimum &&
        maxX - minX + 1 >= horizontalSpanMinimum) {
      for (int yy = qMax(bounds.top(), y - 3); yy <= qMin(bounds.bottom(), y + 3); ++yy) {
        for (int x = bounds.left(); x <= bounds.right(); ++x) {
          filtered [indexFor(x, yy, width)] = 0;
        }
      }
      if (rejectedLineLikeCount != nullptr) {
        ++(*rejectedLineLikeCount);
      }
    }
  }

  return filtered;
}

QVector<Component> connectedComponents(const QVector<unsigned char> &mask,
                                       int width,
                                       int height,
                                       const QRect &bounds)
{
  QVector<Component> components;
  QVector<unsigned char> visited(width * height, 0);

  for (int y = bounds.top(); y <= bounds.bottom(); ++y) {
    for (int x = bounds.left(); x <= bounds.right(); ++x) {
      const int startIndex = indexFor(x, y, width);
      if (!mask [startIndex] || visited [startIndex]) {
        continue;
      }

      Component component;
      QQueue<QPoint> queue;
      queue.enqueue(QPoint(x, y));
      visited [startIndex] = 1;

      while (!queue.isEmpty()) {
        const QPoint point = queue.dequeue();
        addComponentPixel(component, point.x(), point.y());

        for (int yy = point.y() - 1; yy <= point.y() + 1; ++yy) {
          for (int xx = point.x() - 1; xx <= point.x() + 1; ++xx) {
            if (xx < bounds.left() || xx > bounds.right() ||
                yy < bounds.top() || yy > bounds.bottom()) {
              continue;
            }

            const int neighborIndex = indexFor(xx, yy, width);
            if (!mask [neighborIndex] || visited [neighborIndex]) {
              continue;
            }

            visited [neighborIndex] = 1;
            queue.enqueue(QPoint(xx, yy));
          }
        }
      }

      components << component;
    }
  }

  return components;
}

QRect componentRect(const Component &component)
{
  return QRect(QPoint(component.minX, component.minY),
               QPoint(component.maxX, component.maxY));
}

bool componentsShouldMerge(const Component &left,
                           const Component &right)
{
  const QRect leftExpanded = componentRect(left).adjusted(-5, -5, 5, 5);
  const QRect rightRect = componentRect(right);

  if (leftExpanded.intersects(rightRect)) {
    return true;
  }

  const QPoint leftCenter = componentRect(left).center();
  const QPoint rightCenter = rightRect.center();
  const int dx = leftCenter.x() - rightCenter.x();
  const int dy = leftCenter.y() - rightCenter.y();
  const int maxSize = qMax(qMax(componentRect(left).width(), componentRect(left).height()),
                           qMax(rightRect.width(), rightRect.height()));

  return dx * dx + dy * dy <= qMax(36, maxSize * maxSize);
}

void mergeComponent(Component &target,
                    const Component &source)
{
  target.area += source.area;
  target.sumX += source.sumX;
  target.sumY += source.sumY;
  target.minX = qMin(target.minX, source.minX);
  target.maxX = qMax(target.maxX, source.maxX);
  target.minY = qMin(target.minY, source.minY);
  target.maxY = qMax(target.maxY, source.maxY);
}

QVector<Component> mergeNearbyComponents(const QVector<Component> &components)
{
  QVector<Component> merged = components;
  bool changed = true;

  while (changed) {
    changed = false;
    for (int left = 0; left < merged.count() && !changed; ++left) {
      for (int right = left + 1; right < merged.count(); ++right) {
        if (!componentsShouldMerge(merged.at(left), merged.at(right))) {
          continue;
        }

        mergeComponent(merged [left], merged.at(right));
        merged.remove(right);
        changed = true;
        break;
      }
    }
  }

  return merged;
}

bool componentIsLineLike(const Component &component,
                         const QRect &bounds)
{
  const int componentWidth = component.maxX - component.minX + 1;
  const int componentHeight = component.maxY - component.minY + 1;
  const int longSide = qMax(componentWidth, componentHeight);
  const int shortSide = qMax(1, qMin(componentWidth, componentHeight));
  const double aspect = static_cast<double> (longSide) / shortSide;

  if (componentHeight >= qMax(24, bounds.height() / 5) && componentWidth <= qMax(5, bounds.width() / 80)) {
    return true;
  }

  if (componentWidth >= qMax(24, bounds.width() / 5) && componentHeight <= qMax(5, bounds.height() / 80)) {
    return true;
  }

  if (aspect > 3.0 && longSide >= 18) {
    return true;
  }

  if (componentHeight >= bounds.height() / 4 || componentWidth >= bounds.width() / 4) {
    return true;
  }

  return false;
}

bool componentIsMarkerLike(const Component &component,
                           const QRect &bounds)
{
  const int componentWidth = component.maxX - component.minX + 1;
  const int componentHeight = component.maxY - component.minY + 1;
  const int longSide = qMax(componentWidth, componentHeight);
  const int shortSide = qMax(1, qMin(componentWidth, componentHeight));
  const double density = static_cast<double> (component.area) /
                         qMax(1, componentWidth * componentHeight);
  const int maxMarkerWidth = qMax(16, qMin(52, bounds.width() / 10));
  const int maxMarkerHeight = qMax(16, qMin(52, bounds.height() / 4));

  if (component.area < 6 ||
      component.area > 1800 ||
      componentWidth < 3 ||
      componentHeight < 3 ||
      componentWidth > maxMarkerWidth ||
      componentHeight > maxMarkerHeight) {
    return false;
  }

  if (static_cast<double> (longSide) / shortSide > 2.7) {
    return false;
  }

  if (density < 0.05 || density > 0.95) {
    return false;
  }

  return true;
}

QVector<unsigned char> extractNormalizedPatch(const QVector<unsigned char> &mask,
                                              int width,
                                              int height,
                                              const Component &component)
{
  QVector<unsigned char> patch(MARKER_PATCH_SIZE * MARKER_PATCH_SIZE, 0);
  const double centerX = static_cast<double> (component.sumX) / component.area;
  const double centerY = static_cast<double> (component.sumY) / component.area;
  const int componentWidth = component.maxX - component.minX + 1;
  const int componentHeight = component.maxY - component.minY + 1;
  const double side = qMax(8.0,
                           static_cast<double> (qMax(componentWidth, componentHeight) + 6));

  for (int py = 0; py < MARKER_PATCH_SIZE; ++py) {
    const double sourceY = centerY + ((static_cast<double> (py) + 0.5) / MARKER_PATCH_SIZE - 0.5) * side;
    const int y = qRound(sourceY);
    if (y < 0 || y >= height) {
      continue;
    }

    for (int px = 0; px < MARKER_PATCH_SIZE; ++px) {
      const double sourceX = centerX + ((static_cast<double> (px) + 0.5) / MARKER_PATCH_SIZE - 0.5) * side;
      const int x = qRound(sourceX);
      if (x < 0 || x >= width) {
        continue;
      }

      if (mask [indexFor(x, y, width)]) {
        patch [indexFor(px, py, MARKER_PATCH_SIZE)] = 1;
      }
    }
  }

  return patch;
}

QVector<unsigned char> extractNormalizedPatchAroundPoint(const QVector<unsigned char> &mask,
                                                         int width,
                                                         int height,
                                                         const QPointF &center,
                                                         double side)
{
  QVector<unsigned char> patch(MARKER_PATCH_SIZE * MARKER_PATCH_SIZE, 0);
  const double normalizedSide = qMax(8.0, side);

  for (int py = 0; py < MARKER_PATCH_SIZE; ++py) {
    const double sourceY = center.y() + ((static_cast<double> (py) + 0.5) / MARKER_PATCH_SIZE - 0.5) * normalizedSide;
    const int y = qRound(sourceY);
    if (y < 0 || y >= height) {
      continue;
    }

    for (int px = 0; px < MARKER_PATCH_SIZE; ++px) {
      const double sourceX = center.x() + ((static_cast<double> (px) + 0.5) / MARKER_PATCH_SIZE - 0.5) * normalizedSide;
      const int x = qRound(sourceX);
      if (x < 0 || x >= width) {
        continue;
      }

      if (mask [indexFor(x, y, width)]) {
        patch [indexFor(px, py, MARKER_PATCH_SIZE)] = 1;
      }
    }
  }

  return patch;
}

double localCoreDensity(const QVector<unsigned char> &mask,
                        int width,
                        int height,
                        const QPoint &center)
{
  int foreground = 0;
  int pixels = 0;
  const int radius = 3;
  for (int y = qMax(0, center.y() - radius); y <= qMin(height - 1, center.y() + radius); ++y) {
    for (int x = qMax(0, center.x() - radius); x <= qMin(width - 1, center.x() + radius); ++x) {
      const int dx = x - center.x();
      const int dy = y - center.y();
      if (dx * dx + dy * dy > radius * radius) {
        continue;
      }
      foreground += mask [indexFor(x, y, width)];
      ++pixels;
    }
  }

  return static_cast<double> (foreground) / qMax(1, pixels);
}

double patchForegroundRatio(const QVector<unsigned char> &patch)
{
  int foreground = 0;
  for (int index = 0; index < patch.count(); ++index) {
    foreground += patch.at(index);
  }

  return static_cast<double> (foreground) / qMax(1, patch.count());
}

double patchHoleRatio(const QVector<unsigned char> &patch)
{
  QVector<unsigned char> visited(patch.count(), 0);
  QQueue<QPoint> queue;

  for (int x = 0; x < MARKER_PATCH_SIZE; ++x) {
    if (!patch [indexFor(x, 0, MARKER_PATCH_SIZE)]) {
      queue.enqueue(QPoint(x, 0));
      visited [indexFor(x, 0, MARKER_PATCH_SIZE)] = 1;
    }
    if (!patch [indexFor(x, MARKER_PATCH_SIZE - 1, MARKER_PATCH_SIZE)]) {
      queue.enqueue(QPoint(x, MARKER_PATCH_SIZE - 1));
      visited [indexFor(x, MARKER_PATCH_SIZE - 1, MARKER_PATCH_SIZE)] = 1;
    }
  }

  for (int y = 0; y < MARKER_PATCH_SIZE; ++y) {
    if (!patch [indexFor(0, y, MARKER_PATCH_SIZE)] &&
        !visited [indexFor(0, y, MARKER_PATCH_SIZE)]) {
      queue.enqueue(QPoint(0, y));
      visited [indexFor(0, y, MARKER_PATCH_SIZE)] = 1;
    }
    if (!patch [indexFor(MARKER_PATCH_SIZE - 1, y, MARKER_PATCH_SIZE)] &&
        !visited [indexFor(MARKER_PATCH_SIZE - 1, y, MARKER_PATCH_SIZE)]) {
      queue.enqueue(QPoint(MARKER_PATCH_SIZE - 1, y));
      visited [indexFor(MARKER_PATCH_SIZE - 1, y, MARKER_PATCH_SIZE)] = 1;
    }
  }

  while (!queue.isEmpty()) {
    const QPoint point = queue.dequeue();
    const QPoint neighbors [4] = {
      QPoint(point.x() - 1, point.y()),
      QPoint(point.x() + 1, point.y()),
      QPoint(point.x(), point.y() - 1),
      QPoint(point.x(), point.y() + 1)
    };

    for (int index = 0; index < 4; ++index) {
      const QPoint neighbor = neighbors [index];
      if (neighbor.x() < 0 || neighbor.x() >= MARKER_PATCH_SIZE ||
          neighbor.y() < 0 || neighbor.y() >= MARKER_PATCH_SIZE) {
        continue;
      }

      const int neighborIndex = indexFor(neighbor.x(), neighbor.y(), MARKER_PATCH_SIZE);
      if (patch [neighborIndex] || visited [neighborIndex]) {
        continue;
      }

      visited [neighborIndex] = 1;
      queue.enqueue(neighbor);
    }
  }

  int interiorBackground = 0;
  for (int index = 0; index < patch.count(); ++index) {
    if (!patch.at(index) && !visited.at(index)) {
      ++interiorBackground;
    }
  }

  return static_cast<double> (interiorBackground) / patch.count();
}

double patchCenterForegroundRatio(const QVector<unsigned char> &patch)
{
  int foreground = 0;
  int pixels = 0;
  const int minimum = MARKER_PATCH_SIZE / 2 - 4;
  const int maximum = MARKER_PATCH_SIZE / 2 + 3;
  for (int y = minimum; y <= maximum; ++y) {
    for (int x = minimum; x <= maximum; ++x) {
      foreground += patch [indexFor(x, y, MARKER_PATCH_SIZE)];
      ++pixels;
    }
  }

  return static_cast<double> (foreground) / qMax(1, pixels);
}

double patchCoreForegroundRatio(const QVector<unsigned char> &patch)
{
  int foreground = 0;
  int pixels = 0;
  const int minimum = MARKER_PATCH_SIZE / 2 - 2;
  const int maximum = MARKER_PATCH_SIZE / 2 + 1;
  for (int y = minimum; y <= maximum; ++y) {
    for (int x = minimum; x <= maximum; ++x) {
      foreground += patch [indexFor(x, y, MARKER_PATCH_SIZE)];
      ++pixels;
    }
  }

  return static_cast<double> (foreground) / qMax(1, pixels);
}

double patchCornerForegroundRatio(const QVector<unsigned char> &patch)
{
  int foreground = 0;
  int pixels = 0;
  const double center = (MARKER_PATCH_SIZE - 1) / 2.0;
  for (int y = 0; y < MARKER_PATCH_SIZE; ++y) {
    for (int x = 0; x < MARKER_PATCH_SIZE; ++x) {
      const double dx = std::abs(x - center) / center;
      const double dy = std::abs(y - center) / center;
      if (dx < 0.24 || dy < 0.24 || dx * dx + dy * dy < 0.18) {
        continue;
      }
      foreground += patch [indexFor(x, y, MARKER_PATCH_SIZE)];
      ++pixels;
    }
  }

  return static_cast<double> (foreground) / qMax(1, pixels);
}

double patchEdgeDensity(const QVector<unsigned char> &patch)
{
  int transitions = 0;
  int comparisons = 0;
  for (int y = 0; y < MARKER_PATCH_SIZE; ++y) {
    for (int x = 0; x < MARKER_PATCH_SIZE; ++x) {
      const unsigned char value = patch [indexFor(x, y, MARKER_PATCH_SIZE)];
      if (x + 1 < MARKER_PATCH_SIZE) {
        transitions += value != patch [indexFor(x + 1, y, MARKER_PATCH_SIZE)];
        ++comparisons;
      }
      if (y + 1 < MARKER_PATCH_SIZE) {
        transitions += value != patch [indexFor(x, y + 1, MARKER_PATCH_SIZE)];
        ++comparisons;
      }
    }
  }

  return static_cast<double> (transitions) / qMax(1, comparisons);
}

QVector<double> markerFeatures(const MarkerCandidate &candidate)
{
  QVector<double> features;
  const double foregroundRatio = patchForegroundRatio(candidate.patch);
  const double holeRatio = patchHoleRatio(candidate.patch);
  const double edgeDensity = patchEdgeDensity(candidate.patch);
  const double coreRatio = patchCoreForegroundRatio(candidate.patch);
  const double cornerRatio = patchCornerForegroundRatio(candidate.patch);
  const double density = static_cast<double> (candidate.area) /
                         qMax(1, candidate.bounds.width() * candidate.bounds.height());
  const double aspect = static_cast<double> (qMin(candidate.bounds.width(), candidate.bounds.height())) /
                        qMax(1, qMax(candidate.bounds.width(), candidate.bounds.height()));

  double sumX = 0.0;
  double sumY = 0.0;
  double count = 0.0;
  for (int y = 0; y < MARKER_PATCH_SIZE; ++y) {
    for (int x = 0; x < MARKER_PATCH_SIZE; ++x) {
      if (candidate.patch [indexFor(x, y, MARKER_PATCH_SIZE)]) {
        sumX += x;
        sumY += y;
        count += 1.0;
      }
    }
  }

  const double centerX = count > 0.0 ? sumX / count : MARKER_PATCH_SIZE / 2.0;
  const double centerY = count > 0.0 ? sumY / count : MARKER_PATCH_SIZE / 2.0;
  double varX = 0.0;
  double varY = 0.0;
  double covariance = 0.0;
  QVector<double> radial(4, 0.0);
  QVector<double> radialCounts(4, 0.0);
  for (int y = 0; y < MARKER_PATCH_SIZE; ++y) {
    for (int x = 0; x < MARKER_PATCH_SIZE; ++x) {
      const double dx = x - centerX;
      const double dy = y - centerY;
      const double radius = std::sqrt(dx * dx + dy * dy);
      const int radialIndex = qBound(0,
                                     static_cast<int> (radius / (MARKER_PATCH_SIZE / 8.0)),
                                     radial.count() - 1);
      radialCounts [radialIndex] += 1.0;
      if (candidate.patch [indexFor(x, y, MARKER_PATCH_SIZE)]) {
        varX += dx * dx;
        varY += dy * dy;
        covariance += dx * dy;
        radial [radialIndex] += 1.0;
      }
    }
  }

  const double varianceScale = MARKER_PATCH_SIZE * MARKER_PATCH_SIZE;
  features << foregroundRatio * 1.4
           << holeRatio * 2.2
           << edgeDensity
           << coreRatio * 1.6
           << candidate.coreDensity * 1.8
           << cornerRatio * 2.0
           << density
           << aspect
           << (candidate.bounds.width() / qMax(1.0, candidate.size)) * 0.35
           << (candidate.bounds.height() / qMax(1.0, candidate.size)) * 0.35
           << ((centerX / MARKER_PATCH_SIZE) - 0.5)
           << ((centerY / MARKER_PATCH_SIZE) - 0.5)
           << (count > 0.0 ? varX / count / varianceScale : 0.0)
           << (count > 0.0 ? varY / count / varianceScale : 0.0)
           << (count > 0.0 ? covariance / count / varianceScale : 0.0);

  for (int index = 0; index < radial.count(); ++index) {
    features << (radialCounts.at(index) > 0.0 ? radial.at(index) / radialCounts.at(index) : 0.0);
  }

  const int cellSize = MARKER_PATCH_SIZE / MARKER_PATCH_FEATURE_SIZE;
  for (int cellY = 0; cellY < MARKER_PATCH_FEATURE_SIZE; ++cellY) {
    for (int cellX = 0; cellX < MARKER_PATCH_FEATURE_SIZE; ++cellX) {
      int foreground = 0;
      for (int yy = cellY * cellSize; yy < (cellY + 1) * cellSize; ++yy) {
        for (int xx = cellX * cellSize; xx < (cellX + 1) * cellSize; ++xx) {
          foreground += candidate.patch [indexFor(xx, yy, MARKER_PATCH_SIZE)];
        }
      }
      features << 0.45 * foreground / qMax(1, cellSize * cellSize);
    }
  }

  return features;
}

double featureDistance(const QVector<double> &left,
                       const QVector<double> &right)
{
  const int count = qMin(left.count(), right.count());
  if (count == 0) {
    return std::numeric_limits<double>::max();
  }

  double sum = 0.0;
  double weightTotal = 0.0;
  for (int index = 0; index < count; ++index) {
    const double delta = left.at(index) - right.at(index);
    const double weight = index < 16 ? 5.0 : 1.0;
    sum += delta * delta * weight;
    weightTotal += weight;
  }

  return std::sqrt(sum / qMax(1.0, weightTotal));
}

QVector<MarkerCandidate> markerCandidatesFromComponents(const QVector<unsigned char> &mask,
                                                        int width,
                                                        int height,
                                                        const QRect &bounds,
                                                        const Transformation &transformation,
                                                        double yMinimum,
                                                        double yMaximum,
                                                        int *rawCandidateCount,
                                                        int *rejectedLineLikeCount)
{
  QVector<MarkerCandidate> candidates;
  const QVector<Component> components = connectedComponents(mask,
                                                            width,
                                                            height,
                                                            bounds);
  if (rawCandidateCount != nullptr) {
    *rawCandidateCount = components.count();
  }

  if (components.count() > AUTO_CURVE_MAX_RAW_CANDIDATES) {
    return candidates;
  }

  const QVector<Component> mergedComponents = mergeNearbyComponents(components);
  for (int componentIndex = 0; componentIndex < mergedComponents.count(); ++componentIndex) {
    const Component &component = mergedComponents.at(componentIndex);

    if (componentIsLineLike(component, bounds)) {
      if (rejectedLineLikeCount != nullptr) {
        ++(*rejectedLineLikeCount);
      }
      continue;
    }

    if (!componentIsMarkerLike(component, bounds)) {
      continue;
    }

    const int componentWidth = component.maxX - component.minX + 1;
    const int componentHeight = component.maxY - component.minY + 1;
    const QPointF centerRaw((component.minX + component.maxX) / 2.0,
                            (component.minY + component.maxY) / 2.0);
    const QPoint center = clampPointToYRange(centerRaw,
                                             transformation,
                                             yMinimum,
                                             yMaximum);
    if (!bounds.contains(center)) {
      continue;
    }

    MarkerCandidate candidate;
    candidate.center = center;
    candidate.bounds = QRect(QPoint(component.minX, component.minY),
                             QPoint(component.maxX, component.maxY));
    candidate.area = component.area;
    candidate.size = qMax(componentWidth, componentHeight);
    candidate.coreDensity = localCoreDensity(mask,
                                             width,
                                             height,
                                             center);
    candidate.patch = extractNormalizedPatch(mask,
                                             width,
                                             height,
                                             component);
    candidate.features = markerFeatures(candidate);
    candidate.score = 0.55 + qMin(0.35, patchEdgeDensity(candidate.patch) * 2.0);
    candidates << candidate;
  }

  return candidates;
}

double localMarkerScore(const QVector<unsigned char> &mask,
                        int width,
                        int height,
                        const QRect &bounds,
                        const QPoint &center,
                        int radius,
                        QPointF &refinedCenter,
                        QRect &candidateBounds,
                        int &area)
{
  Q_UNUSED(bounds);
  const int left = qMax(0, center.x() - radius);
  const int right = qMin(width - 1, center.x() + radius);
  const int top = qMax(0, center.y() - radius);
  const int bottom = qMin(height - 1, center.y() + radius);
  QVector<int> sectors(8, 0);
  QVector<int> quadrants(4, 0);
  int ringForeground = 0;
  int ringPixels = 0;
  int centerForeground = 0;
  int centerPixels = 0;
  int minX = std::numeric_limits<int>::max();
  int maxX = std::numeric_limits<int>::min();
  int minY = std::numeric_limits<int>::max();
  int maxY = std::numeric_limits<int>::min();
  area = 0;

  const double radiusSquared = radius * radius;
  const double ringInnerSquared = radiusSquared * 0.22;
  const double centerSquared = radiusSquared * 0.16;

  for (int y = top; y <= bottom; ++y) {
    for (int x = left; x <= right; ++x) {
      const int dx = x - center.x();
      const int dy = y - center.y();
      const double distanceSquared = dx * dx + dy * dy;
      if (distanceSquared > radiusSquared) {
        continue;
      }

      if (distanceSquared <= centerSquared) {
        ++centerPixels;
      }
      if (distanceSquared >= ringInnerSquared) {
        ++ringPixels;
      }

      if (!mask [indexFor(x, y, width)]) {
        continue;
      }

      ++area;
      minX = qMin(minX, x);
      maxX = qMax(maxX, x);
      minY = qMin(minY, y);
      maxY = qMax(maxY, y);

      if (distanceSquared <= centerSquared) {
        ++centerForeground;
      }
      if (distanceSquared >= ringInnerSquared) {
        ++ringForeground;
        if (dx != 0 || dy != 0) {
          double angle = std::atan2(static_cast<double> (dy),
                                    static_cast<double> (dx));
          if (angle < 0.0) {
            angle += 2.0 * PI;
          }
          const int sector = qBound(0,
                                    static_cast<int> (angle / (2.0 * PI) * sectors.count()),
                                    sectors.count() - 1);
          sectors [sector] = 1;
        }
      }

      const int quadrant = (dx >= 0 ? 1 : 0) + (dy >= 0 ? 2 : 0);
      quadrants [quadrant] = 1;
    }
  }

  if (area < 8 || minX > maxX || minY > maxY) {
    return 0.0;
  }

  const int localWidth = maxX - minX + 1;
  const int localHeight = maxY - minY + 1;
  const int longSide = qMax(localWidth, localHeight);
  const int shortSide = qMax(1, qMin(localWidth, localHeight));
  const double aspect = static_cast<double> (shortSide) / longSide;
  const double foregroundRatio = static_cast<double> (area) / qMax(1.0, PI * radiusSquared);
  const double centerRatio = static_cast<double> (centerForeground) / qMax(1, centerPixels);
  const double ringRatio = static_cast<double> (ringForeground) / qMax(1, ringPixels);
  int activeSectors = 0;
  int activeQuadrants = 0;
  for (int index = 0; index < sectors.count(); ++index) {
    activeSectors += sectors.at(index);
  }
  for (int index = 0; index < quadrants.count(); ++index) {
    activeQuadrants += quadrants.at(index);
  }

  if (longSide < qMax(5, radius) ||
      longSide > radius * 3 ||
      aspect < 0.52 ||
      activeQuadrants < 3 ||
      activeSectors < 4 ||
      foregroundRatio < 0.055 ||
      foregroundRatio > 0.72) {
    return 0.0;
  }

  const bool openLike = ringRatio >= 0.12 &&
                        foregroundRatio <= 0.42 &&
                        centerRatio <= 0.42 &&
                        activeSectors >= 5;
  const bool filledLike = foregroundRatio >= 0.20 &&
                          centerRatio >= 0.22 &&
                          activeSectors >= 5;
  const bool outlineLike = ringRatio >= 0.10 &&
                           activeSectors >= 5 &&
                           foregroundRatio <= 0.56;
  const bool compactMarkerLike = foregroundRatio >= 0.12 &&
                                 activeSectors >= 4 &&
                                 aspect >= 0.62 &&
                                 longSide <= radius * 3;

  if (!openLike && !filledLike && !outlineLike && !compactMarkerLike) {
    return 0.0;
  }

  const QPointF boundsCenter((minX + maxX) / 2.0,
                             (minY + maxY) / 2.0);
  const double centerOffset = std::sqrt((center.x() - boundsCenter.x()) * (center.x() - boundsCenter.x()) +
                                        (center.y() - boundsCenter.y()) * (center.y() - boundsCenter.y())) /
                              qMax(1.0, static_cast<double> (radius));
  if (centerOffset > 0.58) {
    return 0.0;
  }

  refinedCenter = QPointF(center.x() * 0.75 + boundsCenter.x() * 0.25,
                          center.y() * 0.75 + boundsCenter.y() * 0.25);
  candidateBounds = QRect(QPoint(minX, minY),
                          QPoint(maxX, maxY));

  double score = aspect * 0.30 +
                 qMin(1.0, static_cast<double> (activeSectors) / sectors.count()) * 0.25 +
                 qMin(1.0, ringRatio * 2.0) * 0.20 +
                 qMin(1.0, foregroundRatio * 2.5) * 0.15 +
                 qMax(0.0, 1.0 - centerOffset) * 0.20;
  if (openLike) {
    score += 0.22;
  }
  if (filledLike) {
    score += 0.18;
  }

  return score;
}

QVector<MarkerCandidate> deduplicatedMarkerCandidates(QVector<MarkerCandidate> candidates)
{
  std::sort(candidates.begin(),
            candidates.end(),
            [] (const MarkerCandidate &left, const MarkerCandidate &right) {
              if (std::abs(left.score - right.score) < 0.0001) {
                return left.area > right.area;
              }
              return left.score > right.score;
            });

  QVector<MarkerCandidate> filtered;
  for (int candidateIndex = 0; candidateIndex < candidates.count(); ++candidateIndex) {
    const MarkerCandidate &candidate = candidates.at(candidateIndex);
    bool duplicate = false;
    for (int existingIndex = 0; existingIndex < filtered.count(); ++existingIndex) {
      const MarkerCandidate &existing = filtered.at(existingIndex);
      const QPoint delta = candidate.center - existing.center;
      const double minimumSpacing = qMax(16.0, qMin(candidate.size, existing.size) * 1.05);
      if (delta.x() * delta.x() + delta.y() * delta.y() <= minimumSpacing * minimumSpacing) {
        duplicate = true;
        break;
      }
    }

    if (!duplicate) {
      filtered << candidate;
    }
  }

  return filtered;
}

QVector<MarkerCandidate> localMarkerCandidates(const QVector<unsigned char> &mask,
                                               int width,
                                               int height,
                                               const QRect &bounds,
                                               const Transformation &transformation,
                                               double yMinimum,
                                               double yMaximum)
{
  QVector<MarkerCandidate> candidates;
  const QVector<int> radii = QVector<int>() << 6 << 8 << 10;
  const int step = 2;

  for (int y = bounds.top(); y <= bounds.bottom(); y += step) {
    for (int x = bounds.left(); x <= bounds.right(); x += step) {
      double bestScore = 0.0;
      QPointF bestCenter;
      QRect bestBounds;
      int bestArea = 0;
      int bestRadius = 0;

      for (int radiusIndex = 0; radiusIndex < radii.count(); ++radiusIndex) {
        QPointF refinedCenter;
        QRect candidateBounds;
        int area = 0;
        const int radius = radii.at(radiusIndex);
        const double score = localMarkerScore(mask,
                                              width,
                                              height,
                                              bounds,
                                              QPoint(x, y),
                                              radius,
                                              refinedCenter,
                                              candidateBounds,
                                              area);
        if (score > bestScore) {
          bestScore = score;
          bestCenter = refinedCenter;
          bestBounds = candidateBounds;
          bestArea = area;
          bestRadius = radius;
        }
      }

      if (bestScore < 0.66 || bestRadius <= 0) {
        continue;
      }

      const QPoint center = clampPointToYRange(bestCenter,
                                               transformation,
                                               yMinimum,
                                               yMaximum);
      if (!bounds.contains(center)) {
        continue;
      }

      MarkerCandidate candidate;
      candidate.center = center;
      candidate.bounds = bestBounds.adjusted(-1, -1, 1, 1).intersected(QRect(0, 0, width, height));
      candidate.area = bestArea;
      candidate.size = qMax(bestBounds.width(), bestBounds.height());
      candidate.score = bestScore;
      candidate.coreDensity = localCoreDensity(mask,
                                               width,
                                               height,
                                               center);
      candidate.patch = extractNormalizedPatchAroundPoint(mask,
                                                          width,
                                                          height,
                                                          bestCenter,
                                                          qMax(16.0, candidate.size + 8.0));
      candidate.features = markerFeatures(candidate);
      candidates << candidate;
    }
  }

  return deduplicatedMarkerCandidates(candidates);
}

double circleTemplateScore(const QVector<unsigned char> &mask,
                           int width,
                           int height,
                           const QPoint &center,
                           int radius,
                           bool &filledLike,
                           int &area)
{
  QVector<int> sectors(16, 0);
  int activeSectors = 0;
  int centerForeground = 0;
  int centerPixels = 0;
  int diskForeground = 0;
  int diskPixels = 0;
  int cornerForeground = 0;
  int cornerPixels = 0;
  area = 0;

  const double centerRadiusSquared = radius * radius * 0.18;
  const double diskRadiusSquared = radius * radius * 0.55;
  const double radiusSquared = radius * radius;

  for (int y = qMax(0, center.y() - radius); y <= qMin(height - 1, center.y() + radius); ++y) {
    for (int x = qMax(0, center.x() - radius); x <= qMin(width - 1, center.x() + radius); ++x) {
      const int dx = x - center.x();
      const int dy = y - center.y();
      const double distanceSquared = dx * dx + dy * dy;
      if (distanceSquared > radiusSquared) {
        continue;
      }

      if (distanceSquared <= centerRadiusSquared) {
        ++centerPixels;
      }
      if (distanceSquared <= diskRadiusSquared) {
        ++diskPixels;
      }
      if (std::abs(dx) >= radius * 0.58 &&
          std::abs(dy) >= radius * 0.58 &&
          distanceSquared >= radiusSquared * 0.85) {
        ++cornerPixels;
      }

      if (!mask [indexFor(x, y, width)]) {
        continue;
      }

      ++area;
      if (distanceSquared <= centerRadiusSquared) {
        ++centerForeground;
      }
      if (distanceSquared <= diskRadiusSquared) {
        ++diskForeground;
      }
      if (std::abs(dx) >= radius * 0.58 &&
          std::abs(dy) >= radius * 0.58 &&
          distanceSquared >= radiusSquared * 0.85) {
        ++cornerForeground;
      }
    }
  }

  for (int sector = 0; sector < sectors.count(); ++sector) {
    const double angle = 2.0 * PI * sector / sectors.count();
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    bool sectorHit = false;
    for (int deltaRadius = -1; deltaRadius <= 1 && !sectorHit; ++deltaRadius) {
      const double sampleRadius = radius + deltaRadius;
      for (int tangent = -1; tangent <= 1 && !sectorHit; ++tangent) {
        const int x = qRound(center.x() + cosine * sampleRadius - sine * tangent);
        const int y = qRound(center.y() + sine * sampleRadius + cosine * tangent);
        if (x < 0 || x >= width || y < 0 || y >= height) {
          continue;
        }
        if (mask [indexFor(x, y, width)]) {
          sectorHit = true;
        }
      }
    }
    if (sectorHit) {
      sectors [sector] = 1;
      ++activeSectors;
    }
  }

  const double centerRatio = static_cast<double> (centerForeground) / qMax(1, centerPixels);
  const double diskRatio = static_cast<double> (diskForeground) / qMax(1, diskPixels);
  const double cornerRatio = static_cast<double> (cornerForeground) / qMax(1, cornerPixels);
  const double sectorRatio = static_cast<double> (activeSectors) / sectors.count();
  filledLike = diskRatio >= 0.30 && centerRatio >= 0.30 && activeSectors >= 8;
  const bool openLike = !filledLike &&
                        diskRatio <= 0.56 &&
                        centerRatio <= 0.55 &&
                        activeSectors >= 10;

  if (!filledLike && !openLike) {
    return 0.0;
  }

  if (filledLike) {
    return 1.20 + sectorRatio * 0.30 + diskRatio * 0.25 + centerRatio * 0.20;
  }

  return 1.25 + sectorRatio * 0.35 + (1.0 - centerRatio) * 0.20;
}

QVector<MarkerCandidate> circleTemplateCandidates(const QVector<unsigned char> &mask,
                                                  int width,
                                                  int height,
                                                  const QRect &bounds,
                                                  const Transformation &transformation,
                                                  double yMinimum,
                                                  double yMaximum)
{
  QVector<MarkerCandidate> candidates;
  const QVector<int> radii = QVector<int>() << 5 << 6 << 7 << 8 << 9;
  for (int y = bounds.top(); y <= bounds.bottom(); ++y) {
    for (int x = bounds.left(); x <= bounds.right(); ++x) {
      double bestScore = 0.0;
      int bestRadius = 0;
      int bestArea = 0;
      bool bestFilledLike = false;
      for (int radiusIndex = 0; radiusIndex < radii.count(); ++radiusIndex) {
        bool filledLike = false;
        int area = 0;
        const int radius = radii.at(radiusIndex);
        const double score = circleTemplateScore(mask,
                                                 width,
                                                 height,
                                                 QPoint(x, y),
                                                 radius,
                                                 filledLike,
                                                 area);
        if (score > bestScore) {
          bestScore = score;
          bestRadius = radius;
          bestArea = area;
          bestFilledLike = filledLike;
        }
      }

      if (bestScore <= 0.0 || bestRadius <= 0) {
        continue;
      }

      const QPoint center = clampPointToYRange(QPointF(x, y),
                                               transformation,
                                               yMinimum,
                                               yMaximum);
      if (!bounds.contains(center)) {
        continue;
      }

      MarkerCandidate candidate;
      candidate.center = center;
      candidate.bounds = QRect(QPoint(x - bestRadius, y - bestRadius),
                               QPoint(x + bestRadius, y + bestRadius)).intersected(QRect(0, 0, width, height));
      candidate.area = bestArea;
      candidate.size = bestRadius * 2 + 1;
      candidate.score = bestScore + (bestFilledLike ? 0.03 : 0.06);
      candidate.coreDensity = localCoreDensity(mask,
                                               width,
                                               height,
                                               center);
      candidate.patch = extractNormalizedPatchAroundPoint(mask,
                                                          width,
                                                          height,
                                                          QPointF(x, y),
                                                          qMax(16.0, candidate.size + 8.0));
      candidate.features = markerFeatures(candidate);
      const double foregroundRatio = patchForegroundRatio(candidate.patch);
      const double holeRatio = patchHoleRatio(candidate.patch);
      const double centerRatio = patchCenterForegroundRatio(candidate.patch);
      const double coreRatio = patchCoreForegroundRatio(candidate.patch);
      const double cornerRatio = patchCornerForegroundRatio(candidate.patch);
      const bool cornerHeavyFilled = bestFilledLike &&
                                     cornerRatio >= 0.16 &&
                                     holeRatio < 0.030;
      if (!cornerHeavyFilled &&
          (bestFilledLike ||
           (coreRatio >= 0.65 &&
            candidate.coreDensity >= 0.55 &&
            foregroundRatio >= 0.16) ||
           (centerRatio >= 0.50 &&
            foregroundRatio >= 0.18) ||
           (foregroundRatio >= 0.32 &&
            holeRatio < 0.020))) {
        candidate.shapeKind = 2;
      } else if (!cornerHeavyFilled) {
        candidate.shapeKind = 1;
      }
      candidates << candidate;
    }
  }

  return deduplicatedMarkerCandidates(candidates);
}

int longestMaskedRunOnRow(const QVector<unsigned char> &mask,
                          int width,
                          const QRect &rect,
                          int y)
{
  int best = 0;
  int current = 0;
  for (int x = rect.left(); x <= rect.right(); ++x) {
    if (mask [indexFor(x, y, width)]) {
      ++current;
      best = qMax(best, current);
    } else {
      current = 0;
    }
  }

  return best;
}

int longestMaskedRunOnColumn(const QVector<unsigned char> &mask,
                             int width,
                             const QRect &rect,
                             int x)
{
  int best = 0;
  int current = 0;
  for (int y = rect.top(); y <= rect.bottom(); ++y) {
    if (mask [indexFor(x, y, width)]) {
      ++current;
      best = qMax(best, current);
    } else {
      current = 0;
    }
  }

  return best;
}

bool componentLooksLikeTextBox(const Component &component,
                               const QVector<unsigned char> &mask,
                               int width)
{
  const QRect rect = componentRect(component);
  if (rect.width() < 24 ||
      rect.height() < 12 ||
      rect.height() > 50) {
    return false;
  }

  const int horizontalMinimum = qRound(rect.width() * 0.60);
  const int verticalMinimum = qRound(rect.height() * 0.55);
  int topRun = 0;
  int bottomRun = 0;
  int leftRun = 0;
  int rightRun = 0;

  for (int y = rect.top(); y <= qMin(rect.bottom(), rect.top() + 3); ++y) {
    topRun = qMax(topRun,
                  longestMaskedRunOnRow(mask,
                                        width,
                                        rect,
                                        y));
  }
  for (int y = qMax(rect.top(), rect.bottom() - 3); y <= rect.bottom(); ++y) {
    bottomRun = qMax(bottomRun,
                     longestMaskedRunOnRow(mask,
                                           width,
                                           rect,
                                           y));
  }
  for (int x = rect.left(); x <= qMin(rect.right(), rect.left() + 3); ++x) {
    leftRun = qMax(leftRun,
                   longestMaskedRunOnColumn(mask,
                                            width,
                                            rect,
                                            x));
  }
  for (int x = qMax(rect.left(), rect.right() - 3); x <= rect.right(); ++x) {
    rightRun = qMax(rightRun,
                    longestMaskedRunOnColumn(mask,
                                             width,
                                             rect,
                                             x));
  }

  return topRun >= horizontalMinimum &&
         bottomRun >= horizontalMinimum &&
         leftRun >= verticalMinimum &&
         rightRun >= verticalMinimum;
}

bool candidateNearTextBoxStroke(const MarkerCandidate &candidate,
                                const QVector<unsigned char> &mask,
                                int width,
                                const QRect &bounds)
{
  int horizontalRun = 0;
  int verticalRun = 0;
  const int yMinimum = qMax(bounds.top(), candidate.center.y() - 12);
  const int yMaximum = qMin(bounds.bottom(), candidate.center.y() + 12);
  const int xMinimum = qMax(bounds.left(), candidate.center.x() - 12);
  const int xMaximum = qMin(bounds.right(), candidate.center.x() + 12);

  for (int y = yMinimum; y <= yMaximum; ++y) {
    horizontalRun = qMax(horizontalRun,
                         longestMaskedRunOnRow(mask,
                                               width,
                                               bounds,
                                               y));
  }

  for (int x = xMinimum; x <= xMaximum; ++x) {
    verticalRun = qMax(verticalRun,
                       longestMaskedRunOnColumn(mask,
                                                width,
                                                bounds,
                                                x));
  }

  return horizontalRun >= 24 &&
         verticalRun >= 18;
}

bool candidateInsideTextBoxBand(const MarkerCandidate &candidate,
                                const QVector<unsigned char> &mask,
                                int width,
                                const QRect &bounds)
{
  const int yMinimum = qMax(bounds.top(), candidate.center.y() - 12);
  const int yMaximum = qMin(bounds.bottom(), candidate.center.y() + 12);

  for (int y = yMinimum; y <= yMaximum; ++y) {
    int runStart = -1;
    for (int x = bounds.left(); x <= bounds.right() + 1; ++x) {
      const bool foreground = x <= bounds.right() && mask [indexFor(x, y, width)];
      if (foreground && runStart < 0) {
        runStart = x;
      } else if ((!foreground || x > bounds.right()) && runStart >= 0) {
        const int runEnd = x - 1;
        if (runEnd - runStart + 1 >= 24 &&
            candidate.center.x() >= runStart - 2 &&
            candidate.center.x() <= runEnd + 2) {
          int leftRun = 0;
          int rightRun = 0;
          for (int xx = qMax(bounds.left(), runStart - 3); xx <= qMin(bounds.right(), runStart + 3); ++xx) {
            leftRun = qMax(leftRun,
                           longestMaskedRunOnColumn(mask,
                                                    width,
                                                    bounds,
                                                    xx));
          }
          for (int xx = qMax(bounds.left(), runEnd - 3); xx <= qMin(bounds.right(), runEnd + 3); ++xx) {
            rightRun = qMax(rightRun,
                            longestMaskedRunOnColumn(mask,
                                                     width,
                                                     bounds,
                                                     xx));
          }
          if (leftRun >= 18 || rightRun >= 18) {
            return true;
          }
        }
        runStart = -1;
      }
    }
  }

  return false;
}

QList<QRect> detectedTextBoxRects(const QVector<unsigned char> &mask,
                                  int width,
                                  const QRect &bounds)
{
  QList<Run> horizontalRuns;
  const int minimumWidth = 24;
  const int maximumWidth = qMax(80, bounds.width() / 4);

  for (int y = bounds.top(); y <= bounds.bottom(); ++y) {
    int runStart = -1;
    for (int x = bounds.left(); x <= bounds.right() + 1; ++x) {
      const bool foreground = x <= bounds.right() && mask [indexFor(x, y, width)];
      if (foreground && runStart < 0) {
        runStart = x;
      } else if ((!foreground || x > bounds.right()) && runStart >= 0) {
        const int runEnd = x - 1;
        const int runLength = runEnd - runStart + 1;
        if (runLength >= minimumWidth && runLength <= maximumWidth) {
          Run run;
          run.start = runStart;
          run.end = runEnd;
          run.rowOrColumn = y;
          run.length = runLength;
          horizontalRuns << run;
        }
        runStart = -1;
      }
    }
  }

  QList<QRect> rects;
  for (int topIndex = 0; topIndex < horizontalRuns.count(); ++topIndex) {
    const Run top = horizontalRuns.at(topIndex);
    for (int bottomIndex = topIndex + 1; bottomIndex < horizontalRuns.count(); ++bottomIndex) {
      const Run bottom = horizontalRuns.at(bottomIndex);
      const int boxHeight = bottom.rowOrColumn - top.rowOrColumn + 1;
      if (boxHeight < 10) {
        continue;
      }
      if (boxHeight > 50) {
        break;
      }
      if (std::abs(top.start - bottom.start) > 4 ||
          std::abs(top.end - bottom.end) > 4) {
        continue;
      }

      const QRect candidateRect(QPoint(qMin(top.start, bottom.start),
                                       top.rowOrColumn),
                                QPoint(qMax(top.end, bottom.end),
                                       bottom.rowOrColumn));
      int leftRun = 0;
      int rightRun = 0;
      for (int x = qMax(bounds.left(), candidateRect.left() - 4);
           x <= qMin(bounds.right(), candidateRect.left() + 4);
           ++x) {
        leftRun = qMax(leftRun,
                       longestMaskedRunOnColumn(mask,
                                                width,
                                                bounds,
                                                x));
      }
      for (int x = qMax(bounds.left(), candidateRect.right() - 4);
           x <= qMin(bounds.right(), candidateRect.right() + 4);
           ++x) {
        rightRun = qMax(rightRun,
                        longestMaskedRunOnColumn(mask,
                                                 width,
                                                 bounds,
                                                 x));
      }

      if (leftRun >= qRound(boxHeight * 0.45) ||
          rightRun >= qRound(boxHeight * 0.45)) {
        rects << candidateRect.adjusted(-4, -4, 4, 4);
      }
    }
  }

  return rects;
}

QVector<MarkerCandidate> rejectTextBoxCandidates(const QVector<MarkerCandidate> &candidates,
                                                 const QVector<unsigned char> &mask,
                                                 int width,
                                                 int height,
                                                 const QRect &bounds,
                                                 int *rejectedTextLikeCount)
{
  const QVector<Component> components = connectedComponents(mask,
                                                            width,
                                                            height,
                                                            bounds);
  QList<QRect> textBoxRects;
  textBoxRects << detectedTextBoxRects(mask,
                                       width,
                                       bounds);
  for (int index = 0; index < components.count(); ++index) {
    if (componentLooksLikeTextBox(components.at(index),
                                  mask,
                                  width)) {
      textBoxRects << componentRect(components.at(index)).adjusted(-4, -4, 4, 4);
    }
  }

  if (textBoxRects.isEmpty()) {
    return candidates;
  }

  QVector<MarkerCandidate> filtered;
  for (int candidateIndex = 0; candidateIndex < candidates.count(); ++candidateIndex) {
    bool inTextBox = false;
    for (int rectIndex = 0; rectIndex < textBoxRects.count(); ++rectIndex) {
      if (textBoxRects.at(rectIndex).contains(candidates.at(candidateIndex).center)) {
        inTextBox = true;
        break;
      }
    }
    if (!inTextBox) {
      inTextBox = candidateNearTextBoxStroke(candidates.at(candidateIndex),
                                             mask,
                                             width,
                                             bounds) ||
                  candidateInsideTextBoxBand(candidates.at(candidateIndex),
                                             mask,
                                             width,
                                             bounds);
    }

    if (inTextBox) {
      if (rejectedTextLikeCount != nullptr) {
        ++(*rejectedTextLikeCount);
      }
    } else {
      filtered << candidates.at(candidateIndex);
    }
  }

  return filtered;
}

double medianCandidateSize(const QVector<MarkerCandidate> &candidates)
{
  if (candidates.isEmpty()) {
    return 8.0;
  }

  QVector<double> sizes;
  for (int index = 0; index < candidates.count(); ++index) {
    sizes << candidates.at(index).size;
  }

  std::sort(sizes.begin(), sizes.end());
  return sizes.at(sizes.count() / 2);
}

QVector<MarkerCandidate> rejectVerticalCandidateClusters(const QVector<MarkerCandidate> &candidates,
                                                         const QRect &bounds,
                                                         int *rejectedLineLikeCount)
{
  if (candidates.count() < 4) {
    return candidates;
  }

  QSet<int> rejectedIndexes;
  const int xWindow = qMax(4, qRound(medianCandidateSize(candidates) * 0.7));
  const int spanMinimum = qMax(24, bounds.height() / 6);

  for (int seedIndex = 0; seedIndex < candidates.count(); ++seedIndex) {
    QList<int> alignedIndexes;
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::min();

    for (int index = 0; index < candidates.count(); ++index) {
      if (std::abs(candidates.at(index).center.x() - candidates.at(seedIndex).center.x()) > xWindow) {
        continue;
      }

      alignedIndexes << index;
      minY = qMin(minY, candidates.at(index).center.y());
      maxY = qMax(maxY, candidates.at(index).center.y());
    }

    if (alignedIndexes.count() >= 4 && maxY - minY + 1 >= spanMinimum) {
      for (int index = 0; index < alignedIndexes.count(); ++index) {
        rejectedIndexes.insert(alignedIndexes.at(index));
      }
    }
  }

  QVector<MarkerCandidate> filtered;
  for (int index = 0; index < candidates.count(); ++index) {
    if (!rejectedIndexes.contains(index)) {
      filtered << candidates.at(index);
    }
  }

  if (rejectedLineLikeCount != nullptr) {
    *rejectedLineLikeCount += rejectedIndexes.count();
  }

  return filtered;
}

QVector<MarkerCandidate> rejectTextLikeCandidateClusters(const QVector<MarkerCandidate> &candidates,
                                                         int *rejectedTextLikeCount)
{
  if (candidates.count() < 5) {
    return candidates;
  }

  QSet<int> rejectedIndexes;
  const double medianSize = medianCandidateSize(candidates);
  const int yWindow = qMax(8, qRound(medianSize * 1.2));
  const int xGapMaximum = qMax(12, qRound(medianSize * 1.8));
  const int xSpanMinimum = qMax(24, qRound(medianSize * 2.5));
  const int ySpanMaximum = qMax(18, qRound(medianSize * 2.2));

  QVector<int> order;
  for (int index = 0; index < candidates.count(); ++index) {
    order << index;
  }

  std::sort(order.begin(),
            order.end(),
            [&candidates] (int left, int right) {
              if (candidates.at(left).center.y() == candidates.at(right).center.y()) {
                return candidates.at(left).center.x() < candidates.at(right).center.x();
              }
              return candidates.at(left).center.y() < candidates.at(right).center.y();
            });

  for (int startOrder = 0; startOrder < order.count(); ++startOrder) {
    QList<int> run;
    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::min();
    int previousX = -1;
    const int seedY = candidates.at(order.at(startOrder)).center.y();

    for (int orderIndex = startOrder; orderIndex < order.count(); ++orderIndex) {
      const MarkerCandidate &candidate = candidates.at(order.at(orderIndex));
      if (std::abs(candidate.center.y() - seedY) > yWindow) {
        if (candidate.center.y() > seedY + yWindow) {
          break;
        }
        continue;
      }

      if (!run.isEmpty() && candidate.center.x() - previousX > xGapMaximum) {
        break;
      }

      run << order.at(orderIndex);
      previousX = candidate.center.x();
      minX = qMin(minX, candidate.bounds.left());
      maxX = qMax(maxX, candidate.bounds.right());
      minY = qMin(minY, candidate.bounds.top());
      maxY = qMax(maxY, candidate.bounds.bottom());
    }

    if (run.count() >= 4 &&
        maxX - minX + 1 >= xSpanMinimum &&
        maxY - minY + 1 <= ySpanMaximum) {
      for (int index = 0; index < run.count(); ++index) {
        rejectedIndexes.insert(run.at(index));
      }
    }
  }

  QVector<MarkerCandidate> filtered;
  for (int index = 0; index < candidates.count(); ++index) {
    if (!rejectedIndexes.contains(index)) {
      filtered << candidates.at(index);
    }
  }

  if (rejectedTextLikeCount != nullptr) {
    *rejectedTextLikeCount += rejectedIndexes.count();
  }

  return filtered;
}

void updateClusterCentroid(MarkerCluster &cluster,
                           const QVector<MarkerCandidate> &candidates)
{
  if (cluster.candidateIndexes.isEmpty()) {
    cluster.centroid.clear();
    return;
  }

  const int featureCount = candidates.at(cluster.candidateIndexes.first()).features.count();
  cluster.centroid = QVector<double>(featureCount, 0.0);

  for (int clusterIndex = 0; clusterIndex < cluster.candidateIndexes.count(); ++clusterIndex) {
    const MarkerCandidate &candidate = candidates.at(cluster.candidateIndexes.at(clusterIndex));
    for (int featureIndex = 0; featureIndex < featureCount; ++featureIndex) {
      cluster.centroid [featureIndex] += candidate.features.at(featureIndex);
    }
  }

  for (int featureIndex = 0; featureIndex < featureCount; ++featureIndex) {
    cluster.centroid [featureIndex] /= cluster.candidateIndexes.count();
  }
}

QVector<MarkerCluster> clusterMarkerCandidates(const QVector<MarkerCandidate> &candidates)
{
  QVector<MarkerCluster> clusters;

  for (int candidateIndex = 0; candidateIndex < candidates.count(); ++candidateIndex) {
    const MarkerCandidate &candidate = candidates.at(candidateIndex);
    int bestCluster = -1;
    double bestDistance = std::numeric_limits<double>::max();

    for (int clusterIndex = 0; clusterIndex < clusters.count(); ++clusterIndex) {
      const double shapeDistance = featureDistance(candidate.features,
                                                   clusters [clusterIndex].centroid);
      const MarkerCandidate &representative = candidates.at(clusters [clusterIndex].candidateIndexes.first());
      const double sizeDistance = std::abs(candidate.size - representative.size) /
                                  qMax(1.0, qMax(candidate.size, representative.size));
      const double distance = shapeDistance + sizeDistance * 0.08;
      if (distance < bestDistance) {
        bestDistance = distance;
        bestCluster = clusterIndex;
      }
    }

    if (bestCluster >= 0 && bestDistance <= MARKER_CLUSTER_DISTANCE_THRESHOLD) {
      clusters [bestCluster].candidateIndexes << candidateIndex;
      updateClusterCentroid(clusters [bestCluster], candidates);
    } else {
      MarkerCluster cluster;
      cluster.candidateIndexes << candidateIndex;
      cluster.centroid = candidate.features;
      cluster.meanDistance = 0.0;
      cluster.sizeVariation = 0.0;
      clusters << cluster;
    }
  }

  for (int clusterIndex = 0; clusterIndex < clusters.count(); ++clusterIndex) {
    MarkerCluster &cluster = clusters [clusterIndex];
    double distanceTotal = 0.0;
    double sizeTotal = 0.0;
    for (int index = 0; index < cluster.candidateIndexes.count(); ++index) {
      const MarkerCandidate &candidate = candidates.at(cluster.candidateIndexes.at(index));
      distanceTotal += featureDistance(candidate.features, cluster.centroid);
      sizeTotal += candidate.size;
    }

    const double meanSize = sizeTotal / qMax(1, cluster.candidateIndexes.count());
    double sizeVariance = 0.0;
    for (int index = 0; index < cluster.candidateIndexes.count(); ++index) {
      const MarkerCandidate &candidate = candidates.at(cluster.candidateIndexes.at(index));
      const double delta = candidate.size - meanSize;
      sizeVariance += delta * delta;
    }

    cluster.meanDistance = distanceTotal / qMax(1, cluster.candidateIndexes.count());
    cluster.sizeVariation = std::sqrt(sizeVariance / qMax(1, cluster.candidateIndexes.count())) /
                            qMax(1.0, meanSize);
  }

  return clusters;
}

bool pointsLookLikeVerticalArtifact(const QList<QPoint> &points)
{
  if (points.count() < 4) {
    return false;
  }

  int minX = std::numeric_limits<int>::max();
  int maxX = std::numeric_limits<int>::min();
  int minY = std::numeric_limits<int>::max();
  int maxY = std::numeric_limits<int>::min();

  for (int index = 0; index < points.count(); ++index) {
    minX = qMin(minX, points.at(index).x());
    maxX = qMax(maxX, points.at(index).x());
    minY = qMin(minY, points.at(index).y());
    maxY = qMax(maxY, points.at(index).y());
  }

  const int xSpan = maxX - minX + 1;
  const int ySpan = maxY - minY + 1;
  return xSpan <= qMax(5, ySpan / 12) && ySpan >= 24;
}

bool pointsHavePlausibleSessionSpread(const QList<QPoint> &points)
{
  if (points.count() <= 2) {
    return true;
  }

  QList<int> sortedX;
  for (int index = 0; index < points.count(); ++index) {
    sortedX << points.at(index).x();
  }
  std::sort(sortedX.begin(), sortedX.end());

  QList<int> binCounts;
  for (int index = 0; index < sortedX.count(); ++index) {
    if (binCounts.isEmpty() ||
        std::abs(sortedX.at(index) - sortedX.at(index - 1)) > 6) {
      binCounts << 1;
    } else {
      ++binCounts [binCounts.count() - 1];
    }
  }

  int largestBin = 0;
  for (int index = 0; index < binCounts.count(); ++index) {
    largestBin = qMax(largestBin, binCounts.at(index));
  }

  if (binCounts.count() < qMin(2, points.count())) {
    return false;
  }

  if (largestBin >= qMax(4, qRound(points.count() * 0.60))) {
    return false;
  }

  return true;
}

QString inferredMarkerGroupName(const MarkerCluster &cluster,
                                const QVector<MarkerCandidate> &candidates)
{
  if (cluster.candidateIndexes.isEmpty()) {
    return QString();
  }

  double foregroundTotal = 0.0;
  double holeTotal = 0.0;
  double aspectTotal = 0.0;
  double centerTotal = 0.0;
  for (int index = 0; index < cluster.candidateIndexes.count(); ++index) {
    const MarkerCandidate &candidate = candidates.at(cluster.candidateIndexes.at(index));
    foregroundTotal += patchForegroundRatio(candidate.patch);
    holeTotal += patchHoleRatio(candidate.patch);
    centerTotal += patchCenterForegroundRatio(candidate.patch);
    aspectTotal += static_cast<double> (qMin(candidate.bounds.width(), candidate.bounds.height())) /
                   qMax(1, qMax(candidate.bounds.width(), candidate.bounds.height()));
  }

  const double count = qMax(1, cluster.candidateIndexes.count());
  const double foregroundRatio = foregroundTotal / count;
  const double holeRatio = holeTotal / count;
  const double centerRatio = centerTotal / count;
  const double aspect = aspectTotal / count;

  if (foregroundRatio >= 0.24 &&
      holeRatio < 0.030 &&
      centerRatio >= 0.35 &&
      aspect >= 0.62) {
    return QObject::tr("filled circle-like markers");
  }

  if ((holeRatio >= 0.030 || foregroundRatio <= 0.30) &&
      foregroundRatio <= 0.40 &&
      centerRatio <= 0.48 &&
      aspect >= 0.62) {
    return QObject::tr("open circle-like markers");
  }

  return QString();
}

QList<AutoCurveGroup> groupsFromCircleTemplateKinds(const QVector<MarkerCandidate> &candidates)
{
  QList<AutoCurveGroup> groups;
  for (int shapeKind = 1; shapeKind <= 2; ++shapeKind) {
    QVector<MarkerCluster> clusters;
    for (int candidateIndex = 0; candidateIndex < candidates.count(); ++candidateIndex) {
      if (candidates.at(candidateIndex).shapeKind != shapeKind) {
        continue;
      }

      int bestCluster = -1;
      double bestDistance = std::numeric_limits<double>::max();
      for (int clusterIndex = 0; clusterIndex < clusters.count(); ++clusterIndex) {
        const double shapeDistance = featureDistance(candidates.at(candidateIndex).features,
                                                     clusters [clusterIndex].centroid);
        const MarkerCandidate &representative = candidates.at(clusters [clusterIndex].candidateIndexes.first());
        const double sizeDistance = std::abs(candidates.at(candidateIndex).size - representative.size) /
                                    qMax(1.0, qMax(candidates.at(candidateIndex).size, representative.size));
        const double distance = shapeDistance + sizeDistance * 0.08;
        if (distance < bestDistance) {
          bestDistance = distance;
          bestCluster = clusterIndex;
        }
      }

      const double kindDistanceThreshold = shapeKind == 1 ? 0.16 : 0.20;
      if (bestCluster >= 0 && bestDistance <= kindDistanceThreshold) {
        clusters [bestCluster].candidateIndexes << candidateIndex;
        updateClusterCentroid(clusters [bestCluster], candidates);
      } else {
        MarkerCluster cluster;
        cluster.candidateIndexes << candidateIndex;
        cluster.centroid = candidates.at(candidateIndex).features;
        cluster.meanDistance = 0.0;
        cluster.sizeVariation = 0.0;
        clusters << cluster;
      }
    }

    for (int clusterIndex = 0; clusterIndex < clusters.count(); ++clusterIndex) {
      AutoCurveGroup group;
      group.name = shapeKind == 1 ?
                   QObject::tr("open circle-like markers") :
                   QObject::tr("filled marker matches");
      for (int index = 0; index < clusters.at(clusterIndex).candidateIndexes.count(); ++index) {
        addCurvePointIfSeparated(group.points,
                                 candidates.at(clusters.at(clusterIndex).candidateIndexes.at(index)).center);
      }

      if (!group.points.isEmpty()) {
        group.confidence = 100.0 + group.points.count();
        groups << group;
      }
    }
  }

  return groups;
}

QVector<double> filledCandidateDescriptor(const MarkerCandidate &candidate)
{
  const double foregroundRatio = patchForegroundRatio(candidate.patch);
  const double holeRatio = patchHoleRatio(candidate.patch);
  const double centerRatio = patchCenterForegroundRatio(candidate.patch);
  const double coreRatio = patchCoreForegroundRatio(candidate.patch);
  const double cornerRatio = patchCornerForegroundRatio(candidate.patch);
  const double edgeDensity = patchEdgeDensity(candidate.patch);
  const double aspect = static_cast<double> (qMin(candidate.bounds.width(), candidate.bounds.height())) /
                        qMax(1, qMax(candidate.bounds.width(), candidate.bounds.height()));

  QVector<double> descriptor;
  descriptor << foregroundRatio
             << centerRatio * 1.2
             << coreRatio * 1.8
             << candidate.coreDensity * 2.2
             << holeRatio * 1.8
             << cornerRatio * 2.2
             << edgeDensity
             << aspect
             << candidate.size / 32.0;
  return descriptor;
}

bool candidateLooksFilledForGrouping(const MarkerCandidate &candidate)
{
  const double foregroundRatio = patchForegroundRatio(candidate.patch);
  const double holeRatio = patchHoleRatio(candidate.patch);

  return foregroundRatio >= 0.20 &&
         holeRatio <= 0.035;
}

double descriptorDistance(const QVector<double> &left,
                          const QVector<double> &right)
{
  const int count = qMin(left.count(), right.count());
  if (count == 0) {
    return std::numeric_limits<double>::max();
  }

  double sum = 0.0;
  for (int index = 0; index < count; ++index) {
    const double delta = left.at(index) - right.at(index);
    sum += delta * delta;
  }
  return std::sqrt(sum / count);
}

QList<AutoCurveGroup> groupsFromFilledSimilarity(const QVector<MarkerCandidate> &candidates)
{
  struct FilledCluster
  {
    QList<int> indexes;
    QVector<double> centroid;
  };

  QVector<FilledCluster> clusters;
  QList<int> filledCandidateIndexes;
  for (int candidateIndex = 0; candidateIndex < candidates.count(); ++candidateIndex) {
    if (!candidateLooksFilledForGrouping(candidates.at(candidateIndex))) {
      continue;
    }
    filledCandidateIndexes << candidateIndex;

    const QVector<double> descriptor = filledCandidateDescriptor(candidates.at(candidateIndex));
    int bestCluster = -1;
    double bestDistance = std::numeric_limits<double>::max();
    for (int clusterIndex = 0; clusterIndex < clusters.count(); ++clusterIndex) {
      const double distance = descriptorDistance(descriptor,
                                                 clusters.at(clusterIndex).centroid);
      if (distance < bestDistance) {
        bestDistance = distance;
        bestCluster = clusterIndex;
      }
    }

    if (bestCluster >= 0 && bestDistance <= 0.08) {
      clusters [bestCluster].indexes << candidateIndex;
      QVector<double> centroid(clusters.at(bestCluster).centroid.count(), 0.0);
      for (int index = 0; index < clusters.at(bestCluster).indexes.count(); ++index) {
        const QVector<double> current = filledCandidateDescriptor(candidates.at(clusters.at(bestCluster).indexes.at(index)));
        for (int featureIndex = 0; featureIndex < centroid.count(); ++featureIndex) {
          centroid [featureIndex] += current.at(featureIndex);
        }
      }
      for (int featureIndex = 0; featureIndex < centroid.count(); ++featureIndex) {
        centroid [featureIndex] /= clusters.at(bestCluster).indexes.count();
      }
      clusters [bestCluster].centroid = centroid;
    } else {
      FilledCluster cluster;
      cluster.indexes << candidateIndex;
      cluster.centroid = descriptor;
      clusters << cluster;
    }
  }

  QList<AutoCurveGroup> groups;
  bool hasRepeatedFilledCluster = false;
  for (int clusterIndex = 0; clusterIndex < clusters.count(); ++clusterIndex) {
    AutoCurveGroup group;
    group.name = QObject::tr("filled marker matches");
    for (int index = 0; index < clusters.at(clusterIndex).indexes.count(); ++index) {
      addCurvePointIfSeparated(group.points,
                               candidates.at(clusters.at(clusterIndex).indexes.at(index)).center);
    }
    if (!group.points.isEmpty()) {
      group.confidence = 80.0 + group.points.count();
      groups << group;
      if (group.points.count() >= 4) {
        hasRepeatedFilledCluster = true;
      }
    }
  }

  if (!hasRepeatedFilledCluster && filledCandidateIndexes.count() >= 4) {
    AutoCurveGroup group;
    group.name = QObject::tr("filled marker matches");
    for (int index = 0; index < filledCandidateIndexes.count(); ++index) {
      addCurvePointIfSeparated(group.points,
                               candidates.at(filledCandidateIndexes.at(index)).center);
    }
    if (!group.points.isEmpty()) {
      group.confidence = 70.0 + group.points.count();
      groups << group;
    }
  }

  return groups;
}

QList<AutoCurveGroup> groupsFromClusters(const QVector<MarkerCluster> &clusters,
                                         const QVector<MarkerCandidate> &candidates)
{
  QList<AutoCurveGroup> groups;
  int totalCount = 0;
  for (int index = 0; index < clusters.count(); ++index) {
    totalCount += clusters.at(index).candidateIndexes.count();
  }

  const int minimumGroupSize = totalCount >= 4 ? 4 : 2;

  for (int clusterIndex = 0; clusterIndex < clusters.count(); ++clusterIndex) {
    const MarkerCluster &cluster = clusters.at(clusterIndex);
    if (cluster.candidateIndexes.isEmpty()) {
      continue;
    }

    AutoCurveGroup group;
    for (int index = 0; index < cluster.candidateIndexes.count(); ++index) {
      addCurvePointIfSeparated(group.points,
                               candidates.at(cluster.candidateIndexes.at(index)).center);
    }

    std::sort(group.points.begin(),
              group.points.end(),
              [] (const QPoint &left, const QPoint &right) {
                if (left.x() == right.x()) {
                  return left.y() < right.y();
                }
                return left.x() < right.x();
              });

    group.confidence = static_cast<double> (group.points.count()) /
                       (1.0 + cluster.meanDistance * 10.0 + cluster.sizeVariation * 3.0);
    group.name = inferredMarkerGroupName(cluster,
                                         candidates);
    groups << group;
  }

  const QList<AutoCurveGroup> circleKindGroups = groupsFromCircleTemplateKinds(candidates);
  for (int index = 0; index < circleKindGroups.count(); ++index) {
    groups << circleKindGroups.at(index);
  }
  const QList<AutoCurveGroup> filledSimilarityGroups = groupsFromFilledSimilarity(candidates);
  for (int index = 0; index < filledSimilarityGroups.count(); ++index) {
    groups << filledSimilarityGroups.at(index);
  }

  for (int left = 0; left < groups.count(); ++left) {
    if (groups.at(left).name.isEmpty()) {
      continue;
    }

    for (int right = left + 1; right < groups.count();) {
      if (groups.at(right).name != groups.at(left).name) {
        ++right;
        continue;
      }
      if (groups.at(left).points.count() < 2 ||
          groups.at(right).points.count() < 2) {
        ++right;
        continue;
      }
      if ((groups.at(left).confidence >= 100.0) !=
          (groups.at(right).confidence >= 100.0)) {
        ++right;
        continue;
      }

      if (!groups.at(left).name.contains(QObject::tr("open"), Qt::CaseInsensitive)) {
        int overlap = 0;
        for (int pointIndex = 0; pointIndex < groups.at(right).points.count(); ++pointIndex) {
          for (int existingPointIndex = 0; existingPointIndex < groups.at(left).points.count(); ++existingPointIndex) {
            const QPoint delta = groups.at(right).points.at(pointIndex) - groups.at(left).points.at(existingPointIndex);
            if (delta.x() * delta.x() + delta.y() * delta.y() <= CURVE_POINT_MIN_SPACING * CURVE_POINT_MIN_SPACING) {
              ++overlap;
              break;
            }
          }
        }
        if (overlap < qMax(2, qMin(groups.at(left).points.count(), groups.at(right).points.count()) / 2)) {
          ++right;
          continue;
        }
      }

      for (int pointIndex = 0; pointIndex < groups.at(right).points.count(); ++pointIndex) {
        addCurvePointIfSeparated(groups [left].points,
                                 groups.at(right).points.at(pointIndex));
      }
      groups [left].confidence = qMax(groups.at(left).confidence,
                                 groups.at(right).confidence) +
                                 groups.at(right).points.count() * 0.01;
      groups.removeAt(right);
    }
  }

  QList<AutoCurveGroup> acceptedGroups;
  for (int index = 0; index < groups.count(); ++index) {
    if (groups.at(index).points.count() < minimumGroupSize) {
      continue;
    }
    if (pointsLookLikeVerticalArtifact(groups.at(index).points)) {
      continue;
    }
    if (!pointsHavePlausibleSessionSpread(groups.at(index).points)) {
      continue;
    }

    acceptedGroups << groups.at(index);
  }
  groups = acceptedGroups;

  for (int index = 0; index < groups.count(); ++index) {
    std::sort(groups [index].points.begin(),
              groups [index].points.end(),
              [] (const QPoint &left, const QPoint &right) {
                if (left.x() == right.x()) {
                  return left.y() < right.y();
                }
                return left.x() < right.x();
              });
  }

  std::sort(groups.begin(),
            groups.end(),
            [] (const AutoCurveGroup &left, const AutoCurveGroup &right) {
              const bool leftTemplate = left.confidence >= 100.0;
              const bool rightTemplate = right.confidence >= 100.0;
              if (leftTemplate || rightTemplate) {
                return left.confidence > right.confidence;
              }
              if (left.points.count() == right.points.count()) {
                return left.confidence > right.confidence;
              }
              return left.points.count() > right.points.count();
            });

  QList<AutoCurveGroup> uniqueGroups;
  for (int groupIndex = 0; groupIndex < groups.count(); ++groupIndex) {
    const AutoCurveGroup &group = groups.at(groupIndex);
    bool overlapsExisting = false;
    for (int existingIndex = 0; existingIndex < uniqueGroups.count() && !overlapsExisting; ++existingIndex) {
      int overlap = 0;
      for (int pointIndex = 0; pointIndex < group.points.count(); ++pointIndex) {
        for (int existingPointIndex = 0; existingPointIndex < uniqueGroups.at(existingIndex).points.count(); ++existingPointIndex) {
          const QPoint delta = group.points.at(pointIndex) - uniqueGroups.at(existingIndex).points.at(existingPointIndex);
          if (delta.x() * delta.x() + delta.y() * delta.y() <= CURVE_POINT_MIN_SPACING * CURVE_POINT_MIN_SPACING) {
            ++overlap;
            break;
          }
        }
      }

      if (overlap >= qMax(2, qMin(group.points.count(), uniqueGroups.at(existingIndex).points.count()) / 2)) {
        overlapsExisting = true;
      }
    }

    if (!overlapsExisting) {
      uniqueGroups << group;
    }
  }
  groups = uniqueGroups;

  for (int index = 0; index < groups.count(); ++index) {
    if (groups.at(index).name.isEmpty()) {
      groups [index].name = QObject::tr("Marker Group %1").arg(index + 1);
    }
  }

  return groups;
}

QPoint clampPointToYRange(const QPointF &pointScreen,
                          const Transformation &transformation,
                          double yMinimum,
                          double yMaximum)
{
  QPointF graph;
  transformation.transformScreenToRawGraph(pointScreen, graph);
  graph.setY(qBound(yMinimum, graph.y(), yMaximum));

  QPointF adjustedScreen;
  transformation.transformRawGraphToScreen(graph, adjustedScreen);
  return QPoint(qRound(adjustedScreen.x()), qRound(adjustedScreen.y()));
}

void addCurvePointIfSeparated(QList<QPoint> &points,
                              const QPoint &point)
{
  for (int index = 0; index < points.count(); ++index) {
    const QPoint existing = points.at(index);
    const int dx = existing.x() - point.x();
    const int dy = existing.y() - point.y();
    if (dx * dx + dy * dy < CURVE_POINT_MIN_SPACING * CURVE_POINT_MIN_SPACING) {
      return;
    }
  }

  points << point;
}

bool detectMarkerCandidatesForAutoCurve(const QImage &image,
                                        const QRect &plotRect,
                                        const Transformation &transformation,
                                        double yMinimum,
                                        double yMaximum,
                                        AutoCurveResult &result,
                                        QVector<MarkerCandidate> &candidates,
                                        QVector<unsigned char> *foregroundMaskOut,
                                        QVector<unsigned char> *lineSuppressedMaskOut,
                                        QRect *boundsOut,
                                        int *widthOut,
                                        int *heightOut)
{
  result.rawCandidateCount = 0;
  result.rejectedLineLikeCount = 0;
  result.rejectedTextLikeCount = 0;
  result.finalCandidateCount = 0;

  if (image.isNull() || plotRect.isEmpty() || !transformation.transformIsDefined()) {
    result.message = QObject::tr("No calibrated image area is available for Auto Curve.");
    return false;
  }

  const QImage working = image.convertToFormat(QImage::Format_ARGB32);
  const int width = working.width();
  const int height = working.height();
  const QRect imageRect(0, 0, width, height);
  const int borderMargin = 0;
  const QRect bounds = plotRect.adjusted(borderMargin,
                                         borderMargin,
                                         -borderMargin,
                                         -borderMargin).intersected(imageRect);
  if (bounds.isEmpty()) {
    result.message = QObject::tr("No calibrated image area is available for Auto Curve.");
    return false;
  }

  const QVector<unsigned char> mask = foregroundMask(working);
  QVector<unsigned char> lineSuppressedMask = suppressLongRuns(mask,
                                                               width,
                                                               height,
                                                               bounds);
  lineSuppressedMask = suppressColumnAndRowArtifacts(lineSuppressedMask,
                                                     width,
                                                     height,
                                                     bounds,
                                                     &result.rejectedLineLikeCount);
  const QVector<unsigned char> denseMask = denseForegroundMask(lineSuppressedMask,
                                                              width,
                                                              height,
                                                              bounds);
  candidates = markerCandidatesFromComponents(denseMask,
                                              width,
                                              height,
                                              bounds,
                                              transformation,
                                              yMinimum,
                                              yMaximum,
                                              &result.rawCandidateCount,
                                              &result.rejectedLineLikeCount);
  if (result.rawCandidateCount > AUTO_CURVE_MAX_RAW_CANDIDATES) {
    result.message = QObject::tr("Auto Curve found too many candidates. No points were added. Try a clearer graph or crop the plot area.");
    return false;
  }

  const QVector<MarkerCandidate> circleCandidates = circleTemplateCandidates(lineSuppressedMask,
                                                                             width,
                                                                             height,
                                                                             bounds,
                                                                             transformation,
                                                                             yMinimum,
                                                                             yMaximum);
  const QVector<MarkerCandidate> localCandidates = localMarkerCandidates(lineSuppressedMask,
                                                                         width,
                                                                         height,
                                                                         bounds,
                                                                         transformation,
                                                                         yMinimum,
                                                                         yMaximum);
  for (int index = 0; index < circleCandidates.count(); ++index) {
    candidates << circleCandidates.at(index);
  }
  for (int index = 0; index < localCandidates.count(); ++index) {
    candidates << localCandidates.at(index);
  }
  candidates = deduplicatedMarkerCandidates(candidates);
  result.rawCandidateCount += circleCandidates.count() + localCandidates.count();

  if (result.rawCandidateCount > AUTO_CURVE_MAX_RAW_CANDIDATES) {
    result.message = QObject::tr("Auto Curve found too many candidates. No points were added. Try a clearer graph or crop the plot area.");
    return false;
  }

  candidates = rejectTextBoxCandidates(candidates,
                                       mask,
                                       width,
                                       height,
                                       bounds,
                                       &result.rejectedTextLikeCount);
  candidates = rejectVerticalCandidateClusters(candidates,
                                               bounds,
                                               &result.rejectedLineLikeCount);
  candidates = rejectTextLikeCandidateClusters(candidates,
                                               &result.rejectedTextLikeCount);
  result.finalCandidateCount = candidates.count();

  if (result.finalCandidateCount > AUTO_CURVE_MAX_RAW_CANDIDATES) {
    result.groups.clear();
    result.message = QObject::tr("Auto Curve found too many candidates. No points were added. Try a clearer graph or crop the plot area.");
    return false;
  }

  if (foregroundMaskOut != nullptr) {
    *foregroundMaskOut = mask;
  }
  if (lineSuppressedMaskOut != nullptr) {
    *lineSuppressedMaskOut = lineSuppressedMask;
  }
  if (boundsOut != nullptr) {
    *boundsOut = bounds;
  }
  if (widthOut != nullptr) {
    *widthOut = width;
  }
  if (heightOut != nullptr) {
    *heightOut = height;
  }

  return true;
}

MarkerCandidate markerCandidateFromExamplePoint(const QVector<unsigned char> &mask,
                                                int width,
                                                int height,
                                                const QRect &bounds,
                                                const Transformation &transformation,
                                                double yMinimum,
                                                double yMaximum,
                                                const QPoint &examplePoint)
{
  MarkerCandidate candidate;
  const int radius = 10;
  const int left = qMax(bounds.left(), examplePoint.x() - radius);
  const int right = qMin(bounds.right(), examplePoint.x() + radius);
  const int top = qMax(bounds.top(), examplePoint.y() - radius);
  const int bottom = qMin(bounds.bottom(), examplePoint.y() + radius);

  int minX = std::numeric_limits<int>::max();
  int maxX = std::numeric_limits<int>::min();
  int minY = std::numeric_limits<int>::max();
  int maxY = std::numeric_limits<int>::min();
  for (int y = top; y <= bottom; ++y) {
    for (int x = left; x <= right; ++x) {
      const int dx = x - examplePoint.x();
      const int dy = y - examplePoint.y();
      if (dx * dx + dy * dy > radius * radius ||
          !mask [indexFor(x, y, width)]) {
        continue;
      }
      ++candidate.area;
      minX = qMin(minX, x);
      maxX = qMax(maxX, x);
      minY = qMin(minY, y);
      maxY = qMax(maxY, y);
    }
  }

  if (candidate.area <= 0) {
    minX = qMax(bounds.left(), examplePoint.x() - 8);
    maxX = qMin(bounds.right(), examplePoint.x() + 8);
    minY = qMax(bounds.top(), examplePoint.y() - 8);
    maxY = qMin(bounds.bottom(), examplePoint.y() + 8);
  }

  candidate.center = clampPointToYRange(QPointF(examplePoint),
                                        transformation,
                                        yMinimum,
                                        yMaximum);
  candidate.bounds = QRect(QPoint(minX, minY),
                           QPoint(maxX, maxY));
  candidate.size = qMax(8, qMax(candidate.bounds.width(), candidate.bounds.height()));
  candidate.score = 1.0;
  candidate.coreDensity = localCoreDensity(mask,
                                           width,
                                           height,
                                           candidate.center);
  candidate.patch = extractNormalizedPatchAroundPoint(mask,
                                                      width,
                                                      height,
                                                      QPointF(examplePoint),
                                                      qMax(18.0, candidate.size + 8.0));
  candidate.features = markerFeatures(candidate);

  const double centerRatio = patchCenterForegroundRatio(candidate.patch);
  const double foregroundRatio = patchForegroundRatio(candidate.patch);
  const double holeRatio = patchHoleRatio(candidate.patch);
  if (centerRatio >= 0.50 &&
      foregroundRatio >= 0.18 &&
      holeRatio < 0.030) {
    candidate.shapeKind = 2;
  } else if (foregroundRatio >= 0.05) {
    candidate.shapeKind = 1;
  }

  return candidate;
}

double markerSimilarityToExample(const MarkerCandidate &candidate,
                                 const MarkerCandidate &example)
{
  double distance = featureDistance(candidate.features,
                                    example.features);
  distance += std::abs(candidate.size - example.size) /
              qMax(1.0, qMax(candidate.size, example.size)) * 0.06;
  if (candidate.shapeKind > 0 &&
      example.shapeKind > 0 &&
      candidate.shapeKind != example.shapeKind) {
    distance += 0.12;
  }

  return distance;
}

AutoCurveGroup markerGroupFromExample(const QVector<MarkerCandidate> &candidates,
                                      const QVector<unsigned char> &lineSuppressedMask,
                                      int width,
                                      int height,
                                      const QRect &bounds,
                                      const Transformation &transformation,
                                      double yMinimum,
                                      double yMaximum,
                                      const QPoint &examplePoint)
{
  MarkerCandidate example = markerCandidateFromExamplePoint(lineSuppressedMask,
                                                            width,
                                                            height,
                                                            bounds,
                                                            transformation,
                                                            yMinimum,
                                                            yMaximum,
                                                            examplePoint);

  int nearestIndex = -1;
  int nearestDistanceSquared = std::numeric_limits<int>::max();
  for (int index = 0; index < candidates.count(); ++index) {
    const QPoint delta = candidates.at(index).center - examplePoint;
    const int distanceSquared = delta.x() * delta.x() + delta.y() * delta.y();
    if (distanceSquared < nearestDistanceSquared) {
      nearestDistanceSquared = distanceSquared;
      nearestIndex = index;
    }
  }

  if (nearestIndex >= 0 && nearestDistanceSquared <= 24 * 24) {
    example = candidates.at(nearestIndex);
  }

  double bestDistance = std::numeric_limits<double>::max();
  QVector<double> distances;
  for (int index = 0; index < candidates.count(); ++index) {
    const double distance = markerSimilarityToExample(candidates.at(index),
                                                      example);
    distances << distance;
    bestDistance = qMin(bestDistance, distance);
  }

  const double threshold = example.shapeKind == 2 ?
                           0.40 :
                           example.shapeKind == 1 ?
                           0.20 :
                           qMin(0.20,
                                qMax(0.095,
                                     bestDistance + 0.055));
  AutoCurveGroup group;
  group.name = example.shapeKind == 1 ? QObject::tr("taught open circle-like markers") :
               example.shapeKind == 2 ? QObject::tr("taught filled marker matches") :
               QObject::tr("taught marker matches");
  const bool exampleHasFilledCore = example.coreDensity >= 0.55;
  for (int index = 0; index < candidates.count(); ++index) {
    if (exampleHasFilledCore &&
        candidates.at(index).coreDensity < 0.55) {
      continue;
    }
    if (example.shapeKind == 2 &&
        patchHoleRatio(candidates.at(index).patch) > 0.035) {
      continue;
    }
    if (example.shapeKind == 2 &&
        patchForegroundRatio(candidates.at(index).patch) < 0.20) {
      continue;
    }
    if (distances.at(index) > threshold) {
      continue;
    }
    addCurvePointIfSeparated(group.points,
                             candidates.at(index).center);
  }

  std::sort(group.points.begin(),
            group.points.end(),
            [] (const QPoint &left, const QPoint &right) {
              if (left.x() == right.x()) {
                return left.y() < right.y();
              }
              return left.x() < right.x();
            });
  group.confidence = group.points.count() / qMax(0.05, threshold);

  if (pointsLookLikeVerticalArtifact(group.points) ||
      !pointsHavePlausibleSessionSpread(group.points)) {
    group.points.clear();
  }

  return group;
}

} // namespace

AutoCurveGroup::AutoCurveGroup() :
  confidence (0.0)
{
}

AutoCurveResult::AutoCurveResult() :
  rawCandidateCount (0),
  rejectedLineLikeCount (0),
  rejectedTextLikeCount (0),
  finalCandidateCount (0)
{
}

AutoAxisResult::AutoAxisResult() :
  success (false),
  rotated (false),
  rotationDegrees (0.0)
{
}

AutoAxisResult AutoDigitize::detectAxes(const QImage &image,
                                        double yMaximum)
{
  AutoAxisResult result;
  if (image.isNull()) {
    result.message = QObject::tr("No image is loaded.");
    return result;
  }

  QImage working = image.convertToFormat(QImage::Format_ARGB32);
  const double detectedAngle = estimateHorizontalAngleDegrees(working);
  if (std::abs(detectedAngle) >= AXIS_ROTATION_THRESHOLD_DEGREES &&
      std::abs(detectedAngle) <= MAX_AXIS_ROTATION_DEGREES) {
    working = rotateImage(working, -detectedAngle);
    result.rotated = true;
    result.rotationDegrees = -detectedAngle;
  }

  const QVector<unsigned char> mask = foregroundMask(working);
  Run xAxisRun;
  if (!detectHorizontalAxis(mask, working.width(), working.height(), xAxisRun)) {
    result.message = QObject::tr("Could not find a clear horizontal x-axis line.");
    return result;
  }

  Run yAxisRun;
  if (!detectVerticalAxis(mask, working.width(), working.height(), xAxisRun, yAxisRun)) {
    yAxisRun.start = qMax(0, xAxisRun.rowOrColumn - working.height() / 2);
    yAxisRun.end = xAxisRun.rowOrColumn;
    yAxisRun.rowOrColumn = xAxisRun.start;
    yAxisRun.length = yAxisRun.end - yAxisRun.start + 1;
  }

  const int yAxisX = qBound(0, yAxisRun.rowOrColumn, working.width() - 1);
  const int xAxisY = qBound(0, xAxisRun.rowOrColumn, working.height() - 1);
  const int xAxisRightMinimum = qMin(yAxisX + 1, working.width() - 1);
  const int xAxisRight = qBound(xAxisRightMinimum, xAxisRun.end, working.width() - 1);
  const int yAxisTopMaximum = qMax(0, xAxisY - 1);
  const int yAxisTop = qBound(0, yAxisRun.start, yAxisTopMaximum);
  if (xAxisRight <= yAxisX || yAxisTop >= xAxisY) {
    result.message = QObject::tr("Detected axes are too small to calibrate.");
    return result;
  }

  const double xMaximum = qMax(1.0, static_cast<double> (xAxisRight - yAxisX));

  result.pointsScreen << QPointF(yAxisX, xAxisY)
                      << QPointF(xAxisRight, xAxisY)
                      << QPointF(yAxisX, yAxisTop);
  result.pointsGraph << QPointF(0.0, 0.0)
                     << QPointF(xMaximum, 0.0)
                     << QPointF(0.0, yMaximum);
  result.image = working;
  result.success = true;
  result.message = QObject::tr("Auto axis created three calibration points.");

  return result;
}

QList<QPoint> AutoDigitize::detectCurvePoints(const QImage &image,
                                              const QRect &plotRect,
                                              const Transformation &transformation,
                                              double yMinimum,
                                              double yMaximum)
{
  const AutoCurveResult result = detectCurvePointGroups(image,
                                                        plotRect,
                                                        transformation,
                                                        yMinimum,
                                                        yMaximum);
  QList<QPoint> points;
  if (!result.groups.isEmpty()) {
    points = result.groups.first().points;
  }

  return points;
}

AutoCurveResult AutoDigitize::detectCurvePointGroups(const QImage &image,
                                                     const QRect &plotRect,
                                                     const Transformation &transformation,
                                                     double yMinimum,
                                                     double yMaximum)
{
  AutoCurveResult result;
  QVector<MarkerCandidate> candidates;
  if (!detectMarkerCandidatesForAutoCurve(image,
                                          plotRect,
                                          transformation,
                                          yMinimum,
                                          yMaximum,
                                          result,
                                          candidates,
                                          nullptr,
                                          nullptr,
                                          nullptr,
                                          nullptr,
                                          nullptr)) {
    return result;
  }

  const QVector<MarkerCluster> clusters = clusterMarkerCandidates(candidates);
  result.groups = groupsFromClusters(clusters,
                                     candidates);

  if (result.groups.isEmpty()) {
    result.message = QObject::tr("No repeated marker groups were detected inside the calibrated plot area.");
  } else {
    int pointCount = 0;
    for (int index = 0; index < result.groups.count(); ++index) {
      pointCount += result.groups.at(index).points.count();
    }
    result.message = QObject::tr("Detected %1 marker groups with %2 candidate points.")
                     .arg(result.groups.count())
                     .arg(pointCount);
  }

  return result;
}

AutoCurveResult AutoDigitize::detectCurvePointGroupFromExample(const QImage &image,
                                                               const QRect &plotRect,
                                                               const Transformation &transformation,
                                                               double yMinimum,
                                                               double yMaximum,
                                                               const QPoint &examplePoint)
{
  AutoCurveResult result;
  QVector<MarkerCandidate> candidates;
  QVector<unsigned char> lineSuppressedMask;
  QRect bounds;
  int width = 0;
  int height = 0;
  if (!detectMarkerCandidatesForAutoCurve(image,
                                          plotRect,
                                          transformation,
                                          yMinimum,
                                          yMaximum,
                                          result,
                                          candidates,
                                          nullptr,
                                          &lineSuppressedMask,
                                          &bounds,
                                          &width,
                                          &height)) {
    return result;
  }

  if (!bounds.contains(examplePoint)) {
    result.message = QObject::tr("Auto Curve Teach Marker needs an example point inside the calibrated plot area.");
    return result;
  }

  const AutoCurveGroup group = markerGroupFromExample(candidates,
                                                      lineSuppressedMask,
                                                      width,
                                                      height,
                                                      bounds,
                                                      transformation,
                                                      yMinimum,
                                                      yMaximum,
                                                      examplePoint);
  if (group.points.isEmpty()) {
    result.message = QObject::tr("Auto Curve Teach Marker did not find matching data markers.");
    return result;
  }

  result.groups << group;
  result.message = QObject::tr("Auto Curve Teach Marker detected %1 matching points.")
                   .arg(group.points.count());
  return result;
}
