# AutoTrace Digitizer v1.0.0 Release Notes

AutoTrace Digitizer is a modified fork of Engauge Digitizer. It is not the official upstream Engauge Digitizer release. Original Engauge Digitizer copyright and license notices are preserved.

## Added

- Windows portable release workflow.
- Auto Axis button in the Digitize menu and drawing toolbar.
- Auto Curve button in the Digitize menu and drawing toolbar, including Shift + Auto Curve Teach Marker mode.
- Portable settings support through `portable-settings.ini`, `ENGAUGE_PORTABLE=1`, and a local `settings` folder.
- US-English default locale unless the user selects another locale.
- Unicode-safe portable path handling for non-English Windows folders.

## Auto Axis

Auto Axis detects the bottom x-axis, supports mild image tilt correction, creates editable axis points, sets x minimum to 0, sets y minimum to 0, and prompts for y maximum.

## Auto Curve

Auto Curve now parses single-case design graph structure rather than raw foreground pixels. It rejects axes, gridlines, phase dividers, dashed separators, labels, text boxes, arrows, brackets, slash breaks, and other annotation artifacts, places one editable curve point at each detected marker center, groups visually matching markers, and cycles detected marker groups on repeated clicks. Hold Shift while clicking Auto Curve to teach one example marker and search for visually similar markers.

## Limitations

- Auto Axis does not read axis numbers automatically.
- Auto Curve is experimental. It is designed for common single-case design graphs, but it may still need correction on low-resolution images, dense gridlines, overlapping labels, unusual marker symbols, crossing series, or connecting lines touching markers.
- Manual review of digitized points is recommended before using exported data.
