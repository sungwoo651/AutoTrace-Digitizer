# AutoTrace Digitizer

AutoTrace Digitizer is a modified fork of Engauge Digitizer. It is not the official upstream Engauge Digitizer release.

This project preserves the original Engauge Digitizer copyright and license notices. Engauge Digitizer was written and maintained by Mark Mitchell. The currently maintained upstream fork used by many Linux distributions is the [GitHub `akhuettel/engauge-digitizer` project](https://github.com/akhuettel/engauge-digitizer).

| Resource | Description |
| --- | --- |
| [Original project information](http://akhuettel.github.io/engauge-digitizer) | Explains what Engauge Digitizer does and how it is used |
| [Active upstream fork](https://github.com/akhuettel/engauge-digitizer) | Upstream Engauge Digitizer source and releases |
| [License](LICENSE) | Original GPL license file preserved from Engauge Digitizer |

## Version

Current AutoTrace Digitizer release: `v1.0.0`.

## AutoTrace Digitizer v1.0.0 Features

AutoTrace Digitizer adds Windows portable packaging and early automation tools for graph digitizing workflows:

- Windows portable release generated locally for GitHub Releases.
- Auto Axis button.
- Auto Curve button.
- Auto Axis detects the bottom x-axis, supports mild image tilt correction, creates editable axis points, sets x minimum to 0, sets y minimum to 0, and prompts for y maximum.
- Auto Curve parses single-case design graph structure, rejects axes, gridlines, phase dividers, labels, text boxes, arrows, brackets, slash breaks, and other annotation artifacts, then creates one editable curve point at each marker center on the selected curve.
- Repeated Auto Curve clicks cycle through visually similar marker groups and replace only the previous auto-created points.
- Shift + Auto Curve starts Teach Marker mode: click one example marker to search the calibrated plot area for visually similar markers.
- Auto Axis and Auto Curve are available from the Digitize menu and drawing toolbar.
- Portable settings support through `portable-settings.ini`, `ENGAUGE_PORTABLE=1`, and a local `settings` folder beside the executable.
- Default locale is US English (`en_US`) unless the user selects another locale in Settings / Main Window.
- Portable paths are resolved relative to the application directory with Qt Unicode-safe path APIs for non-English and Unicode Windows folders.

## Windows Portable Release

The portable ZIP is a generated release artifact and should not be committed to Git.

Expected local release artifact:

`release/AutoTrace-Digitizer-v1.0.0-Windows-Portable.zip`

Expected portable folder:

`release/AutoTrace-Digitizer-Windows-Portable/`

Run AutoTrace Digitizer with:

`Start AutoTrace Digitizer.cmd`

or directly with:

`AutoTraceDigitizer.exe`

## Current Limitations

- Auto Axis does not read axis numbers automatically.
- Auto Curve is experimental. It is designed for common single-case design graphs, but it may still need correction on low-resolution images, dense gridlines, overlapping labels, unusual marker symbols, crossing series, or connecting lines touching markers.
- Manual review of digitized points is recommended before using exported data.
