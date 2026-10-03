# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2025-10-14]

### Added
- **Configuration File Support**: Introduced TOML-based configuration file (`config/default.toml`) to simplify CLI usage and reduce command-line complexity.
- **TOML Dependency**: Added `tomli` package for TOML parsing (compatible with Python < 3.11).
- **Dracula Theme**: Applied Dracula color scheme to the dashboard UI for better visual consistency and modern appearance.

### Changed
- **CLI Refactoring**: Refactored `cli.py` to load default values from configuration file instead of hardcoding them.
- **Default Configuration**: Updated default settings to match common usage patterns:
  - Mode: `ros2`
  - ROS Profile: `px4_interface`
  - Poll Interval: `0.5` seconds
  - Log Level: `DEBUG`
- **Parameter Handling**: Command-line arguments now override configuration file values, maintaining backward compatibility.
- **UI Styling**: Updated `dashboard.css` to use Dracula theme colors directly (removed unsupported CSS variables) and added color reference comments for maintainability.

### Technical Details
- Configuration file supports all CLI parameters with sensible defaults
- Empty strings in config are converted to `None` for optional parameters
- Added `--config` option to specify custom configuration file path
- Maintained support for existing ROS profiles and argument parsing
- CSS now uses direct hex color values compatible with Textual framework

## [2025-10-15]

### Added
- **Parser Unit Tests**: Added `tests/core/test_parsers.py` covering NED→ENU pose conversion, battery percentage/voltage derivation, and VehicleState inheritance scenarios.

### Changed
- **Core Module Split**: Refactored `core/datasource.py` into dedicated modules (`core/base.py`, `core/sim.py`, `core/ros.py`, `core/parsers.py`) to reduce file size and improve cohesion.
- **Public Exports**: Updated `core/__init__.py` and package-level imports to surface the new module structure while keeping existing APIs consistent.

### Deprecated
- **Legacy Import Path**: `uav_tui_dashboard.core.datasource` now operates as a shim emitting a deprecation warning; migrate imports to the new submodules.