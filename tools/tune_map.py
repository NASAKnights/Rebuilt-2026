#!/usr/bin/env python3
import csv
import hashlib
import json
import math
import os
import sys
import tempfile
import time

try:
    import ntcore
except ImportError:
    print("Error: ntcore is not installed.")
    print("Install GUI dependencies with: python -m pip install -r tools/requirements.txt")
    sys.exit(1)

try:
    from PySide6.QtCharts import QChart, QChartView, QLineSeries, QScatterSeries, QValueAxis
    from PySide6.QtCore import QPointF, QSignalBlocker, Qt, QTimer
    from PySide6.QtGui import QAction, QColor, QPainter, QPen
    from PySide6.QtWidgets import (
        QApplication,
        QAbstractItemView,
        QCheckBox,
        QDialog,
        QDialogButtonBox,
        QDoubleSpinBox,
        QFileDialog,
        QFormLayout,
        QFrame,
        QGridLayout,
        QHBoxLayout,
        QHeaderView,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QTableWidget,
        QVBoxLayout,
        QWidget,
    )
except ImportError:
    print("Error: PySide6 is not installed.")
    print("Install GUI dependencies with: python -m pip install -r tools/requirements.txt")
    sys.exit(1)

PUBLISH_PERIOD_MS = 250
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LOCAL_CSV_DIR = os.path.join(REPO_ROOT, "src", "main", "deploy")
SETTINGS_PATH = os.path.join(os.path.dirname(__file__), ".tune_map_qt_settings.json")
FALLBACK_SERVER = "127.0.0.1"
FALLBACK_NT_PATH = "LaunchCalculator/Points"
CHART_COLORS = [
    "#6ea0ff",
    "#48d17a",
    "#f3c64e",
    "#ff7a70",
    "#b18cff",
    "#38c7d8",
    "#ff9f43",
    "#a3e635",
]


class CellDelegateState:
    def __init__(self):
        self.invalid_cells = set()
        self.dirty = False
        self.loading = False


class AddRowDialog(QDialog):
    def __init__(self, param_names, next_distance, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add Target")
        self._value_inputs = []

        layout = QVBoxLayout(self)
        form = QFormLayout()

        self.distance_input = QLineEdit(f"{next_distance:.1f}")
        form.addRow("Distance", self.distance_input)

        for name in param_names:
            edit = QLineEdit("0.00")
            form.addRow(name, edit)
            self._value_inputs.append(edit)

        layout.addLayout(form)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def row_values(self):
        distance = float(self.distance_input.text())
        values = [float(edit.text()) for edit in self._value_inputs]
        return distance, values


class MapChartView(QChartView):
    def __init__(self, chart_window):
        super().__init__()
        self.chart_window = chart_window
        self.setMouseTracking(True)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)

    def mouseMoveEvent(self, event):
        self.chart_window.update_hover_readout(self.mapToScene(event.position().toPoint()))
        super().mouseMoveEvent(event)

    def leaveEvent(self, event):
        self.chart_window.clear_hover_readout()
        super().leaveEvent(event)


class MapChartWindow(QDialog):
    def __init__(self, tuner_window):
        super().__init__(tuner_window)
        self.tuner_window = tuner_window
        self.setWindowTitle("Map Visualization")
        self.resize(920, 560)
        self.series_checks = {}
        self.last_series_names = []
        self.visible_points = {}

        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(12, 12, 12, 12)
        root_layout.setSpacing(10)

        top_layout = QHBoxLayout()
        top_layout.addWidget(QLabel("Series"))
        self.series_toggle_layout = QHBoxLayout()
        self.series_toggle_layout.setSpacing(8)
        top_layout.addLayout(self.series_toggle_layout)
        top_layout.addStretch(1)

        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh)
        top_layout.addWidget(self.refresh_button)
        root_layout.addLayout(top_layout)

        self.chart = QChart()
        self.chart.setBackgroundBrush(QColor("#151922"))
        self.chart.setPlotAreaBackgroundBrush(QColor("#10131a"))
        self.chart.setPlotAreaBackgroundVisible(True)
        self.chart.legend().setVisible(True)
        self.chart.legend().setLabelColor(QColor("#e6e8ee"))

        self.chart_view = MapChartView(self)
        self.chart_view.setChart(self.chart)
        root_layout.addWidget(self.chart_view, 1)

        self.hover_label = QLabel("Move over the chart to inspect interpolated values.")
        self.hover_label.setObjectName("Detail")
        root_layout.addWidget(self.hover_label)

    def refresh(self):
        data = self.tuner_window.numeric_chart_data()
        series = data.get("series", [])
        series_names = [item["name"] for item in series]
        if series_names != self.last_series_names:
            self.rebuild_series_toggles(series_names)

        self.chart.removeAllSeries()
        for axis in self.chart.axes():
            self.chart.removeAxis(axis)
        self.visible_points = {}

        selected_series = [
            item for item in series
            if self.series_checks.get(item["name"]) and self.series_checks[item["name"]].isChecked()
        ]
        if not selected_series:
            self.configure_empty_chart(data.get("key_name", "Distance"), "Value")
            self.hover_label.setText("Select at least one series to visualize.")
            return

        x_values = [point[0] for item in selected_series for point in item["points"]]
        y_values = [point[1] for item in selected_series for point in item["points"]]
        if not x_values or not y_values:
            self.configure_empty_chart(data.get("key_name", "Distance"), "Value")
            self.hover_label.setText("No numeric map points are available.")
            return

        axis_x = self.value_axis(data.get("key_name", "Distance"), min(x_values), max(x_values))
        axis_y = self.value_axis("Value", min(y_values), max(y_values))
        self.chart.addAxis(axis_x, Qt.AlignmentFlag.AlignBottom)
        self.chart.addAxis(axis_y, Qt.AlignmentFlag.AlignLeft)

        for index, item in enumerate(selected_series):
            color = QColor(CHART_COLORS[index % len(CHART_COLORS)])
            points = item["points"]
            self.visible_points[item["name"]] = points

            line_series = QLineSeries()
            line_series.setName(item["name"])
            line_series.setPen(QPen(color, 2))
            for x_value, y_value in points:
                line_series.append(QPointF(x_value, y_value))
            self.chart.addSeries(line_series)
            line_series.attachAxis(axis_x)
            line_series.attachAxis(axis_y)

            point_series = QScatterSeries()
            point_series.setName(f"{item['name']} points")
            point_series.setColor(color)
            point_series.setBorderColor(QColor("#f3f5f8"))
            point_series.setMarkerSize(9.0)
            for x_value, y_value in points:
                point_series.append(QPointF(x_value, y_value))
            point_series.hovered.connect(
                lambda point, state, name=item["name"]: self.update_point_hover(name, point, state)
            )
            self.chart.addSeries(point_series)
            point_series.attachAxis(axis_x)
            point_series.attachAxis(axis_y)

        self.hover_label.setText("Move over the chart to inspect interpolated values.")

    def rebuild_series_toggles(self, series_names):
        existing_state = {
            name: checkbox.isChecked()
            for name, checkbox in self.series_checks.items()
        }
        while self.series_toggle_layout.count():
            item = self.series_toggle_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

        self.series_checks = {}
        self.last_series_names = list(series_names)
        for name in series_names:
            checkbox = QCheckBox(name)
            checkbox.setChecked(existing_state.get(name, True))
            checkbox.stateChanged.connect(lambda _state: self.refresh())
            self.series_checks[name] = checkbox
            self.series_toggle_layout.addWidget(checkbox)

    def configure_empty_chart(self, x_title, y_title):
        self.chart.removeAllSeries()
        for axis in self.chart.axes():
            self.chart.removeAxis(axis)
        axis_x = self.value_axis(x_title, 0.0, 1.0)
        axis_y = self.value_axis(y_title, 0.0, 1.0)
        self.chart.addAxis(axis_x, Qt.AlignmentFlag.AlignBottom)
        self.chart.addAxis(axis_y, Qt.AlignmentFlag.AlignLeft)

    def value_axis(self, title, minimum, maximum):
        axis = QValueAxis()
        axis.setTitleText(title)
        axis.setLabelsColor(QColor("#d9dee8"))
        axis.setTitleBrush(QColor("#e6e8ee"))
        axis.setGridLineColor(QColor("#303848"))
        low, high = self.padded_range(minimum, maximum)
        axis.setRange(low, high)
        axis.setTickCount(6)
        return axis

    def padded_range(self, minimum, maximum):
        if not math.isfinite(minimum) or not math.isfinite(maximum):
            return 0.0, 1.0
        if minimum == maximum:
            pad = max(abs(minimum) * 0.1, 1.0)
            return minimum - pad, maximum + pad
        pad = (maximum - minimum) * 0.08
        return minimum - pad, maximum + pad

    def update_hover_readout(self, chart_position):
        if not self.visible_points:
            return
        plot_area = self.chart.plotArea()
        if not plot_area.contains(chart_position):
            self.clear_hover_readout()
            return

        value_point = self.chart.mapToValue(chart_position)
        x_value = value_point.x()
        parts = [f"{self.tuner_window.key_column_name} {x_value:.3f}"]
        for name, points in self.visible_points.items():
            evaluated = self.interpolate(points, x_value)
            nearest = self.nearest_point(points, x_value)
            if evaluated is None or nearest is None:
                continue
            parts.append(
                f"{name}: eval {evaluated:.3f}, nearest ({nearest[0]:.3f}, {nearest[1]:.3f})"
            )
        self.hover_label.setText(" | ".join(parts))

    def update_point_hover(self, name, point, state):
        if state:
            self.hover_label.setText(f"{name} point: ({point.x():.3f}, {point.y():.3f})")

    def clear_hover_readout(self):
        self.hover_label.setText("Move over the chart to inspect interpolated values.")

    def interpolate(self, points, x_value):
        if not points:
            return None
        if x_value <= points[0][0]:
            return points[0][1]
        if x_value >= points[-1][0]:
            return points[-1][1]
        for index in range(1, len(points)):
            left_x, left_y = points[index - 1]
            right_x, right_y = points[index]
            if left_x <= x_value <= right_x:
                if right_x == left_x:
                    return right_y
                t = (x_value - left_x) / (right_x - left_x)
                return left_y + (right_y - left_y) * t
        return points[-1][1]

    def nearest_point(self, points, x_value):
        if not points:
            return None
        return min(points, key=lambda point: abs(point[0] - x_value))


class MapTunerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Launch Map Tuner")
        self.resize(900, 680)

        self.nt_inst = ntcore.NetworkTableInstance.getDefault()
        self.points_table = None
        self.settings = self.load_settings()
        self.nt_path = self.settings.get("nt_path", FALLBACK_NT_PATH)
        self.connected = False
        self.key_column_name = "Distance"
        self.row_key_kind = "double"
        self.param_names = []
        self.column_specs = []
        self.publishers = {}
        self.chart_window = None
        self.state = CellDelegateState()
        self.csv_dir = self.settings.get("csv_dir", os.path.dirname(self.settings.get("csv_path", os.path.join(LOCAL_CSV_DIR, "Points.csv"))))
        self.local_csv_path = self.derived_csv_path(self.nt_path)

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.publish_if_enabled)
        self.publish_timer.start(PUBLISH_PERIOD_MS)

        self.connection_timer = QTimer(self)
        self.connection_timer.timeout.connect(self.update_connection_state)
        self.connection_timer.start(500)

        self.setup_actions()
        self.setup_ui()
        self.apply_style()

    def setup_actions(self):
        reload_action = QAction("Reload From Robot", self)
        reload_action.triggered.connect(self.reload_from_robot)
        self.addAction(reload_action)

    def load_settings(self):
        try:
            with open(SETTINGS_PATH, "r", encoding="utf-8") as settings_file:
                settings = json.load(settings_file)
                return settings if isinstance(settings, dict) else {}
        except (OSError, json.JSONDecodeError):
            return {}

    def save_settings(self):
        self.settings["server"] = self.server_input.text().strip()
        self.settings["nt_path"] = self.normalize_nt_path(self.path_input.text())
        self.settings["csv_dir"] = self.csv_dir
        try:
            with open(SETTINGS_PATH, "w", encoding="utf-8") as settings_file:
                json.dump(self.settings, settings_file, indent=2)
                settings_file.write("\n")
        except OSError as exc:
            self.detail_label.setText(f"Could not save GUI settings: {exc}")

    def derived_csv_name(self, nt_path):
        path = self.normalize_nt_path(nt_path).strip("/")
        if not path:
            return "map.csv"
        safe = path.replace("/", "_").replace("\\", "_")
        return f"{safe}.csv"

    def derived_csv_path(self, nt_path):
        return os.path.abspath(os.path.join(self.csv_dir, self.derived_csv_name(nt_path)))

    def refresh_derived_csv_path(self):
        self.local_csv_path = self.derived_csv_path(self.nt_path)
        self.csv_path_input.setText(self.local_csv_path)

    def apply_nt_path_input(self):
        self.nt_path = self.normalize_nt_path(self.path_input.text())
        self.path_input.setText(self.nt_path)
        self.refresh_derived_csv_path()
        self.save_settings()
        self.detail_label.setText(f"CSV name derived from NT path: {os.path.basename(self.local_csv_path)}")

    def apply_csv_dir_input(self):
        path = self.csv_dir_input.text().strip() or LOCAL_CSV_DIR
        if not os.path.isabs(path):
            path = os.path.abspath(os.path.join(REPO_ROOT, path))
        self.csv_dir = path
        self.csv_dir_input.setText(path)
        self.refresh_derived_csv_path()
        self.save_settings()
        deploy_root = os.path.abspath(os.path.join(REPO_ROOT, "src", "main", "deploy"))
        try:
            in_deploy = os.path.commonpath([deploy_root, os.path.abspath(path)]) == deploy_root
        except ValueError:
            in_deploy = False
        if not in_deploy:
            self.detail_label.setText("Hint: CSV files should usually live under src/main/deploy so Gradle deploys them.")
        else:
            self.detail_label.setText(f"Local CSV: {self.local_csv_path}")

    def backup_csv_path(self):
        backup_dir = os.path.join(tempfile.gettempdir(), "frc122_tune_map_backups")
        csv_name = os.path.basename(self.local_csv_path) or "map.csv"
        base, ext = os.path.splitext(csv_name)
        path_hash = hashlib.sha1(os.path.abspath(self.local_csv_path).encode("utf-8")).hexdigest()[:10]
        return os.path.join(backup_dir, f"{base}.{path_hash}.backup{ext or '.csv'}")

    def backup_exists(self):
        return os.path.exists(self.backup_csv_path())

    def browse_csv_path(self):
        start_dir = self.csv_dir_input.text().strip() or LOCAL_CSV_DIR
        if not os.path.isabs(start_dir):
            start_dir = os.path.abspath(os.path.join(REPO_ROOT, start_dir))
        selected = QFileDialog.getExistingDirectory(
            self,
            "Choose CSV Folder",
            start_dir if os.path.exists(start_dir) else LOCAL_CSV_DIR,
        )
        if selected:
            self.csv_dir_input.setText(selected)
            self.apply_csv_dir_input()

    def setup_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(14, 14, 14, 14)
        root_layout.setSpacing(12)

        connection_frame = QFrame()
        connection_frame.setObjectName("Panel")
        connection_layout = QGridLayout(connection_frame)
        connection_layout.setContentsMargins(12, 12, 12, 12)
        connection_layout.setHorizontalSpacing(10)
        connection_layout.setVerticalSpacing(8)

        self.server_input = QLineEdit(self.settings.get("server", FALLBACK_SERVER))
        self.server_input.setPlaceholderText("10.1.22.2, 127.0.0.1, or roboRIO name")
        self.path_input = QLineEdit(self.nt_path)
        self.path_input.editingFinished.connect(self.apply_nt_path_input)
        self.csv_dir_input = QLineEdit(self.csv_dir)
        self.csv_dir_input.setPlaceholderText("Recommended: src/main/deploy")
        self.csv_dir_input.editingFinished.connect(self.apply_csv_dir_input)
        self.csv_path_input = QLineEdit(self.local_csv_path)
        self.csv_path_input.setReadOnly(True)
        self.browse_csv_button = QPushButton("Browse")
        self.browse_csv_button.clicked.connect(self.browse_csv_path)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_and_load)
        self.reload_button = QPushButton("Reload")
        self.reload_button.clicked.connect(self.reload_from_robot)
        self.reload_button.setEnabled(False)

        self.auto_publish_checkbox = QCheckBox("Live publish table")
        self.auto_publish_checkbox.setChecked(True)
        self.auto_publish_checkbox.setEnabled(False)

        self.status_label = QLabel("Disconnected")
        self.status_label.setObjectName("StatusBad")

        connection_layout.addWidget(QLabel("Server"), 0, 0)
        connection_layout.addWidget(self.server_input, 0, 1)
        connection_layout.addWidget(QLabel("NT Path"), 0, 2)
        connection_layout.addWidget(self.path_input, 0, 3)
        connection_layout.addWidget(self.connect_button, 0, 4)
        connection_layout.addWidget(self.reload_button, 0, 5)
        connection_layout.addWidget(QLabel("CSV Folder"), 1, 0)
        connection_layout.addWidget(self.csv_dir_input, 1, 1, 1, 3)
        connection_layout.addWidget(self.browse_csv_button, 1, 4)
        connection_layout.addWidget(QLabel("Derived CSV"), 2, 0)
        connection_layout.addWidget(self.csv_path_input, 2, 1, 1, 3)
        connection_layout.addWidget(self.auto_publish_checkbox, 3, 1)
        connection_layout.addWidget(self.status_label, 3, 3, 1, 3)
        connection_layout.setColumnStretch(3, 1)
        root_layout.addWidget(connection_frame)

        toolbar = QHBoxLayout()
        self.add_row_button = QPushButton("Add Row")
        self.add_row_button.clicked.connect(self.add_row_dialog)
        self.add_row_button.setEnabled(False)
        self.delete_row_button = QPushButton("Delete Selected")
        self.delete_row_button.clicked.connect(self.delete_selected_rows)
        self.delete_row_button.setEnabled(False)
        self.sync_button = QPushButton("Publish Now")
        self.sync_button.clicked.connect(self.publish_now)
        self.sync_button.setEnabled(False)
        self.visualize_button = QPushButton("Visualize")
        self.visualize_button.clicked.connect(self.open_chart_window)
        self.visualize_button.setEnabled(False)
        self.restore_backup_button = QPushButton("Restore Backup")
        self.restore_backup_button.clicked.connect(self.restore_backup)

        self.nudge_decrease_button = QPushButton("-")
        self.nudge_decrease_button.setObjectName("StepButton")
        self.nudge_decrease_button.clicked.connect(lambda: self.change_increment_scale(-1))
        self.nudge_decrease_button.setEnabled(False)
        self.nudge_increase_button = QPushButton("+")
        self.nudge_increase_button.setObjectName("StepButton")
        self.nudge_increase_button.clicked.connect(lambda: self.change_increment_scale(1))
        self.nudge_increase_button.setEnabled(False)
        self.increment_input = QLineEdit("0.1")
        self.increment_input.setObjectName("ShortInput")
        self.increment_input.setAlignment(Qt.AlignCenter)
        self.increment_input.editingFinished.connect(self.apply_increment_input)

        self.decimals_value = 2
        self.decimals_label = QLabel(str(self.decimals_value))
        self.decimals_label.setObjectName("DecimalsValue")
        self.decimals_decrease_button = QPushButton("-")
        self.decimals_decrease_button.setObjectName("StepButton")
        self.decimals_decrease_button.clicked.connect(lambda: self.change_decimals(-1))
        self.decimals_increase_button = QPushButton("+")
        self.decimals_increase_button.setObjectName("StepButton")
        self.decimals_increase_button.clicked.connect(lambda: self.change_decimals(1))

        toolbar.addWidget(self.add_row_button)
        toolbar.addWidget(self.delete_row_button)
        toolbar.addWidget(self.sync_button)
        toolbar.addWidget(self.visualize_button)
        toolbar.addWidget(self.restore_backup_button)
        toolbar.addWidget(QLabel("Nudge"))
        toolbar.addWidget(self.nudge_decrease_button)
        toolbar.addWidget(self.increment_input)
        toolbar.addWidget(self.nudge_increase_button)
        toolbar.addStretch(1)
        toolbar.addWidget(QLabel("Decimals"))
        toolbar.addWidget(self.decimals_decrease_button)
        toolbar.addWidget(self.decimals_label)
        toolbar.addWidget(self.decimals_increase_button)
        root_layout.addLayout(toolbar)

        self.table = QTableWidget(0, len(self.param_names) + 1)
        self.table.setHorizontalHeaderLabels(["Distance", *self.param_names])
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectItems)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.verticalHeader().setDefaultSectionSize(38)
        self.configure_table_column_widths()
        root_layout.addWidget(self.table, 1)

        footer = QHBoxLayout()
        self.detail_label = QLabel(f"Local CSV: {self.local_csv_path}")
        self.detail_label.setObjectName("Detail")
        self.last_publish_label = QLabel("")
        self.last_publish_label.setObjectName("Detail")
        footer.addWidget(self.detail_label, 1)
        footer.addWidget(self.last_publish_label)
        root_layout.addLayout(footer)

    def apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #111318;
                color: #e6e8ee;
                font-family: Segoe UI, Arial;
                font-size: 10pt;
            }
            QFrame#Panel {
                background: #181b22;
                border: 1px solid #2c3340;
                border-radius: 6px;
            }
            QLineEdit, QDoubleSpinBox {
                background: #0d0f14;
                color: #f3f5f8;
                border: 1px solid #3a4352;
                border-radius: 4px;
                padding: 5px 7px;
                min-height: 20px;
                selection-background-color: #2f6fed;
            }
            QLineEdit:focus, QDoubleSpinBox:focus {
                border-color: #4f8cff;
            }
            QDoubleSpinBox {
                padding-right: 24px;
                min-height: 24px;
            }
            QDoubleSpinBox::up-button,
            QDoubleSpinBox::down-button {
                width: 24px;
                background: #202531;
                border-left: 1px solid #3a4352;
            }
            QDoubleSpinBox::up-button:hover,
            QDoubleSpinBox::down-button:hover {
                background: #2a3140;
            }
            QPushButton {
                background: #202531;
                color: #eef2f7;
                border: 1px solid #465267;
                border-radius: 4px;
                padding: 6px 10px;
                min-width: 82px;
            }
            QPushButton:hover {
                background: #2a3140;
                border-color: #6ea0ff;
            }
            QPushButton:pressed {
                background: #182848;
            }
            QPushButton:disabled {
                color: #6e7582;
                background: #171a20;
                border-color: #2a303b;
            }
            QPushButton#StepButton {
                min-width: 34px;
                max-width: 34px;
                padding: 6px 0;
                font-size: 12pt;
                font-weight: 700;
            }
            QLineEdit#ShortInput {
                min-width: 54px;
                max-width: 54px;
            }
            QLabel#DecimalsValue {
                background: #0d0f14;
                border: 1px solid #3a4352;
                border-radius: 4px;
                min-width: 34px;
                padding: 6px 8px;
                qproperty-alignment: AlignCenter;
                font-weight: 600;
            }
            QCheckBox {
                spacing: 8px;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
                border: 1px solid #4a5568;
                border-radius: 3px;
                background: #0d0f14;
            }
            QCheckBox::indicator:checked {
                background: #4f8cff;
                border-color: #7fb0ff;
            }
            QTableWidget {
                background: #151922;
                color: #eef2f7;
                alternate-background-color: #1b202b;
                gridline-color: #303848;
                border: 1px solid #323b4d;
                selection-background-color: #1f4f9c;
                selection-color: #ffffff;
            }
            QHeaderView::section {
                background: #222938;
                color: #f4f7fb;
                border: 0;
                border-right: 1px solid #323b4d;
                border-bottom: 1px solid #323b4d;
                padding: 7px;
                font-weight: 600;
            }
            QScrollBar:vertical {
                background: #111318;
                width: 12px;
                margin: 0;
            }
            QScrollBar::handle:vertical {
                background: #394457;
                border-radius: 5px;
                min-height: 24px;
            }
            QScrollBar::handle:vertical:hover {
                background: #4a5870;
            }
            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical {
                height: 0;
            }
            QLabel#StatusGood {
                color: #48d17a;
                font-weight: 600;
            }
            QLabel#StatusWarn {
                color: #f3c64e;
                font-weight: 600;
            }
            QLabel#StatusBad {
                color: #ff6b63;
                font-weight: 600;
            }
            QLabel#Detail {
                color: #aab4c3;
            }
            """
        )

    def set_status(self, text, kind):
        self.status_label.setText(text)
        self.status_label.setObjectName(f"Status{kind}")
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)

    def connect_and_load(self):
        server = self.server_input.text().strip()
        path = self.normalize_nt_path(self.path_input.text())
        if not server:
            QMessageBox.warning(self, "Missing Server", "Enter a robot IP, roboRIO name, or localhost.")
            return

        self.set_status("Connecting...", "Warn")
        QApplication.processEvents()

        if self.connected:
            self.nt_inst.stopClient()
            self.connected = False

        self.nt_inst.setServer(server)
        self.nt_inst.startClient4("LaunchMapTunerQt")

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            QApplication.processEvents()
            if self.nt_inst.isConnected():
                break
            time.sleep(0.05)

        if not self.nt_inst.isConnected():
            self.points_table = None
            self.set_status(f"No robot at {server}", "Bad")
            self.detail_label.setText("Connection timed out.")
            return

        self.connected = True
        self.nt_path = path
        self.path_input.setText(path)
        self.refresh_derived_csv_path()
        self.points_table = self.nt_inst.getTable(self.nt_path)
        self.save_settings()
        self.enable_workflow(True)
        self.set_status(f"Connected to {server}", "Good")
        if self.load_from_nt():
            self.save_connection_backup()

    def enable_workflow(self, enabled):
        self.reload_button.setEnabled(enabled)
        self.add_row_button.setEnabled(enabled)
        self.delete_row_button.setEnabled(enabled)
        self.sync_button.setEnabled(enabled)
        self.visualize_button.setEnabled(enabled and self.supports_numeric_table())
        self.nudge_decrease_button.setEnabled(enabled)
        self.nudge_increase_button.setEnabled(enabled)
        self.auto_publish_checkbox.setEnabled(enabled)

    def update_connection_state(self):
        if self.connected and not self.nt_inst.isConnected():
            self.connected = False
            self.set_status("Disconnected", "Bad")
            self.detail_label.setText("Robot disconnected. Edits remain in the table.")
        elif not self.connected and self.nt_inst.isConnected():
            self.connected = True
            self.set_status("Connected", "Good")

    def reload_from_robot(self):
        if not self.points_table:
            return
        if self.state.dirty:
            answer = QMessageBox.question(
                self,
                "Reload From Robot",
                "Discard unsaved table edits and reload the robot's current map?",
            )
            if answer != QMessageBox.StandardButton.Yes:
                return
        self.load_from_nt()

    def restore_backup(self):
        backup_path = self.backup_csv_path()
        rows, param_names = self.load_csv_rows(backup_path)
        if not rows:
            QMessageBox.warning(self, "No Backup", f"No backup values were found at:\n{backup_path}")
            return

        if self.state.dirty:
            answer = QMessageBox.question(
                self,
                "Restore Backup",
                "Discard current edits and restore the backup values?",
            )
            if answer != QMessageBox.StandardButton.Yes:
                return

        self.load_rows_into_table(rows, param_names)
        self.state.dirty = True
        if self.connected and self.points_table:
            self.publish_now(show_success=False)
            self.detail_label.setText(f"Restored backup and published values from {backup_path}.")
        else:
            self.detail_label.setText(f"Restored backup from {backup_path}. Connect to publish it.")

    def load_rows_into_table(self, rows, param_names):
        self.state.loading = True
        self.table.clearContents()
        self.table.setRowCount(0)
        self.row_key_kind = "double"
        self.key_column_name = "Distance"
        self.param_names = param_names
        self.column_specs = [{"name": name, "kind": "double"} for name in param_names]
        self.configure_columns()
        for distance, values in rows:
            self.add_row(distance, values)
        self.state.loading = False
        self.state.invalid_cells.clear()
        self.update_invalid_state()
        self.refresh_chart_window()

    def load_from_nt(self):
        self.state.loading = True
        self.publishers.clear()
        self.table.clearContents()
        self.table.setRowCount(0)

        row_keys = self.wait_for_nt_rows()
        if not row_keys:
            csv_rows, csv_param_names = self.load_csv_rows(self.local_csv_path)
            if csv_rows:
                self.load_rows_into_table(csv_rows, csv_param_names)
                self.state.dirty = False
                self.detail_label.setText(f"Loaded {len(csv_rows)} rows from {self.local_csv_path}.")
                self.set_status(f"Loaded {len(csv_rows)} CSV rows", "Good")
                self.publish_table(show_errors=False)
                return True

        if not row_keys:
            self.key_column_name = "State"
            self.row_key_kind = "string"
            self.param_names = []
            self.column_specs = []
            self.configure_columns()
            self.state.loading = False
            self.set_status("No map rows", "Warn")
            self.detail_label.setText(f"No rows found under {self.nt_path} or {self.local_csv_path}.")
            self.update_invalid_state()
            self.refresh_chart_window()
            return False

        self.row_key_kind = self.infer_row_key_kind(row_keys)
        self.key_column_name = self.infer_key_column_name(self.row_key_kind)
        self.column_specs = self.discover_column_specs(row_keys[0])
        self.param_names = [spec["name"] for spec in self.column_specs]
        self.configure_columns()

        rows_to_load = []
        subscribers = []
        for key in row_keys:
            row_table = self.points_table.getSubTable(key)
            row_subscribers = []
            for spec in self.column_specs:
                param_name = spec["name"]
                kind = spec["kind"]
                options = ntcore.PubSubOptions()
                options.sendAll = True
                subscriber = self.subscribe_for_kind(row_table, param_name, kind, options)
                row_subscribers.append(subscriber)
                subscribers.append(subscriber)
            rows_to_load.append((key, row_subscribers))

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            QApplication.processEvents()
            if subscribers and any(sub.getLastChange() != 0 for sub in subscribers):
                break
            self.nt_inst.flush()
            time.sleep(0.05)

        loaded = 0
        for row_key, row_subscribers in rows_to_load:
            values = []
            saw_value = False
            for subscriber in row_subscribers:
                value = subscriber.get()
                saw_value = saw_value or subscriber.getLastChange() != 0 or subscriber.exists()
                values.append(value)
            if saw_value:
                self.add_row(row_key, values)
                loaded += 1

        self.state.loading = False
        self.state.dirty = False
        self.state.invalid_cells.clear()
        self.update_invalid_state()
        self.refresh_chart_window()

        if loaded:
            self.save_local_csv()

        self.detail_label.setText(f"Loaded {loaded} rows from NetworkTables at {self.nt_path}.")
        if loaded:
            self.set_status(f"Loaded {loaded} rows", "Good")
        else:
            self.set_status("No published values", "Warn")
        return loaded > 0

    def infer_row_key_kind(self, row_keys):
        keys = [str(key) for key in row_keys if str(key)]
        if keys and all(self.is_bool(key) for key in keys):
            return "bool"
        if keys and all(self.is_int(key) for key in keys):
            return "int"
        if keys and all(self.is_float(key) for key in keys):
            return "double"
        return "string"

    def infer_key_column_name(self, kind):
        if kind == "double":
            return "Distance"
        if kind == "int":
            return "Index"
        if kind == "bool":
            return "Active"
        return "State"

    def topic_kind_from_string(self, type_str):
        lowered = (type_str or "").lower()
        if "double" in lowered or "float" in lowered:
            return "double"
        if "integer" in lowered or "int" in lowered:
            return "int"
        if "boolean" in lowered or lowered == "bool":
            return "bool"
        if "string" in lowered:
            return "string"
        return "unknown"

    def topic_kind_from_info(self, info):
        return self.topic_kind_from_string(getattr(info, "type_str", ""))

    def topic_for_kind(self, row_table, param_name, kind):
        if kind == "int":
            return row_table.getIntegerTopic(param_name)
        if kind == "bool":
            return row_table.getBooleanTopic(param_name)
        if kind == "string":
            return row_table.getStringTopic(param_name)
        return row_table.getDoubleTopic(param_name)

    def subscribe_for_kind(self, row_table, param_name, kind, options):
        if kind == "int":
            return row_table.getIntegerTopic(param_name).subscribe(0, options)
        if kind == "bool":
            return row_table.getBooleanTopic(param_name).subscribe(False, options)
        if kind == "string":
            return row_table.getStringTopic(param_name).subscribe("", options)
        return row_table.getDoubleTopic(param_name).subscribe(float("nan"), options)

    def coerce_numeric_value(self, value):
        if isinstance(value, bool):
            return 1.0 if value else 0.0
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return float(value)
        if self.is_float(value):
            return float(value)
        return 0.0

    def coerce_bool_value(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return value != 0
        text = str(value).strip().lower()
        return text in {"true", "1", "yes", "on"}

    def coerce_text_value(self, value):
        if value is None:
            return ""
        return str(value)

    def coerce_publish_value(self, value, kind):
        if kind == "int":
            return int(round(self.coerce_numeric_value(value)))
        if kind == "bool":
            return self.coerce_bool_value(value)
        if kind == "string":
            return self.coerce_text_value(value)
        return float(self.coerce_numeric_value(value))

    def format_row_key(self, value, kind):
        if kind == "int":
            return str(int(round(self.coerce_numeric_value(value))))
        if kind == "bool":
            return "true" if self.coerce_bool_value(value) else "false"
        if kind == "string":
            return self.coerce_text_value(value)
        return f"{self.coerce_numeric_value(value):.1f}"

    def sort_key(self, value, kind):
        if kind == "int" or kind == "double":
            return self.coerce_numeric_value(value)
        if kind == "bool":
            return 1 if self.coerce_bool_value(value) else 0
        return self.coerce_text_value(value).lower()

    def is_int(self, value):
        try:
            text = str(value).strip()
            if text.startswith(("+", "-")):
                text = text[1:]
            return text.isdigit()
        except (TypeError, ValueError):
            return False

    def is_bool(self, value):
        text = str(value).strip().lower()
        return text in {"true", "false", "1", "0", "yes", "no", "on", "off"}

    def configure_columns(self):
        with QSignalBlocker(self.table):
            self.table.setColumnCount(self.data_column_count())
            self.table.setHorizontalHeaderLabels([self.key_column_name, *self.param_names])
            self.configure_table_column_widths()

    def data_column_count(self):
        return len(self.param_names) + 1

    def configure_table_column_widths(self):
        header = self.table.horizontalHeader()
        for column in range(self.data_column_count()):
            header.setSectionResizeMode(column, QHeaderView.ResizeMode.Stretch)

    def add_row(self, distance, values):
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.set_cell_widget(row, 0, distance, self.row_key_kind, 1)
        for column, value in enumerate(values, start=1):
            spec = self.column_specs[column - 1] if column - 1 < len(self.column_specs) else {"kind": "double"}
            self.set_cell_widget(row, column, value, spec["kind"], self.decimals_value)

    def set_cell_widget(self, row, column, value, kind, decimals):
        if kind in ("double", "int"):
            spinbox = QDoubleSpinBox()
            spinbox.setRange(0.0 if column == 0 and kind != "string" else -10000.0, 10000.0)
            spinbox.setDecimals(1 if column == 0 and kind == "double" else (0 if kind == "int" else decimals))
            spinbox.setSingleStep(self.current_increment(default=0.1))
            spinbox.setValue(self.coerce_numeric_value(value))
            spinbox.setAlignment(Qt.AlignCenter)
            spinbox.setKeyboardTracking(False)
            spinbox.valueChanged.connect(lambda _value, widget=spinbox, col=column: self.on_spinbox_changed(widget, col))
            self.table.setCellWidget(row, column, spinbox)
            return

        if kind == "bool":
            checkbox = QCheckBox()
            checkbox.setChecked(self.coerce_bool_value(value))
            checkbox.setTristate(False)
            checkbox.stateChanged.connect(lambda _state, widget=checkbox, col=column: self.on_widget_changed(widget, col))
            self.table.setCellWidget(row, column, checkbox)
            return

        edit = QLineEdit(self.coerce_text_value(value))
        edit.setAlignment(Qt.AlignCenter)
        edit.setPlaceholderText("text")
        edit.editingFinished.connect(lambda widget=edit, col=column: self.on_widget_changed(widget, col))
        self.table.setCellWidget(row, column, edit)

    def cell_spinbox(self, row, column):
        widget = self.table.cellWidget(row, column)
        return widget if isinstance(widget, QDoubleSpinBox) else None

    def cell_widget(self, row, column):
        return self.table.cellWidget(row, column)

    def cell_value(self, row, column):
        widget = self.cell_widget(row, column)
        if isinstance(widget, QDoubleSpinBox):
            return widget.value()
        if isinstance(widget, QCheckBox):
            return widget.isChecked()
        if isinstance(widget, QLineEdit):
            return widget.text()
        return 0.0

    def add_row_dialog(self):
        if not self.supports_numeric_table():
            QMessageBox.information(self, "Unsupported Table", "Add Row is only available for numeric tables.")
            return
        next_distance = self.next_distance()
        dialog = AddRowDialog(self.param_names, next_distance, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        try:
            distance, values = dialog.row_values()
        except ValueError:
            QMessageBox.warning(self, "Invalid Row", "All row values must be numeric.")
            return
        self.add_row(distance, values)
        self.sort_rows()
        self.state.dirty = True
        self.publish_now(show_success=False)
        self.refresh_chart_window()

    def delete_selected_rows(self):
        selected = sorted({index.row() for index in self.table.selectedIndexes()}, reverse=True)
        if not selected:
            return
        for row in selected:
            self.table.removeRow(row)
        self.state.dirty = True
        self.publish_now(show_success=False)
        self.update_invalid_state()
        self.refresh_chart_window()
        self.detail_label.setText(f"Deleted {len(selected)} row(s).")

    def current_increment(self, default=None):
        try:
            increment = abs(float(self.increment_input.text()))
        except ValueError:
            if default is not None:
                return default
            self.increment_input.setStyleSheet("border-color: #ff6b63;")
            self.detail_label.setText("Increment must be numeric.")
            return None
        if increment == 0.0:
            if default is not None:
                return default
            self.increment_input.setStyleSheet("border-color: #ff6b63;")
            self.detail_label.setText("Increment must be greater than zero.")
            return None
        self.increment_input.setStyleSheet("")
        return increment

    def apply_increment_input(self):
        increment = self.current_increment()
        if increment is None:
            return
        self.increment_input.setText(f"{increment:g}")
        self.apply_increment_to_spinboxes(increment)
        self.detail_label.setText(f"Spinbox increment set to {increment:g}.")

    def change_increment_scale(self, direction):
        increment = self.current_increment(default=0.1)
        scaled = increment * (10.0 if direction > 0 else 0.1)
        scaled = max(0.0001, min(100.0, scaled))
        self.increment_input.setText(f"{scaled:g}")
        self.apply_increment_input()

    def apply_increment_to_spinboxes(self, increment):
        for row in range(self.table.rowCount()):
            for column in range(self.data_column_count()):
                spinbox = self.cell_spinbox(row, column)
                if spinbox:
                    spinbox.setSingleStep(increment)

    def on_spinbox_changed(self, widget, column):
        self.on_widget_changed(widget, column)

    def on_widget_changed(self, widget, column):
        if self.state.loading:
            return
        self.state.dirty = True
        self.update_invalid_state()
        if column == 0:
            self.sort_rows()
        self.publish_now(show_success=False)
        self.refresh_chart_window()

    def next_distance(self):
        distances = []
        for row in range(self.table.rowCount()):
            spinbox = self.cell_spinbox(row, 0)
            if spinbox:
                distances.append(spinbox.value())
        return max(distances, default=0.5) + 0.5

    def change_decimals(self, delta):
        self.decimals_value = max(0, min(4, self.decimals_value + delta))
        self.decimals_label.setText(str(self.decimals_value))
        self.decimals_decrease_button.setEnabled(self.decimals_value > 0)
        self.decimals_increase_button.setEnabled(self.decimals_value < 4)
        self.reformat_table()
        self.refresh_chart_window()

    def reformat_table(self):
        if self.state.loading:
            return
        decimals = self.decimals_value
        for row in range(self.table.rowCount()):
            for column in range(self.data_column_count()):
                spinbox = self.cell_spinbox(row, column)
                if spinbox:
                    if column == 0:
                        spinbox.setDecimals(1 if self.row_key_kind == "double" else 0)
                    else:
                        spec = self.column_specs[column - 1] if column - 1 < len(self.column_specs) else {"kind": "double"}
                        spinbox.setDecimals(0 if spec["kind"] == "int" else decimals)

    def update_invalid_state(self):
        invalid = bool(self.state.invalid_cells)
        numeric_table = self.supports_numeric_table()
        self.sync_button.setEnabled(self.connected and not invalid)
        self.nudge_decrease_button.setEnabled(self.connected and not invalid and numeric_table)
        self.nudge_increase_button.setEnabled(self.connected and not invalid and numeric_table)
        self.auto_publish_checkbox.setEnabled(self.connected and not invalid)
        self.add_row_button.setEnabled(self.connected and numeric_table)
        self.visualize_button.setEnabled(numeric_table and self.table.rowCount() > 0)
        if invalid:
            self.detail_label.setText("Fix highlighted cells before publishing.")

    def open_chart_window(self):
        if not self.supports_numeric_table() or self.table.rowCount() == 0:
            QMessageBox.information(self, "No Numeric Map", "Load or create a numeric map before visualizing it.")
            return
        if self.chart_window is None:
            self.chart_window = MapChartWindow(self)
        self.chart_window.refresh()
        self.chart_window.show()
        self.chart_window.raise_()
        self.chart_window.activateWindow()

    def refresh_chart_window(self):
        if self.chart_window and self.chart_window.isVisible():
            self.chart_window.refresh()

    def numeric_chart_data(self):
        if not self.supports_numeric_table():
            return {"key_name": self.key_column_name, "series": []}

        rows = []
        for row in range(self.table.rowCount()):
            x_value = self.coerce_numeric_value(self.cell_value(row, 0))
            row_values = []
            for column, spec in enumerate(self.column_specs, start=1):
                if spec["kind"] not in {"double", "int"}:
                    continue
                row_values.append((spec["name"], self.coerce_numeric_value(self.cell_value(row, column))))
            rows.append((x_value, row_values))
        rows.sort(key=lambda item: item[0])

        series = []
        for column_index, spec in enumerate(self.column_specs):
            if spec["kind"] not in {"double", "int"}:
                continue
            points = []
            for x_value, row_values in rows:
                if column_index < len(row_values):
                    points.append((x_value, row_values[column_index][1]))
            series.append({"name": spec["name"], "points": points})
        return {"key_name": self.key_column_name, "series": series}

    def supports_numeric_table(self):
        if not self.column_specs or self.row_key_kind not in {"double", "int"}:
            return False
        return all(spec["kind"] in {"double", "int"} for spec in self.column_specs)

    def sort_rows(self):
        if self.state.loading:
            return
        rows = []
        for row in range(self.table.rowCount()):
            row_values = []
            for column in range(self.data_column_count()):
                row_values.append(self.cell_value(row, column))
            rows.append(row_values)
        rows.sort(key=lambda values: self.sort_key(values[0], self.row_key_kind))

        with QSignalBlocker(self.table):
            self.table.setRowCount(0)
            for values in rows:
                row = self.table.rowCount()
                self.table.insertRow(row)
                self.set_cell_widget(row, 0, values[0], self.row_key_kind, 1)
                for column, value in enumerate(values[1:], start=1):
                    spec = self.column_specs[column - 1] if column - 1 < len(self.column_specs) else {"kind": "double"}
                    self.set_cell_widget(row, column, value, spec["kind"], self.decimals_value)

    def publish_if_enabled(self):
        if self.auto_publish_checkbox.isChecked():
            self.publish_table(show_errors=False)

    def publish_now(self, show_success=True):
        if self.publish_table(show_errors=True):
            if show_success:
                self.detail_label.setText("Published visible table to NetworkTables.")
            return True
        return False

    def publish_table(self, show_errors):
        if not self.connected or not self.points_table or self.table.rowCount() == 0:
            return False
        if self.state.invalid_cells:
            return False

        should_save_csv = self.state.dirty
        try:
            for row in range(self.table.rowCount()):
                row_key = self.format_row_key(self.cell_value(row, 0), self.row_key_kind)
                row_table = self.points_table.getSubTable(row_key)
                for column, spec in enumerate(self.column_specs, start=1):
                    param_name = spec["name"]
                    kind = spec["kind"]
                    value = self.cell_value(row, column)
                    pub_key = (row_key, param_name)
                    if pub_key not in self.publishers:
                        topic = self.topic_for_kind(row_table, param_name, kind)
                        topic.setRetained(True)
                        topic.setPersistent(True)
                        self.publishers[pub_key] = topic.publish()
                    self.publishers[pub_key].set(self.coerce_publish_value(value, kind))
            self.nt_inst.flush()
        except (AttributeError, ValueError) as exc:
            if show_errors:
                QMessageBox.warning(self, "Publish Failed", f"Check the highlighted table values.\n\n{exc}")
            return False

        self.state.dirty = False
        self.last_publish_label.setText(time.strftime("Last publish %H:%M:%S"))
        if should_save_csv:
            self.save_local_csv()
        return True

    def wait_for_nt_rows(self, timeout_sec=3.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            QApplication.processEvents()
            rows = self.discover_row_keys()
            if rows:
                return rows
            self.nt_inst.flush()
            time.sleep(0.05)
        return self.discover_row_keys()

    def discover_row_keys(self):
        row_keys = set()
        if not self.points_table:
            return []

        listener_handle = None
        try:
            def on_subtable(_table, subkey, _subtable):
                normalized = str(subkey).strip("/")
                if normalized and not normalized.startswith("."):
                    row_keys.add(normalized)

            listener_handle = self.points_table.addSubTableListener(on_subtable)
            deadline = time.monotonic() + 0.25
            while time.monotonic() < deadline:
                QApplication.processEvents()
                if row_keys:
                    break
                self.nt_inst.flush()
                time.sleep(0.02)
        finally:
            if listener_handle is not None:
                try:
                    self.points_table.removeListener(listener_handle)
                except Exception:
                    pass

        for subkey in self.points_table.getSubTables():
            normalized = str(subkey).strip("/")
            if normalized and not normalized.startswith("."):
                row_keys.add(normalized)

        for topic in self.points_table.getTopics():
            self.extract_row_key_from_topic_name(
                topic.getName() if hasattr(topic, "getName") else topic,
                row_keys,
            )

        try:
            for topic in self.nt_inst.getTopics(self.nt_path):
                self.extract_row_key_from_topic_name(
                    topic.getName() if hasattr(topic, "getName") else topic,
                    row_keys,
                )
        except TypeError:
            for topic in self.nt_inst.getTopics():
                self.extract_row_key_from_topic_name(
                    topic.getName() if hasattr(topic, "getName") else topic,
                    row_keys,
                )

        inferred_kind = self.infer_row_key_kind(sorted(row_keys))
        return sorted(row_keys, key=lambda key: self.sort_key(key, inferred_kind))

    def extract_row_key_from_topic_name(self, name, row_keys):
        normalized = str(name).strip("/")
        if not normalized:
            return

        prefix = self.nt_path.strip("/")
        if not prefix or not normalized.startswith(prefix + "/"):
            return

        suffix = normalized[len(prefix) + 1 :]
        parts = suffix.split("/")
        if len(parts) >= 2 and parts[0] and not parts[0].startswith("."):
            row_keys.add(parts[0])

    def discover_column_specs(self, row_key):
        specs = []
        row_table = self.points_table.getSubTable(row_key)
        names = [name for name in row_table.getKeys() if name and not str(name).startswith(".")]

        prefix = f"{self.nt_path}/{row_key}".strip("/")
        prefixes = [prefix]
        if self.nt_path.startswith("/"):
            prefixes.append("/" + prefix)

        def visit_topic_info(name, kind_hint):
            normalized = str(name).strip("/")
            if not normalized:
                return
            for base in prefixes:
                base = base.strip("/")
                if not base:
                    continue
                if not normalized.startswith(base + "/"):
                    continue
                suffix = normalized[len(base) + 1 :]
                param_name = suffix.split("/")[0]
                if param_name and not param_name.startswith("."):
                    if param_name not in names:
                        names.append(param_name)
                    if kind_hint:
                        type_by_name[param_name] = kind_hint

        try:
            topic_infos = self.nt_inst.getTopicInfo("/" + prefix)
        except TypeError:
            topic_infos = self.nt_inst.getTopicInfo(prefix)

        type_by_name = {}
        for info in topic_infos:
            visit_topic_info(info.name, self.topic_kind_from_info(info))

        for topic in self.points_table.getTopics():
            topic_name = topic.getName() if hasattr(topic, "getName") else topic
            try:
                kind_hint = self.topic_kind_from_string(topic.getTypeString()) if hasattr(topic, "getTypeString") else None
            except Exception:
                kind_hint = None
            visit_topic_info(topic_name, kind_hint)

        for name in names:
            topic = row_table.getTopic(name)
            kind = self.topic_kind_from_string(topic.getTypeString()) if hasattr(topic, "getTypeString") else type_by_name.get(name, "double")
            if kind == "unknown":
                kind = type_by_name.get(name, "double")
            specs.append({"name": name, "kind": kind})

        return specs

    def load_csv_rows(self, csv_path):
        rows = []
        try:
            with open(csv_path, newline="") as csv_file:
                reader = csv.DictReader(csv_file)
                if not reader.fieldnames or "Distance" not in reader.fieldnames:
                    return [], []

                param_names = [name for name in reader.fieldnames if name != "Distance"]
                if not param_names:
                    return [], []

                for row in reader:
                    distance = row.get("Distance")
                    if distance is None or not self.is_float(distance):
                        continue

                    values = []
                    valid_row = True
                    for param_name in param_names:
                        value = row.get(param_name, "")
                        if not self.is_float(value):
                            valid_row = False
                            break
                        values.append(float(value))

                    if valid_row:
                        rows.append((float(distance), values))
        except OSError:
            return [], []

        rows.sort(key=lambda item: item[0])
        return rows, param_names

    def save_local_csv(self):
        self.save_csv(self.local_csv_path)

    def save_connection_backup(self):
        if self.table.rowCount() == 0:
            return
        self.save_csv(self.backup_csv_path())
        self.detail_label.setText(f"Connection backup saved: {self.backup_csv_path()}")

    def save_csv(self, csv_path):
        try:
            directory = os.path.dirname(csv_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(csv_path, "w", newline="") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([self.key_column_name, *self.param_names])
                for row in range(self.table.rowCount()):
                    values = [self.cell_value(row, column) for column in range(self.data_column_count())]
                    formatted = [self.format_row_key(values[0], self.row_key_kind)]
                    for column, value in enumerate(values[1:]):
                        spec = self.column_specs[column] if column < len(self.column_specs) else {"kind": "double"}
                        if spec["kind"] == "int":
                            formatted.append(str(int(round(self.coerce_numeric_value(value)))))
                        elif spec["kind"] == "bool":
                            formatted.append("true" if self.coerce_bool_value(value) else "false")
                        elif spec["kind"] == "string":
                            formatted.append(self.coerce_text_value(value))
                        else:
                            formatted.append(f"{self.coerce_numeric_value(value):.{self.decimals_value}f}")
                    writer.writerow(formatted)
        except OSError as exc:
            self.detail_label.setText(f"Could not save CSV: {exc}")

    def normalize_nt_path(self, path):
        path = path.strip().strip("/")
        return path or self.settings.get("nt_path", FALLBACK_NT_PATH)

    def is_float(self, value):
        try:
            float(value)
            return True
        except (TypeError, ValueError):
            return False


def main():
    app = QApplication(sys.argv)
    window = MapTunerWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
