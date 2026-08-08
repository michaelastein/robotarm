#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import threading
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node


HTML_PAGE = r"""<!doctype html>
<html lang="de">
<head>
    <meta charset="utf-8">
    <meta
        name="viewport"
        content="width=device-width, initial-scale=1, viewport-fit=cover"
    >
    <title>Hotspot Evaluation</title>

    <style>
        :root {
            color-scheme: dark;
        }

        * {
            box-sizing: border-box;
        }

        html,
        body {
            width: 100%;
            height: 100%;
            margin: 0;
            overflow: hidden;
            background: #000;
            font-family: system-ui, sans-serif;
        }

        canvas {
            position: fixed;
            inset: 0;
            display: block;
            width: 100%;
            height: 100%;
            background: #000;
        }

        #controls {
            position: fixed;
            z-index: 10;
            top: 14px;
            left: 14px;
            display: flex;
            align-items: center;
            gap: 8px;
            padding: 9px;
            border: 1px solid #0a0a0a;
            border-radius: 10px;
            background: rgba(15, 15, 15, 0.88);
            backdrop-filter: blur(6px);
        }

        select,
        button {
            min-height: 38px;
            border: 1px solid #111;
            border-radius: 7px;
            padding: 7px 10px;
            color: #080808;
            background: #171717;
            font-size: 14px;
        }

        button {
            cursor: pointer;
        }

        button:hover {
            background: #292929;
        }

        #status {
            min-width: 220px;
            color: #080808;
            font-size: 13px;
        }

        #startOverlay {
            position: fixed;
            z-index: 9;
            right: 14px;
            bottom: 14px;
            pointer-events: none;
        }

        #startMessage {
            padding: 12px 16px;
            border: 1px solid #111;
            border-radius: 10px;
            color: #080808;
            background: rgba(20, 20, 20, 0.92);
            font-size: 16px;
            text-align: center;
            pointer-events: auto;
            cursor: pointer;
        }

        #startMessage:hover {
            background: #292929;
        }

        @media (max-width: 750px) {
            #controls {
                right: 14px;
                flex-wrap: wrap;
            }

            #status {
                width: 100%;
            }
        }
    </style>
</head>

<body>
    <canvas id="canvas"></canvas>

    <div id="startOverlay">
        <div id="startMessage">Zum Starten klicken</div>
    </div>

    <div id="controls">
        <select id="mode">
            <option value="lines">
                Linien
            </option>
            <option value="expandingPoints">
                Punkte
            </option>
            <option value="curves">
                Kurven
            </option>
        </select>

        <button id="resetButton" type="button">Stop / Reset</button>
        <button id="fullscreenButton" type="button">Vollbild</button>

        <span id="status">Warte auf Klick</span>
    </div>

    <script>
        "use strict";

        const canvas = document.getElementById("canvas");
        const context = canvas.getContext("2d", { alpha: false });

        const modeSelect = document.getElementById("mode");
        const resetButton = document.getElementById("resetButton");
        const fullscreenButton =
            document.getElementById("fullscreenButton");
        const statusElement = document.getElementById("status");
        const startOverlay = document.getElementById("startOverlay");

        const CIRCLE_RADIUS = 18;
        const EXPERIMENT_DURATION_SECONDS = 180;
        const CSS_PIXELS_PER_CM = 96 / 2.54;
        const POINT_STEP_PIXELS = 3 * CSS_PIXELS_PER_CM;
        const POINT_WAIT_SECONDS = 10;
        const LINE_SPEED_PIXELS_PER_SECOND = 35;

        // Bewegungsbereich: abgerundetes Rechteck.
        // Das Rechteck lässt links, oben und unten 10 % frei; rechts 30 %.
        // Der Kreisradius wird zusätzlich berücksichtigt, damit der Kreis
        // vollständig innerhalb des Bewegungsbereichs bleibt.
        const MOVEMENT_MARGIN_FRACTION = 0.10;
        const MOVEMENT_RIGHT_MARGIN_FRACTION = 0.30;
        const ROUNDED_RECT_CORNER_RADIUS_FRACTION = 0.08;

        const MODE_SEEDS = {
            lines: 104729,
            expandingPoints: 224737,
            curves: 736879
        };

        let viewportWidth = 1;
        let viewportHeight = 1;
        let deviceScale = 1;

        let selectedMode = modeSelect.value;
        let generatedPattern = null;

        let isRunning = false;
        let isStarting = false;
        let experimentFinished = false;
        let finishRequestSent = false;

        let startTime = 0;

        let currentPosition = {
            x: 0.5,
            y: 0.5
        };

        function createRandom(seed) {
            let state = seed >>> 0;

            return function random() {
                state += 0x6D2B79F5;

                let value = state;
                value = Math.imul(
                    value ^ (value >>> 15),
                    value | 1
                );
                value ^= value + Math.imul(
                    value ^ (value >>> 7),
                    value | 61
                );

                return (
                    (value ^ (value >>> 14)) >>> 0
                ) / 4294967296;
            };
        }

        function clamp(value, minimum, maximum) {
            return Math.max(minimum, Math.min(maximum, value));
        }

        function lerp(start, end, amount) {
            return start + (end - start) * amount;
        }

        function easeInOut(amount) {
            const value = clamp(amount, 0, 1);
            return value * value * (3 - 2 * value);
        }

        function getRoundedRectBounds() {
            const marginLeft = viewportWidth * MOVEMENT_MARGIN_FRACTION;
            const marginRight =
                viewportWidth * MOVEMENT_RIGHT_MARGIN_FRACTION;
            const marginY = viewportHeight * MOVEMENT_MARGIN_FRACTION;

            // Zusätzlich zum prozentualen Rand wird der Kreisradius eingerückt,
            // damit auch die Kreisfläche komplett im Rechteck bleibt.
            const left = marginLeft + CIRCLE_RADIUS;
            const right = Math.max(
                left + 2,
                viewportWidth - marginRight - CIRCLE_RADIUS
            );
            const top = marginY + CIRCLE_RADIUS;
            const bottom = Math.max(
                top + 2,
                viewportHeight - marginY - CIRCLE_RADIUS
            );

            const halfWidth = Math.max(1, (right - left) / 2);
            const halfHeight = Math.max(1, (bottom - top) / 2);

            const cornerRadius = Math.min(
                Math.min(viewportWidth, viewportHeight) *
                    ROUNDED_RECT_CORNER_RADIUS_FRACTION,
                halfWidth,
                halfHeight
            );

            return {
                left,
                right,
                top,
                bottom,
                cornerRadius
            };
        }

        function isInsideRoundedRectPixels(x, y, bounds) {
            const { left, right, top, bottom, cornerRadius } = bounds;

            if (x < left || x > right || y < top || y > bottom) {
                return false;
            }

            if (cornerRadius <= 0) {
                return true;
            }

            const innerLeft = left + cornerRadius;
            const innerRight = right - cornerRadius;
            const innerTop = top + cornerRadius;
            const innerBottom = bottom - cornerRadius;

            if (
                (x >= innerLeft && x <= innerRight) ||
                (y >= innerTop && y <= innerBottom)
            ) {
                return true;
            }

            const cornerX = x < innerLeft ? innerLeft : innerRight;
            const cornerY = y < innerTop ? innerTop : innerBottom;

            return Math.hypot(x - cornerX, y - cornerY) <= cornerRadius;
        }

        function projectInsideRoundedRect(point) {
            const bounds = getRoundedRectBounds();
            const { left, right, top, bottom, cornerRadius } = bounds;

            let x = clamp(point.x * viewportWidth, left, right);
            let y = clamp(point.y * viewportHeight, top, bottom);

            if (cornerRadius > 0) {
                const innerLeft = left + cornerRadius;
                const innerRight = right - cornerRadius;
                const innerTop = top + cornerRadius;
                const innerBottom = bottom - cornerRadius;

                const inCornerColumn = x < innerLeft || x > innerRight;
                const inCornerRow = y < innerTop || y > innerBottom;

                if (inCornerColumn && inCornerRow) {
                    const cornerX = x < innerLeft ? innerLeft : innerRight;
                    const cornerY = y < innerTop ? innerTop : innerBottom;
                    const dx = x - cornerX;
                    const dy = y - cornerY;
                    const distance = Math.hypot(dx, dy);

                    if (distance > cornerRadius) {
                        const scale = cornerRadius / distance;
                        x = cornerX + dx * scale;
                        y = cornerY + dy * scale;
                    }
                }
            }

            return {
                x: x / viewportWidth,
                y: y / viewportHeight
            };
        }

        function randomPointInRoundedRect(random) {
            const bounds = getRoundedRectBounds();

            for (let attempt = 0; attempt < 1000; attempt += 1) {
                const x = lerp(bounds.left, bounds.right, random());
                const y = lerp(bounds.top, bounds.bottom, random());

                if (isInsideRoundedRectPixels(x, y, bounds)) {
                    return {
                        x: x / viewportWidth,
                        y: y / viewportHeight
                    };
                }
            }

            return { x: 0.5, y: 0.5 };
        }

        function reflectInside(value, minimum, maximum) {
            const range = maximum - minimum;

            if (range <= 0) {
                return minimum;
            }

            let normalized = (value - minimum) % (range * 2);

            if (normalized < 0) {
                normalized += range * 2;
            }

            if (normalized > range) {
                normalized = range * 2 - normalized;
            }

            return minimum + normalized;
        }

        function formatTime(seconds) {
            const roundedSeconds = Math.max(0, Math.ceil(seconds));
            const minutes = Math.floor(roundedSeconds / 60);
            const remainingSeconds = roundedSeconds % 60;

            return (
                String(minutes).padStart(2, "0") +
                ":" +
                String(remainingSeconds).padStart(2, "0")
            );
        }

        function resizeCanvas() {
            viewportWidth = window.innerWidth;
            viewportHeight = window.innerHeight;
            deviceScale = Math.min(window.devicePixelRatio || 1, 2);

            canvas.width = Math.round(viewportWidth * deviceScale);
            canvas.height = Math.round(viewportHeight * deviceScale);
            canvas.style.width = `${viewportWidth}px`;
            canvas.style.height = `${viewportHeight}px`;

            context.setTransform(
                deviceScale,
                0,
                0,
                deviceScale,
                0,
                0
            );

            if (!isRunning) {
                prepareMode();
            }

            draw(currentPosition);
        }

        function prepareMode() {
            selectedMode = modeSelect.value;
            generatedPattern = generatePattern(selectedMode);

            currentPosition = {
                x: 0.5,
                y: 0.5
            };

            experimentFinished = false;
            finishRequestSent = false;
        }

        function buildPathMetrics(samples) {
            const cumulativeLengths = [0];
            let totalLength = 0;

            for (let index = 1; index < samples.length; index += 1) {
                const previous = samples[index - 1];
                const current = samples[index];

                totalLength += Math.hypot(
                    (current.x - previous.x) * viewportWidth,
                    (current.y - previous.y) * viewportHeight
                );

                cumulativeLengths.push(totalLength);
            }

            return {
                samples,
                cumulativeLengths,
                totalLength,
                speed: LINE_SPEED_PIXELS_PER_SECOND
            };
        }

        function generatePattern(modeName) {
            const seed = MODE_SEEDS[modeName] ?? 123456;
            const random = createRandom(seed);

            if (modeName === "lines") {
                const anchors = [{
                    x: 0.5,
                    y: 0.5
                }];

                for (let index = 0; index < 32; index += 1) {
                    anchors.push(randomPointInRoundedRect(random));
                }

                const samples = [anchors[0]];

                for (let index = 0; index < anchors.length - 1; index += 1) {
                    const start = anchors[index];
                    const end = anchors[index + 1];
                    const pixelDistance = Math.hypot(
                        (end.x - start.x) * viewportWidth,
                        (end.y - start.y) * viewportHeight
                    );
                    const sampleCount = Math.max(2, Math.ceil(pixelDistance / 8));

                    for (let step = 1; step <= sampleCount; step += 1) {
                        const t = step / sampleCount;
                        samples.push({
                            x: lerp(start.x, end.x, t),
                            y: lerp(start.y, end.y, t)
                        });
                    }
                }

                return buildPathMetrics(samples);
            }

            if (modeName === "expandingPoints") {
                const points = [{
                    x: 0.5,
                    y: 0.5
                }];
                const cycleDurations = [];
                const cumulativeCycleDurations = [0];

                let x = 0.5;
                let y = 0.5;

                for (let index = 1; index < 500; index += 1) {
                    const angle = random() * Math.PI * 2;
                    const distanceX = POINT_STEP_PIXELS / viewportWidth;
                    const distanceY = POINT_STEP_PIXELS / viewportHeight;

                    const projectedPoint = projectInsideRoundedRect({
                        x: x + Math.cos(angle) * distanceX,
                        y: y + Math.sin(angle) * distanceY
                    });

                    x = projectedPoint.x;
                    y = projectedPoint.y;

                    points.push({ x, y });

                    // Der Punkt springt sofort 3 cm in eine zufällige Richtung
                    // und bleibt anschließend exakt 10 Sekunden stehen.
                    const cycleDuration = POINT_WAIT_SECONDS;

                    cycleDurations.push(cycleDuration);
                    cumulativeCycleDurations.push(
                        cumulativeCycleDurations[
                            cumulativeCycleDurations.length - 1
                        ] + cycleDuration
                    );
                }

                return {
                    points,
                    cycleDurations,
                    cumulativeCycleDurations,
                    totalCycleDuration:
                        cumulativeCycleDurations[
                            cumulativeCycleDurations.length - 1
                        ]
                };
            }

            if (modeName === "curves") {
                const anchors = [{ x: 0.5, y: 0.5 }];

                for (let index = 0; index < 26; index += 1) {
                    anchors.push(randomPointInRoundedRect(random));
                }

                const samples = [anchors[0]];

                for (let index = 0; index < anchors.length - 1; index += 1) {
                    const start = anchors[index];
                    const end = anchors[index + 1];
                    const dx = end.x - start.x;
                    const dy = end.y - start.y;
                    const length = Math.hypot(dx, dy) || 1;
                    const normalX = -dy / length;
                    const normalY = dx / length;

                    // Seeded random: bei gleicher Fenstergröße jedes Mal dieselben
                    // runden Kurven, aber mit unterschiedlich großen Schlenkern.
                    const bendDirection = random() < 0.5 ? -1 : 1;
                    const bend1 = bendDirection * lerp(0.035, 0.22, random());
                    const bend2 = bendDirection * lerp(0.025, 0.18, random());
                    const along1 = lerp(0.20, 0.42, random());
                    const along2 = lerp(0.58, 0.82, random());

                    const control1 = projectInsideRoundedRect({
                        x: start.x + dx * along1 + normalX * bend1,
                        y: start.y + dy * along1 + normalY * bend1
                    });

                    const control2 = projectInsideRoundedRect({
                        x: start.x + dx * along2 + normalX * bend2,
                        y: start.y + dy * along2 + normalY * bend2
                    });

                    const sampleCount = 72;

                    for (let step = 1; step <= sampleCount; step += 1) {
                        const t = step / sampleCount;
                        const inverse = 1 - t;

                        samples.push({
                            x:
                                inverse * inverse * inverse * start.x +
                                3 * inverse * inverse * t * control1.x +
                                3 * inverse * t * t * control2.x +
                                t * t * t * end.x,
                            y:
                                inverse * inverse * inverse * start.y +
                                3 * inverse * inverse * t * control1.y +
                                3 * inverse * t * t * control2.y +
                                t * t * t * end.y
                        });
                    }
                }

                return buildPathMetrics(samples);
            }

            return {};
        }

        function getPosition(modeName, seconds) {
            if (modeName === "lines" || modeName === "curves") {
                return getConstantSpeedPathPosition(seconds);
            }

            if (modeName === "expandingPoints") {
                return getExpandingPointsPosition(seconds);
            }

            return {
                x: 0.5,
                y: 0.5
            };
        }

        function getConstantSpeedPathPosition(seconds) {
            const pattern = generatedPattern;
            const travelledDistance = pattern.speed * seconds;
            const distance = travelledDistance % pattern.totalLength;

            let low = 0;
            let high = pattern.cumulativeLengths.length - 1;

            while (low < high) {
                const middle = Math.floor((low + high) / 2);

                if (pattern.cumulativeLengths[middle] < distance) {
                    low = middle + 1;
                } else {
                    high = middle;
                }
            }

            const endIndex = Math.max(1, low);
            const startIndex = endIndex - 1;
            const startDistance = pattern.cumulativeLengths[startIndex];
            const endDistance = pattern.cumulativeLengths[endIndex];
            const segmentLength = Math.max(
                0.000001,
                endDistance - startDistance
            );
            const progress = (distance - startDistance) / segmentLength;

            const start = pattern.samples[startIndex];
            const end = pattern.samples[endIndex];

            return {
                x: lerp(start.x, end.x, progress),
                y: lerp(start.y, end.y, progress)
            };
        }

        function getExpandingPointsPosition(seconds) {
            const pattern = generatedPattern;
            const wrappedTime =
                seconds % Math.max(0.001, pattern.totalCycleDuration);

            let low = 0;
            let high = pattern.cycleDurations.length - 1;

            while (low < high) {
                const middle = Math.floor((low + high + 1) / 2);

                if (
                    pattern.cumulativeCycleDurations[middle] <=
                    wrappedTime
                ) {
                    low = middle;
                } else {
                    high = middle - 1;
                }
            }

            const cycleIndex = low;

            // Sofort auf B springen und dort 10 Sekunden stehen bleiben.
            return pattern.points[
                (cycleIndex + 1) % pattern.points.length
            ];
        }

        function draw(position) {
            context.fillStyle = "#000";

            context.fillRect(
                0,
                0,
                viewportWidth,
                viewportHeight
            );

            if (!position) {
                return;
            }

            const pixelX =
                clamp(position.x, 0, 1) *
                viewportWidth;

            const pixelY =
                clamp(position.y, 0, 1) *
                viewportHeight;

            context.beginPath();

            context.arc(
                pixelX,
                pixelY,
                CIRCLE_RADIUS,
                0,
                Math.PI * 2
            );

            context.fillStyle = "#fff";
            context.fill();
        }

        async function startExperiment() {
            if (
                isRunning ||
                isStarting ||
                experimentFinished
            ) {
                return;
            }

            isStarting = true;
            finishRequestSent = false;

            statusElement.textContent =
                "Starte Rosbag-Aufzeichnung …";

            try {
                const response = await fetch(
                    "/api/start",
                    {
                        method: "POST",
                        headers: {
                            "Content-Type":
                                "application/json"
                        },
                        body: JSON.stringify({
                            mode: selectedMode
                        })
                    }
                );

                const result = await response.json();

                if (!response.ok) {
                    throw new Error(
                        result.error ||
                        "Start fehlgeschlagen"
                    );
                }

                startTime = performance.now();
                isRunning = true;

                startOverlay.style.display = "none";

                statusElement.textContent =
                    "Aufzeichnung: 03:00 verbleibend";

            } catch (error) {
                statusElement.textContent =
                    `Fehler: ${error.message}`;

                startOverlay.style.display = "grid";

            } finally {
                isStarting = false;
            }
        }

        async function finishExperiment() {
            if (finishRequestSent) {
                return;
            }

            finishRequestSent = true;
            isRunning = false;
            experimentFinished = true;

            statusElement.textContent =
                "Fertig – Rosbag wird beendet …";

            try {
                const response = await fetch(
                    "/api/finish",
                    {
                        method: "POST"
                    }
                );

                const result = await response.json();

                if (!response.ok) {
                    throw new Error(
                        result.error ||
                        "Rosbag konnte nicht beendet werden"
                    );
                }

                statusElement.textContent =
                    "Fertig – 3 Minuten aufgezeichnet";

            } catch (error) {
                statusElement.textContent =
                    "Bewegung beendet – " +
                    `Rosbag-Fehler: ${error.message}`;
            }
        }

        async function resetExperiment() {
            isRunning = false;
            isStarting = false;
            experimentFinished = false;
            finishRequestSent = false;

            try {
                await fetch(
                    "/api/reset",
                    {
                        method: "POST"
                    }
                );
            } catch (error) {
                console.error(
                    "Reset fehlgeschlagen:",
                    error
                );
            }

            prepareMode();

            startOverlay.style.display = "grid";

            statusElement.textContent =
                "Warte auf Klick";

            draw(currentPosition);
        }

        function animate(now) {
            if (isRunning) {
                const elapsedSeconds =
                    Math.max(0, now - startTime) / 1000;

                const animationTime =
                    Math.min(
                        elapsedSeconds,
                        EXPERIMENT_DURATION_SECONDS
                    );

                currentPosition =
                    getPosition(
                        selectedMode,
                        animationTime
                    );

                const remainingSeconds =
                    Math.max(
                        0,
                        EXPERIMENT_DURATION_SECONDS -
                            elapsedSeconds
                    );

                statusElement.textContent =
                    "Aufzeichnung: " +
                    formatTime(remainingSeconds) +
                    " verbleibend";

                if (
                    elapsedSeconds >=
                    EXPERIMENT_DURATION_SECONDS
                ) {
                    finishExperiment();
                }
            }

            draw(currentPosition);
            requestAnimationFrame(animate);
        }

        document.getElementById("startMessage").addEventListener(
            "click",
            () => {
                startExperiment();
            }
        );

        modeSelect.addEventListener(
            "change",
            () => {
                resetExperiment();
            }
        );

        resetButton.addEventListener(
            "click",
            () => {
                resetExperiment();
            }
        );

        fullscreenButton.addEventListener(
            "click",
            async () => {
                try {
                    if (!document.fullscreenElement) {
                        await document
                            .documentElement
                            .requestFullscreen();
                    } else {
                        await document.exitFullscreen();
                    }
                } catch (error) {
                    console.error(
                        "Vollbild fehlgeschlagen:",
                        error
                    );
                }
            }
        );

        window.addEventListener(
            "resize",
            resizeCanvas
        );

        prepareMode();
        resizeCanvas();
        requestAnimationFrame(animate);
    </script>
</body>
</html>
"""


class ExperimentNode(Node):
    """Steuert die Rosbag-Aufzeichnung für das Experiment."""

    EXPERIMENT_DURATION_SECONDS = 180.0

    def __init__(
        self,
        bag_directory: Path,
        bag_storage: str | None,
    ) -> None:
        super().__init__("hotspot_animation_server")

        self.bag_directory = bag_directory
        self.bag_storage = bag_storage

        self.bag_process: subprocess.Popen[str] | None = None
        self.bag_lock = threading.RLock()
        self.stop_timer: threading.Timer | None = None

    @staticmethod
    def _safe_name(value: str) -> str:
        """Macht ROS-Namen/Typen sicher für Verzeichnisnamen."""
        safe = "".join(
            character
            if character.isalnum() or character in "-_"
            else "_"
            for character in value
        )
        return safe.strip("_") or "unknown"

    def get_active_controller_label(self) -> str:
        """Liest den aktiven Bewegungscontroller über ros2 control aus."""
        try:
            result = subprocess.run(
                ["ros2", "control", "list_controllers"],
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().warning(
                f"Controller-Abfrage fehlgeschlagen: {error}"
            )
            return "controller_unknown"

        if result.returncode != 0:
            error_text = result.stderr.strip() or "unbekannter Fehler"
            self.get_logger().warning(
                "ros2 control list_controllers fehlgeschlagen: "
                + error_text
            )
            return "controller_unknown"

        active_controllers: list[tuple[str, str]] = []

        for raw_line in result.stdout.splitlines():
            columns = raw_line.split()
            if len(columns) < 3:
                continue

            controller_name = columns[0]
            controller_type = columns[1]
            state = columns[-1].lower()

            if state != "active":
                continue

            # Broadcaster liefern Zustände, sind aber nicht der eigentliche
            # Bewegungscontroller, der hier im Bag-Namen stehen soll.
            if "broadcaster" in controller_name.lower():
                continue

            active_controllers.append(
                (controller_name, controller_type)
            )

        if not active_controllers:
            self.get_logger().warning(
                "Kein aktiver Bewegungscontroller gefunden."
            )
            return "controller_unknown"

        controller_name, controller_type = active_controllers[0]
        type_short = controller_type.rsplit("/", 1)[-1]

        label = (
            "controller_"
            + self._safe_name(controller_name)
            + "_"
            + self._safe_name(type_short)
        )

        self.get_logger().info(
            "Aktiver Controller für Rosbag-Namen: "
            f"{controller_name} ({controller_type})"
        )
        return label

    def start_recording(
        self,
        mode: str,
    ) -> dict[str, Any]:
        with self.bag_lock:
            if self.recording_is_active_unlocked():
                raise RuntimeError(
                    "Eine Rosbag-Aufzeichnung läuft bereits."
                )

            self.cancel_stop_timer_unlocked()

            self.bag_directory.mkdir(
                parents=True,
                exist_ok=True,
            )

            safe_mode = self._safe_name(mode)
            controller_label = self.get_active_controller_label()

            timestamp = datetime.now().strftime(
                "%Y%m%d_%H%M%S_%f"
            )

            bag_name = (
                f"hotspot_{safe_mode}_{controller_label}_{timestamp}"
            )
            bag_path = self.bag_directory / bag_name

            command = [
                "ros2",
                "bag",
                "record",
                "-o",
                str(bag_path),
            ]

            if self.bag_storage:
                command.extend(
                    [
                        "--storage",
                        self.bag_storage,
                    ]
                )

            command.extend(
                [
                    "/joint_states",
                    "/hotspot/target",
                ]
            )

            self.get_logger().info(
                "Starte Rosbag: " + " ".join(command)
            )

            try:
                self.bag_process = subprocess.Popen(
                    command,
                    text=True,
                    start_new_session=True,
                )

            except FileNotFoundError as error:
                self.bag_process = None

                raise RuntimeError(
                    "Der Befehl 'ros2' wurde nicht gefunden. "
                    "Wurde die ROS-2-Umgebung gesourct?"
                ) from error

            try:
                return_code = self.bag_process.wait(
                    timeout=0.4
                )

            except subprocess.TimeoutExpired:
                return_code = None

            if return_code is not None:
                self.bag_process = None

                raise RuntimeError(
                    "ros2 bag record wurde sofort "
                    f"beendet, Exit-Code {return_code}."
                )

            self.stop_timer = threading.Timer(
                self.EXPERIMENT_DURATION_SECONDS,
                self.stop_recording_after_timeout,
            )

            self.stop_timer.daemon = True
            self.stop_timer.start()

            self.get_logger().info(
                "Rosbag läuft für maximal 180 Sekunden."
            )

            return {
                "bag_name": bag_name,
                "bag_path": str(bag_path),
                "duration_seconds":
                    self.EXPERIMENT_DURATION_SECONDS,
            }

    def stop_recording_after_timeout(self) -> None:
        self.get_logger().info(
            "180 Sekunden erreicht – "
            "beende Rosbag automatisch."
        )

        self.stop_recording()

    def recording_is_active_unlocked(self) -> bool:
        return (
            self.bag_process is not None
            and self.bag_process.poll() is None
        )

    def recording_is_active(self) -> bool:
        with self.bag_lock:
            return self.recording_is_active_unlocked()

    def cancel_stop_timer_unlocked(self) -> None:
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None

    def stop_recording(self) -> None:
        with self.bag_lock:
            self.cancel_stop_timer_unlocked()

            process = self.bag_process

            if process is None:
                return

            if process.poll() is not None:
                self.bag_process = None
                return

            self.get_logger().info(
                "Beende Rosbag sauber mit SIGINT …"
            )

            try:
                os.killpg(
                    os.getpgid(process.pid),
                    signal.SIGINT,
                )

                process.wait(timeout=10.0)

            except subprocess.TimeoutExpired:
                self.get_logger().warning(
                    "Rosbag reagiert nicht auf SIGINT. "
                    "Sende SIGTERM."
                )

                try:
                    os.killpg(
                        os.getpgid(process.pid),
                        signal.SIGTERM,
                    )

                    process.wait(timeout=3.0)

                except subprocess.TimeoutExpired:
                    self.get_logger().warning(
                        "Rosbag reagiert nicht auf "
                        "SIGTERM. Sende SIGKILL."
                    )

                    os.killpg(
                        os.getpgid(process.pid),
                        signal.SIGKILL,
                    )

                    process.wait(timeout=2.0)

            except ProcessLookupError:
                pass

            finally:
                self.bag_process = None

                self.get_logger().info(
                    "Rosbag-Aufzeichnung beendet."
                )

    def destroy_node(self) -> bool:
        self.stop_recording()
        return super().destroy_node()


class WebsiteRequestHandler(BaseHTTPRequestHandler):
    node: ExperimentNode

    server_version = "HotspotAnimation/1.0"

    def send_json(
        self,
        status: HTTPStatus,
        payload: dict[str, Any],
    ) -> None:
        content = json.dumps(payload).encode("utf-8")

        self.send_response(status.value)
        self.send_header(
            "Content-Type",
            "application/json; charset=utf-8",
        )
        self.send_header(
            "Content-Length",
            str(len(content)),
        )
        self.send_header(
            "Cache-Control",
            "no-store",
        )
        self.end_headers()
        self.wfile.write(content)

    def read_json(self) -> dict[str, Any]:
        content_length = int(
            self.headers.get(
                "Content-Length",
                "0",
            )
        )

        if content_length <= 0:
            return {}

        raw_content = self.rfile.read(content_length)

        try:
            parsed = json.loads(
                raw_content.decode("utf-8")
            )

        except json.JSONDecodeError as error:
            raise ValueError(
                "Ungültige JSON-Daten."
            ) from error

        if not isinstance(parsed, dict):
            raise ValueError(
                "Der JSON-Inhalt muss ein Objekt sein."
            )

        return parsed

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            content = HTML_PAGE.encode("utf-8")

            self.send_response(HTTPStatus.OK.value)
            self.send_header(
                "Content-Type",
                "text/html; charset=utf-8",
            )
            self.send_header(
                "Content-Length",
                str(len(content)),
            )
            self.send_header(
                "Cache-Control",
                "no-store",
            )
            self.end_headers()
            self.wfile.write(content)
            return

        if self.path == "/api/status":
            self.send_json(
                HTTPStatus.OK,
                {
                    "recording":
                        self.node.recording_is_active()
                },
            )
            return

        self.send_error(
            HTTPStatus.NOT_FOUND.value,
            "Nicht gefunden",
        )

    def do_POST(self) -> None:
        try:
            if self.path == "/api/start":
                data = self.read_json()
                mode = str(
                    data.get(
                        "mode",
                        "unknown",
                    )
                )

                result = self.node.start_recording(mode)

                self.send_json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        **result,
                    },
                )
                return

            if self.path == "/api/finish":
                self.node.stop_recording()

                self.send_json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        "message":
                            "Rosbag-Aufzeichnung beendet.",
                    },
                )
                return

            if self.path == "/api/reset":
                self.node.stop_recording()

                self.send_json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                    },
                )
                return

            self.send_error(
                HTTPStatus.NOT_FOUND.value,
                "Nicht gefunden",
            )

        except (TypeError, ValueError) as error:
            self.send_json(
                HTTPStatus.BAD_REQUEST,
                {
                    "error": str(error),
                },
            )

        except RuntimeError as error:
            self.send_json(
                HTTPStatus.CONFLICT,
                {
                    "error": str(error),
                },
            )

        except Exception as error:
            self.node.get_logger().error(
                "HTTP-Anfrage fehlgeschlagen: "
                f"{error}"
            )

            self.send_json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {
                    "error": str(error),
                },
            )

    def log_message(
        self,
        format_string: str,
        *arguments: object,
    ) -> None:
        self.node.get_logger().info(
            f"HTTP {self.client_address[0]}: "
            f"{format_string % arguments}"
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Webanimation mit automatischer "
            "ROS-2-Bag-Aufzeichnung"
        )
    )

    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Bind-Adresse des HTTP-Servers",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="Port des HTTP-Servers",
    )

    parser.add_argument(
        "--bag-directory",
        type=Path,
        default=Path.home() / "robotarm" / "evaluation",
        help="Verzeichnis für Rosbag-Dateien",
    )

    parser.add_argument(
        "--bag-storage",
        default=None,
        help=(
            "Optionales Storage-Plugin, "
            "beispielsweise mcap oder sqlite3"
        ),
    )

    return parser.parse_args(
        rclpy.utilities.remove_ros_args()[1:]
    )


def main() -> None:
    rclpy.init()

    arguments = parse_arguments()

    node = ExperimentNode(
        bag_directory=arguments.bag_directory,
        bag_storage=arguments.bag_storage,
    )

    WebsiteRequestHandler.node = node

    http_server = ThreadingHTTPServer(
        (
            arguments.host,
            arguments.port,
        ),
        WebsiteRequestHandler,
    )

    http_thread = threading.Thread(
        target=http_server.serve_forever,
        name="hotspot-animation-http",
        daemon=True,
    )

    http_thread.start()

    node.get_logger().info(
        "Webseite läuft unter "
        f"http://localhost:{arguments.port}"
    )

    node.get_logger().info(
        "Der Kreis wartet auf einen Klick im Browser."
    )

    node.get_logger().info(
        "Aufgezeichnete Topics: "
        "/joint_states, "
        "/hotspot/target"
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            "Node wird beendet."
        )

    finally:
        http_server.shutdown()
        http_server.server_close()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

