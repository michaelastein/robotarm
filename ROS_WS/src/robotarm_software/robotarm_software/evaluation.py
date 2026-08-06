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
            border: 1px solid #333;
            border-radius: 10px;
            background: rgba(15, 15, 15, 0.88);
            backdrop-filter: blur(6px);
        }

        select,
        button {
            min-height: 38px;
            border: 1px solid #555;
            border-radius: 7px;
            padding: 7px 10px;
            color: #fff;
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
            color: #bbb;
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
            border: 1px solid #555;
            border-radius: 10px;
            color: #fff;
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
            <option value="acceleratingLines">
                Linien
            </option>
            <option value="expandingPointsConstant">
                Punkte – konstante Zeit
            </option>
            <option value="expandingPointsSlower">
                Punkte – längere Zeit bei großen Sprüngen
            </option>
            <option value="randomWalk">
                Richtungswechsel
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
        const INITIAL_POINT_STEP_PIXELS = 1 * CSS_PIXELS_PER_CM;
        const MAXIMUM_POINT_STEP_PIXELS = 8 * CSS_PIXELS_PER_CM;

        const MODE_SEEDS = {
            acceleratingLines: 104729,
            expandingPointsConstant: 224737,
            expandingPointsSlower: 224737,
            randomWalk: 736879
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

        function generatePattern(modeName) {
            const seed = MODE_SEEDS[modeName] ?? 123456;
            const random = createRandom(seed);

            const marginX = (CIRCLE_RADIUS + 8) / viewportWidth;
            const marginY = (CIRCLE_RADIUS + 8) / viewportHeight;

            if (modeName === "acceleratingLines") {
                const anchors = [{
                    x: 0.5,
                    y: 0.5
                }];

                for (let index = 0; index < 32; index += 1) {
                    anchors.push({
                        x: lerp(marginX, 1 - marginX, random()),
                        y: lerp(marginY, 1 - marginY, random())
                    });
                }

                const samples = [anchors[0]];

                for (let index = 0; index < anchors.length - 1; index += 1) {
                    const start = anchors[index];
                    const end = anchors[index + 1];
                    const useCurve = index % 3 !== 0;
                    const sampleCount = useCurve ? 48 : 24;

                    if (useCurve) {
                        const midpointX = (start.x + end.x) * 0.5;
                        const midpointY = (start.y + end.y) * 0.5;
                        const normalX = -(end.y - start.y);
                        const normalY = end.x - start.x;
                        const normalLength = Math.hypot(normalX, normalY) || 1;
                        const bend = lerp(-0.16, 0.16, random());
                        const controlX = clamp(
                            midpointX + normalX / normalLength * bend,
                            marginX,
                            1 - marginX
                        );
                        const controlY = clamp(
                            midpointY + normalY / normalLength * bend,
                            marginY,
                            1 - marginY
                        );

                        for (let step = 1; step <= sampleCount; step += 1) {
                            const t = step / sampleCount;
                            const inverse = 1 - t;
                            samples.push({
                                x:
                                    inverse * inverse * start.x +
                                    2 * inverse * t * controlX +
                                    t * t * end.x,
                                y:
                                    inverse * inverse * start.y +
                                    2 * inverse * t * controlY +
                                    t * t * end.y
                            });
                        }
                    } else {
                        for (let step = 1; step <= sampleCount; step += 1) {
                            const t = step / sampleCount;
                            samples.push({
                                x: lerp(start.x, end.x, t),
                                y: lerp(start.y, end.y, t)
                            });
                        }
                    }
                }

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
                    initialSpeed: 22,
                    acceleration: 2.1
                };
            }

            if (
                modeName === "expandingPointsConstant" ||
                modeName === "expandingPointsSlower"
            ) {
                const points = [{
                    x: 0.5,
                    y: 0.5
                }];
                const cycleDurations = [];
                const cumulativeCycleDurations = [0];

                let x = 0.5;
                let y = 0.5;
                let stepPixels = INITIAL_POINT_STEP_PIXELS;

                // 8 cm, soweit es die aktuelle Fenstergröße zulässt.
                const maximumStepPixels = Math.min(
                    MAXIMUM_POINT_STEP_PIXELS,
                    Math.min(viewportWidth, viewportHeight) * 0.72
                );

                for (let index = 1; index < 500; index += 1) {
                    const angle = random() * Math.PI * 2;
                    const distanceX = stepPixels / viewportWidth;
                    const distanceY = stepPixels / viewportHeight;

                    x = reflectInside(
                        x + Math.cos(angle) * distanceX,
                        marginX,
                        1 - marginX
                    );
                    y = reflectInside(
                        y + Math.sin(angle) * distanceY,
                        marginY,
                        1 - marginY
                    );

                    points.push({ x, y });

                    const stepProgress = clamp(
                        (stepPixels - INITIAL_POINT_STEP_PIXELS) /
                            Math.max(
                                1,
                                maximumStepPixels -
                                    INITIAL_POINT_STEP_PIXELS
                            ),
                        0,
                        1
                    );

                    const cycleDuration =
                        modeName === "expandingPointsSlower"
                            ? lerp(2.0, 5.0, stepProgress)
                            : 2.0;

                    cycleDurations.push(cycleDuration);
                    cumulativeCycleDurations.push(
                        cumulativeCycleDurations[
                            cumulativeCycleDurations.length - 1
                        ] + cycleDuration
                    );

                    stepPixels = Math.min(
                        maximumStepPixels,
                        stepPixels * 1.014 + 0.45
                    );
                }

                return {
                    points,
                    cycleDurations,
                    cumulativeCycleDurations,
                    totalCycleDuration:
                        cumulativeCycleDurations[
                            cumulativeCycleDurations.length - 1
                        ],
                    blankDuration: 0.08
                };
            }

            if (modeName === "bouncing") {
                return {
                    startX: 0.5,
                    startY: 0.5,
                    velocityX: lerp(0.14, 0.25, random()),
                    velocityY: lerp(0.12, 0.22, random()),
                    marginX,
                    marginY
                };
            }

            if (modeName === "orbits") {
                const orbits = [];

                for (let index = 0; index < 20; index += 1) {
                    orbits.push({
                        duration: lerp(2.0, 4.5, random()),
                        radiusX: lerp(0.08, 0.39, random()),
                        radiusY: lerp(0.06, 0.35, random()),
                        phase: random() * Math.PI * 2,
                        direction: random() > 0.5 ? 1 : -1,
                        centerX: lerp(0.42, 0.58, random()),
                        centerY: lerp(0.42, 0.58, random())
                    });
                }

                return { orbits };
            }

            if (modeName === "randomWalk") {
                const segments = [];

                let x = 0.5;
                let y = 0.5;
                let angle = random() * Math.PI * 2;

                for (let index = 0; index < 160; index += 1) {
                    angle += lerp(-1.7, 1.7, random());

                    const distance = lerp(0.08, 0.32, random());

                    const nextX = reflectInside(
                        x + Math.cos(angle) * distance,
                        marginX,
                        1 - marginX
                    );

                    const nextY = reflectInside(
                        y + Math.sin(angle) * distance,
                        marginY,
                        1 - marginY
                    );

                    segments.push({
                        fromX: x,
                        fromY: y,
                        toX: nextX,
                        toY: nextY
                    });

                    x = nextX;
                    y = nextY;
                }

                return {
                    segments,
                    initialDuration: 4.50,
                    speedup: 0.985,
                    minimumDuration: 0.30
                };
            }

            return {};
        }

        function getPosition(modeName, seconds) {
            if (modeName === "acceleratingLines") {
                return getAcceleratingLinesPosition(seconds);
            }

            if (
                modeName === "expandingPointsConstant" ||
                modeName === "expandingPointsSlower"
            ) {
                return getExpandingPointsPosition(seconds);
            }

            if (modeName === "spiral") {
                return getSpiralPosition(seconds);
            }

            if (modeName === "bouncing") {
                return getBouncingPosition(seconds);
            }

            if (modeName === "orbits") {
                return getOrbitPosition(seconds);
            }

            if (modeName === "randomWalk") {
                return getRandomWalkPosition(seconds);
            }

            return {
                x: 0.5,
                y: 0.5
            };
        }

        function getAcceleratingLinesPosition(seconds) {
            const pattern = generatedPattern;
            const travelledDistance =
                pattern.initialSpeed * seconds +
                0.5 * pattern.acceleration * seconds * seconds;

            const distance =
                travelledDistance % pattern.totalLength;

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
            const startDistance =
                pattern.cumulativeLengths[startIndex];
            const endDistance =
                pattern.cumulativeLengths[endIndex];
            const segmentLength =
                Math.max(0.000001, endDistance - startDistance);
            const progress =
                (distance - startDistance) / segmentLength;

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
            const localTime =
                wrappedTime -
                pattern.cumulativeCycleDurations[cycleIndex];
            const cycleDuration =
                pattern.cycleDurations[cycleIndex];
            const visibleDuration = Math.max(
                0,
                cycleDuration - pattern.blankDuration
            );

            if (localTime >= visibleDuration) {
                return null;
            }

            return pattern.points[
                (cycleIndex + 1) % pattern.points.length
            ];
        }

        function getSpiralPosition(seconds) {
            const maximumRadius = 0.44;

            const normalizedTime =
                seconds / EXPERIMENT_DURATION_SECONDS;

            const radius =
                maximumRadius *
                Math.pow(
                    clamp(normalizedTime, 0, 1),
                    0.72
                );

            const angle =
                normalizedTime * Math.PI * 42 +
                normalizedTime *
                normalizedTime *
                Math.PI *
                30;

            return {
                x: 0.5 + Math.cos(angle) * radius,
                y: 0.5 + Math.sin(angle) * radius
            };
        }

        function getBouncingPosition(seconds) {
            const pattern = generatedPattern;

            const transformedTime =
                seconds +
                0.045 *
                seconds *
                seconds;

            return {
                x: reflectInside(
                    pattern.startX +
                        pattern.velocityX *
                        transformedTime,
                    pattern.marginX,
                    1 - pattern.marginX
                ),

                y: reflectInside(
                    pattern.startY +
                        pattern.velocityY *
                        transformedTime,
                    pattern.marginY,
                    1 - pattern.marginY
                )
            };
        }

        function getOrbitPosition(seconds) {
            const orbits = generatedPattern.orbits;

            const totalDuration = orbits.reduce(
                (sum, orbit) => sum + orbit.duration,
                0
            );

            let localTime = seconds % totalDuration;
            let selectedOrbit = orbits[0];

            for (const orbit of orbits) {
                if (localTime <= orbit.duration) {
                    selectedOrbit = orbit;
                    break;
                }

                localTime -= orbit.duration;
            }

            const progress =
                localTime / selectedOrbit.duration;

            const speedIncrease =
                1 +
                seconds /
                    EXPERIMENT_DURATION_SECONDS *
                    2.5;

            const angle =
                selectedOrbit.phase +
                selectedOrbit.direction *
                    progress *
                    Math.PI *
                    2 *
                    speedIncrease;

            return {
                x:
                    selectedOrbit.centerX +
                    Math.cos(angle) *
                        selectedOrbit.radiusX,

                y:
                    selectedOrbit.centerY +
                    Math.sin(angle) *
                        selectedOrbit.radiusY
            };
        }

        function getRandomWalkPosition(seconds) {
            let remainingTime = seconds;
            let segmentIndex = 0;

            while (segmentIndex < 10000) {
                const duration = Math.max(
                    generatedPattern.minimumDuration,

                    generatedPattern.initialDuration *
                        Math.pow(
                            generatedPattern.speedup,
                            segmentIndex
                        )
                );

                if (remainingTime < duration) {
                    const segments =
                        generatedPattern.segments;

                    const segment =
                        segments[
                            segmentIndex %
                            segments.length
                        ];

                    const progress = easeInOut(
                        remainingTime / duration
                    );

                    return {
                        x: lerp(
                            segment.fromX,
                            segment.toX,
                            progress
                        ),

                        y: lerp(
                            segment.fromY,
                            segment.toY,
                            progress
                        )
                    };
                }

                remainingTime -= duration;
                segmentIndex += 1;
            }

            return currentPosition;
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

            safe_mode = "".join(
                character
                if character.isalnum()
                or character in "-_"
                else "_"
                for character in mode
            )

            timestamp = datetime.now().strftime(
                "%Y%m%d_%H%M%S_%f"
            )

            bag_name = f"hotspot_{safe_mode}_{timestamp}"
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
                    "/joint_state_broadcaster/joint_states",
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
        default=Path("./bags"),
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
        "/joint_state_broadcaster/joint_states, "
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
