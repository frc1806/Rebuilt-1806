package frc.robot.lib.pathfollowing;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Loads and parses BLine JSON path files.
 * Handles JSON parsing, constraint processing, alliance flipping, and path caching.
 */
public class BLinePathLoader {
    private static final Gson gson = new Gson();
    private static final Map<String, ParsedPath> bluePathCache = new HashMap<>();
    private static final Map<String, ParsedPath> redPathCache = new HashMap<>();

    // 2026 field dimensions for alliance flipping
    private static final double FIELD_LENGTH = 16.540988; // meters
    private static final double FIELD_WIDTH = 8.069326; // meters

    // Default constraints from config.json
    private static double defaultMaxVelocity = 2.0;
    private static double defaultMaxAcceleration = 3.4;
    private static double defaultEndTranslationTolerance = 0.3;
    private static double defaultEndRotationTolerance = 10.0;
    private static boolean configLoaded = false;

    /**
     * Loads the config.json file to get default constraints.
     */
    private static void loadConfig() {
        if (configLoaded) {
            return;
        }

        try {
            Path configPath = Filesystem.getDeployDirectory().toPath()
                .resolve("autos/config.json");
            String jsonContent = Files.readString(configPath);
            JsonObject config = gson.fromJson(jsonContent, JsonObject.class);

            if (config.has("kinematic_constraints")) {
                JsonObject constraints = config.getAsJsonObject("kinematic_constraints");

                if (constraints.has("default_max_velocity_meters_per_sec")) {
                    defaultMaxVelocity = constraints.get("default_max_velocity_meters_per_sec").getAsDouble();
                }
                if (constraints.has("default_max_acceleration_meters_per_sec2")) {
                    defaultMaxAcceleration = constraints.get("default_max_acceleration_meters_per_sec2").getAsDouble();
                }
                if (constraints.has("default_end_translation_tolerance_meters")) {
                    defaultEndTranslationTolerance = constraints.get("default_end_translation_tolerance_meters").getAsDouble();
                }
                if (constraints.has("default_end_rotation_tolerance_deg")) {
                    defaultEndRotationTolerance = constraints.get("default_end_rotation_tolerance_deg").getAsDouble();
                }
            }

            configLoaded = true;
            System.out.println("BLinePathLoader: Loaded config defaults - vel=" + defaultMaxVelocity
                + " m/s, accel=" + defaultMaxAcceleration + " m/s^2");
        } catch (Exception e) {
            System.err.println("BLinePathLoader: Failed to load config.json, using fallback defaults: " + e.getMessage());
            configLoaded = true; // Don't keep trying
        }
    }

    /**
     * Loads a path from JSON file with optional alliance flipping.
     *
     * @param pathName The name of the path (without .json extension)
     * @param shouldFlip Whether to flip the path for red alliance
     * @return The parsed path
     * @throws RuntimeException if path loading or parsing fails
     */
    public static ParsedPath loadPath(String pathName, boolean shouldFlip) {
        // Load config on first use
        loadConfig();

        // Check cache first
        Map<String, ParsedPath> cache = shouldFlip ? redPathCache : bluePathCache;
        if (cache.containsKey(pathName)) {
            return cache.get(pathName);
        }

        // Load from filesystem
        Path filePath = Filesystem.getDeployDirectory().toPath()
            .resolve("autos/paths/" + pathName + ".json");

        String jsonContent;
        try {
            jsonContent = Files.readString(filePath);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load path file: " + pathName, e);
        }

        // Parse JSON
        ParsedPath path;
        try {
            path = parseJson(jsonContent, pathName);
        } catch (JsonParseException e) {
            throw new RuntimeException("Failed to parse path JSON: " + pathName, e);
        }

        // Flip for red alliance if needed
        if (shouldFlip) {
            path = flipPath(path);
        }

        // Cache and return
        cache.put(pathName, path);
        return path;
    }

    /**
     * Parses a BLine JSON string into a ParsedPath.
     *
     * @param jsonContent The JSON string content
     * @param pathName The name of the path (for error messages)
     * @return The parsed path
     */
    private static ParsedPath parseJson(String jsonContent, String pathName) {
        JsonObject root = gson.fromJson(jsonContent, JsonObject.class);

        // Parse path elements (waypoints)
        JsonArray elements = root.getAsJsonArray("path_elements");
        List<PathWaypoint> waypoints = new ArrayList<>();

        for (JsonElement elem : elements) {
            JsonObject obj = elem.getAsJsonObject();
            String type = obj.get("type").getAsString();

            if (type.equals("waypoint")) {
                // Waypoint with translation and rotation targets
                JsonObject translationTarget = obj.getAsJsonObject("translation_target");
                JsonObject rotationTarget = obj.getAsJsonObject("rotation_target");

                double x = translationTarget.get("x_meters").getAsDouble();
                double y = translationTarget.get("y_meters").getAsDouble();
                double radius = translationTarget.get("intermediate_handoff_radius_meters").getAsDouble();
                double rotation = rotationTarget.get("rotation_radians").getAsDouble();

                waypoints.add(new PathWaypoint(
                    new Translation2d(x, y),
                    new Rotation2d(rotation),
                    radius,
                    true
                ));
            } else if (type.equals("translation")) {
                // Translation only (no rotation target)
                double x = obj.get("x_meters").getAsDouble();
                double y = obj.get("y_meters").getAsDouble();
                double radius = obj.get("intermediate_handoff_radius_meters").getAsDouble();

                waypoints.add(new PathWaypoint(
                    new Translation2d(x, y),
                    null, // Will be filled in below
                    radius,
                    false
                ));
            }
        }

        if (waypoints.isEmpty()) {
            throw new RuntimeException("Path has no waypoints: " + pathName);
        }

        // Fill in rotation targets for translation-only elements
        fillRotationTargets(waypoints);

        // Parse constraints
        JsonObject constraints = root.getAsJsonObject("constraints");

        // Parse velocity constraints
        double[] velocityPerSegment = new double[waypoints.size()];
        parseVelocityConstraints(constraints, velocityPerSegment);

        // Parse acceleration constraints
        double[] accelerationPerSegment = new double[waypoints.size()];
        parseAccelerationConstraints(constraints, accelerationPerSegment);

        // Parse end tolerances (use config defaults or path-specific values)
        double endTranslationTolerance = defaultEndTranslationTolerance;
        double endRotationTolerance = defaultEndRotationTolerance;
        double endVelocityTolerance = Constants.PurePursuitConstants.DEFAULT_VELOCITY_TOLERANCE;

        if (constraints != null && constraints.has("end_translation_tolerance_meters")) {
            endTranslationTolerance = constraints.get("end_translation_tolerance_meters").getAsDouble();
        }
        if (constraints != null && constraints.has("end_rotation_tolerance_deg")) {
            endRotationTolerance = constraints.get("end_rotation_tolerance_deg").getAsDouble();
        }

        return new ParsedPath(
            waypoints,
            velocityPerSegment,
            accelerationPerSegment,
            endTranslationTolerance,
            endRotationTolerance,
            endVelocityTolerance
        );
    }

    /**
     * Fills in rotation targets for translation-only waypoints.
     * Maintains the last valid rotation target from previous waypoints.
     *
     * @param waypoints The list of waypoints to process
     */
    private static void fillRotationTargets(List<PathWaypoint> waypoints) {
        Rotation2d lastValidRotation = waypoints.get(0).getRotation();

        // If first waypoint has no rotation, use 0 degrees
        if (lastValidRotation == null) {
            lastValidRotation = new Rotation2d();
            waypoints.get(0).setRotation(lastValidRotation);
        }

        // Fill in rotation for subsequent waypoints
        for (int i = 1; i < waypoints.size(); i++) {
            PathWaypoint wp = waypoints.get(i);
            if (wp.getRotation() != null) {
                lastValidRotation = wp.getRotation();
            } else {
                wp.setRotation(lastValidRotation);
            }
        }
    }

    /**
     * Parses velocity constraints and fills per-segment array.
     *
     * @param constraints The constraints JSON object
     * @param velocityPerSegment The array to fill (output)
     */
    private static void parseVelocityConstraints(JsonObject constraints, double[] velocityPerSegment) {
        // Use default from config (or fall back to robot max speed if config value is too high)
        double defaultVelocity = Math.min(defaultMaxVelocity, Constants.MAX_SPEED);
        for (int i = 0; i < velocityPerSegment.length; i++) {
            velocityPerSegment[i] = defaultVelocity;
        }

        if (constraints == null || !constraints.has("max_velocity_meters_per_sec")) {
            return;
        }

        JsonArray velocityConstraints = constraints.getAsJsonArray("max_velocity_meters_per_sec");
        for (JsonElement elem : velocityConstraints) {
            JsonObject constraint = elem.getAsJsonObject();
            double value = constraint.get("value").getAsDouble();
            int startOrdinal = constraint.get("start_ordinal").getAsInt();
            int endOrdinal = constraint.get("end_ordinal").getAsInt();

            // Apply constraint to range
            for (int i = startOrdinal; i <= endOrdinal && i < velocityPerSegment.length; i++) {
                velocityPerSegment[i] = value;
            }
        }
    }

    /**
     * Parses acceleration constraints and fills per-segment array.
     *
     * @param constraints The constraints JSON object
     * @param accelerationPerSegment The array to fill (output)
     */
    private static void parseAccelerationConstraints(JsonObject constraints, double[] accelerationPerSegment) {
        // Use default from config
        for (int i = 0; i < accelerationPerSegment.length; i++) {
            accelerationPerSegment[i] = defaultMaxAcceleration;
        }

        if (constraints == null || !constraints.has("max_acceleration_meters_per_sec2")) {
            return;
        }

        JsonArray accelConstraints = constraints.getAsJsonArray("max_acceleration_meters_per_sec2");
        for (JsonElement elem : accelConstraints) {
            JsonObject constraint = elem.getAsJsonObject();
            double value = constraint.get("value").getAsDouble();
            int startOrdinal = constraint.get("start_ordinal").getAsInt();
            int endOrdinal = constraint.get("end_ordinal").getAsInt();

            // Apply constraint to range
            for (int i = startOrdinal; i <= endOrdinal && i < accelerationPerSegment.length; i++) {
                accelerationPerSegment[i] = value;
            }
        }
    }

    /**
     * Flips a path for red alliance using 2026 field dimensions.
     * Performs a 180° rotation around the field center point (rotationally symmetric field).
     *
     * @param path The path to flip
     * @return The flipped path
     */
    private static ParsedPath flipPath(ParsedPath path) {
        List<PathWaypoint> flippedWaypoints = new ArrayList<>();

        for (PathWaypoint wp : path.getWaypoints()) {
            // 180° rotation around field center: flip both X and Y
            Translation2d flippedTranslation = new Translation2d(
                FIELD_LENGTH - wp.getTranslation().getX(),
                FIELD_WIDTH - wp.getTranslation().getY()
            );

            // Rotate heading by 180° (add π to the angle)
            Rotation2d flippedRotation = new Rotation2d(
                wp.getRotation().getRadians() + Math.PI
            );

            flippedWaypoints.add(new PathWaypoint(
                flippedTranslation,
                flippedRotation,
                wp.getHandoffRadius(),
                wp.hasRotationTarget()
            ));
        }

        return new ParsedPath(
            flippedWaypoints,
            path.getMaxVelocityPerSegment(),
            path.getMaxAccelerationPerSegment(),
            path.getEndTranslationTolerance(),
            path.getEndRotationTolerance(),
            path.getEndVelocityTolerance()
        );
    }

    /**
     * Mirrors a path across the Y-axis center of the field (for left/right symmetry).
     * X coordinates stay the same, Y coordinates flip, headings mirror across vertical axis.
     *
     * @param path The path to mirror
     * @return The mirrored path
     */
    public static ParsedPath mirrorPath(ParsedPath path) {
        List<PathWaypoint> mirroredWaypoints = new ArrayList<>();

        for (PathWaypoint wp : path.getWaypoints()) {
            // Mirror Y coordinate across field center, keep X the same
            Translation2d mirroredTranslation = new Translation2d(
                wp.getTranslation().getX(),
                FIELD_WIDTH - wp.getTranslation().getY()
            );

            // Mirror heading across vertical axis: heading_new = -heading_old
            // (0° stays 0°, 90° becomes 270°, 180° stays 180°, 270° becomes 90°)
            Rotation2d mirroredRotation = new Rotation2d(
                -wp.getRotation().getRadians()
            );

            mirroredWaypoints.add(new PathWaypoint(
                mirroredTranslation,
                mirroredRotation,
                wp.getHandoffRadius(),
                wp.hasRotationTarget()
            ));
        }

        return new ParsedPath(
            mirroredWaypoints,
            path.getMaxVelocityPerSegment(),
            path.getMaxAccelerationPerSegment(),
            path.getEndTranslationTolerance(),
            path.getEndRotationTolerance(),
            path.getEndVelocityTolerance()
        );
    }

    /**
     * Clears the path cache (useful for testing or reloading paths).
     */
    public static void clearCache() {
        bluePathCache.clear();
        redPathCache.clear();
    }

    /**
     * Pre-loads specific paths into cache for faster access during autonomous.
     * This eliminates file I/O lag when the match starts.
     *
     * @param pathNames List of path names to pre-load (without .json extension)
     * @param shouldFlip Whether to load for red alliance (true) or blue alliance (false)
     */
    public static void preloadPaths(List<String> pathNames, boolean shouldFlip) {
        for (String pathName : pathNames) {
            try {
                loadPath(pathName, shouldFlip); // This will cache the path
            } catch (Exception e) {
                System.err.println("BLinePathLoader: Failed to preload path '" + pathName + "': " + e.getMessage());
            }
        }
    }

    /**
     * Returns the total number of cached paths across both alliances.
     *
     * @return Total cached path count
     */
    public static int getCachedPathCount() {
        return bluePathCache.size() + redPathCache.size();
    }

    /**
     * Checks if a specific path is already cached.
     *
     * @param pathName The name of the path to check
     * @param shouldFlip Whether to check red alliance cache (true) or blue alliance cache (false)
     * @return True if the path is cached
     */
    public static boolean isPathCached(String pathName, boolean shouldFlip) {
        Map<String, ParsedPath> cache = shouldFlip ? redPathCache : bluePathCache;
        return cache.containsKey(pathName);
    }
}
