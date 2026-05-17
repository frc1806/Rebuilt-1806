package frc.robot.lib.pathfollowing;

/**
 * Simple wrapper class holding a path name.
 * Provides a drop-in replacement for BLine's Path class.
 * The actual path loading is done by BLinePathLoader.
 */
public class BLinePath {
    private final String pathName;

    /**
     * Creates a BLinePath with the specified name.
     * The path will be loaded from src/main/deploy/autos/paths/{pathName}.json
     *
     * @param pathName The name of the path (without .json extension)
     */
    public BLinePath(String pathName) {
        this.pathName = pathName;
    }

    /**
     * Gets the name of this path.
     *
     * @return The path name
     */
    public String getPathName() {
        return pathName;
    }

    @Override
    public String toString() {
        return "BLinePath(" + pathName + ")";
    }
}
